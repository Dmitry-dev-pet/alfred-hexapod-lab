"""Constructive *planar-only* quasi-static crawl witness for four point feet.

This is deliberately NOT a CAD30 IK, actuator-force, collision, or MuJoCo proof.
No external dependencies; every listed segment is certified by convexity.
"""
from __future__ import annotations
import argparse
import itertools
import json
import math
from pathlib import Path

V = tuple[float, float]


def add(a: V, b: V) -> V:
    return a[0] + b[0], a[1] + b[1]


def sub(a: V, b: V) -> V:
    return a[0] - b[0], a[1] - b[1]


def dot(a: V, b: V) -> float:
    return a[0]*b[0] + a[1]*b[1]


def cross(a: V, b: V) -> float:
    return a[0]*b[1] - a[1]*b[0]


def distance(a: V, b: V) -> float:
    return math.hypot(a[0]-b[0], a[1]-b[1])


def hull(points: list[V]) -> list[V]:
    """CCW convex hull, monotone chain; errors on collinear degenerate input."""
    pts = sorted(set(points))
    if len(pts) < 3:
        raise ValueError('Need >=3 distinct noncollinear feet')
    lower: list[V] = []
    for p in pts:
        while len(lower) >= 2 and cross(sub(lower[-1], lower[-2]), sub(p, lower[-1])) <= 1e-14:
            lower.pop()
        lower.append(p)
    upper: list[V] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(sub(upper[-1], upper[-2]), sub(p, upper[-1])) <= 1e-14:
            upper.pop()
        upper.append(p)
    out = lower[:-1] + upper[:-1]
    if len(out) < 3:
        raise ValueError('Degenerate collinear feet')
    return out


def signed_margin(poly: list[V], point: V) -> float:
    return min(cross(sub(poly[(i+1) % len(poly)], p), sub(point, p)) /
               distance(p, poly[(i+1) % len(poly)]) for i, p in enumerate(poly))


def clip_inward(poly: list[V], start: V, end: V, margin: float) -> list[V]:
    """Clip polygon to the interior left of oriented line with margin."""
    edge = sub(end, start)
    length = math.hypot(*edge)
    if length <= 1e-14:
        raise ValueError('Degenerate polygon edge')
    def d(p: V) -> float:
        return cross(edge, sub(p, start))/length - margin
    result: list[V] = []
    for i, next_p in enumerate(poly):
        prev_p = poly[i-1]
        da, db = d(prev_p), d(next_p)
        if (da < -1e-12 and db > 1e-12) or (db < -1e-12 and da > 1e-12):
            t = da / (da-db)
            result.append(add(prev_p, (t*(next_p[0]-prev_p[0]), t*(next_p[1]-prev_p[1]))))
        elif abs(da) <= 1e-12 and db < -1e-12:
            result.append(prev_p)
        elif abs(db) <= 1e-12 and da < -1e-12:
            result.append(next_p)
        if db >= -1e-12:
            result.append(next_p)
    unique: list[V] = []
    for p in result:
        if not unique or distance(p, unique[-1]) > 1e-12:
            unique.append(p)
    if len(unique)>1 and distance(unique[0], unique[-1]) <= 1e-12:
        unique.pop()
    return unique


def eroded_triangle(points: list[V], margin: float) -> list[V]:
    if margin <= 0: raise ValueError('Require strictly positive support margin')
    tri = hull(points)
    if len(tri)!=3: raise ValueError('Tripod points must form a nondegenerate triangle')
    poly=tri
    for i in range(3):
        poly=clip_inward(poly,tri[i],tri[(i+1)%3],margin)
        if not poly:return []
    if len(poly)<3:return []
    return hull(poly)


def project_to_polygon(point: V, poly: list[V]) -> V:
    if signed_margin(poly,point)>=-1e-12:return point
    best_point=None
    best_dist=float('inf')
    for i,a in enumerate(poly):
        b=poly[(i+1)%len(poly)]
        ab=sub(b,a)
        t=max(0.,min(1.,dot(sub(point,a),ab)/dot(ab,ab)))
        p=add(a,(t*ab[0],t*ab[1]))
        d=distance(p,point)
        if d<best_dist:
            best_point,best_dist=p,d
    assert best_point is not None
    return best_point


def vec_mm(v: V) -> list[float]:
    return [round(x*1000,4) for x in v]


def certificate(feet: dict[str,V], com0: V, order: tuple[str,...], delta_y: float, m:float) -> dict:
    if set(feet)!=set(order) or len(order)!=4 or len(set(order))!=4:
        raise ValueError('Order must be one permutation of all four foot IDs')
    if delta_y<=0:raise ValueError('Positive forward displacement is required')
    current=dict(feet)
    first_quad=hull(list(current.values()))
    if len(first_quad)!=4:raise ValueError('Requires a convex four-foot footprint')
    original_margin=signed_margin(first_quad,com0)
    if original_margin<m-1e-9:
        return {'status':'PLANAR_FAIL','reason':'initial_COM_outside_eroded_four_support',
                'initial_margin_mm':round(original_margin*1000,4)}
    here=com0
    total=0.
    shift_sizes=[]
    phases=[]
    segs=[]
    delta=(0.,delta_y)
    for idx, leg in enumerate(order):
        tripod=hull([current[k] for k in order if k!=leg])
        target_set=eroded_triangle(tripod,m)
        if not target_set:
            return {'status':'PLANAR_FAIL','reason':'eroded_tripod_empty','leg':leg,'phase':idx+1}
        # Closest to nominal COM translated by the elapsed fraction of the step.
        nominal=add(com0,(0.,delta_y*idx/4.))
        c=project_to_polygon(nominal,target_set)
        before_quad=hull(list(current.values()))
        bound=min(signed_margin(before_quad,here), signed_margin(before_quad,c))
        if bound < m-1e-9:
            return {'status':'PLANAR_FAIL','reason':'four_support_transition_margin','leg':leg,'bound_mm':bound*1000}
        segs.append({'kind':'four_stance_com_shift','before_leg':leg,
                     'from_mm':vec_mm(here),'to_mm':vec_mm(c),
                     'guaranteed_margin_mm':round(bound*1000,4)})
        shift=distance(here,c)
        total+=shift;shift_sizes.append(shift)
        margin=signed_margin(tripod,c)
        if margin < m-1e-9:raise AssertionError('Geometric certificate internal error')
        phases.append({'swing_leg':leg,'stance_legs':[k for k in order if k!=leg],
                       'COM_hold_mm':vec_mm(c),'minimum_margin_mm':round(margin*1000,4),
                       'new_foot_world_mm':vec_mm(add(current[leg],delta)),
                       'preceding_COM_shift_mm':round(shift*1000,4)})
        current[leg]=add(current[leg],delta)
        here=c
    final=add(com0,delta)
    final_poly=hull(list(current.values()))
    final_bound=min(signed_margin(final_poly,here),signed_margin(final_poly,final))
    if final_bound<m-1e-9:
        return {'status':'PLANAR_FAIL','reason':'terminal_four_support_margin','bound_mm':final_bound*1000}
    segs.append({'kind':'terminal_four_stance_com_shift','from_mm':vec_mm(here),'to_mm':vec_mm(final),
                 'guaranteed_margin_mm':round(final_bound*1000,4)})
    total+=distance(here,final)
    if any(distance(current[k],add(feet[k],delta))>1e-12 for k in order):
        raise AssertionError('Periodic foot translation check failed')
    return {'status':'PLANAR_GEOMETRY_CERTIFIED_ONLY',
            'important':'Proves ONLY planar COM/support-polygon existence; NOT robot IK/torques/forces/continuous 3D dynamics.',
            'order':list(order),'forward_world_y_mm':round(delta_y*1000,4),
            'requested_margin_mm':round(m*1000,4),'initial_COM_mm':vec_mm(com0),
            'final_COM_mm':vec_mm(final),
            'initial_four_support_margin_mm':round(original_margin*1000,4),
            'minimum_swing_tripod_margin_mm':min(p['minimum_margin_mm'] for p in phases),
            'min_four_support_segment_bound_mm':min(p['guaranteed_margin_mm'] for p in segs),
            'total_planar_COM_travel_mm':round(total*1000,4),
            'max_single_COM_shift_mm':round(max([*shift_sizes,distance(here,final)])*1000,4),
            'phases':phases,'segments':segs}


def search_orders(feet: dict[str,V], com0:V, delta_y:float, margin:float) -> dict:
    results=[]
    for order in itertools.permutations(feet):
        results.append(certificate(feet,com0,order,delta_y,margin))
    valid=[v for v in results if v['status']=='PLANAR_GEOMETRY_CERTIFIED_ONLY']
    if not valid:
        return {'status':'PLANAR_FAIL','n_orders_checked':len(results), 'failures':results}
    valid.sort(key=lambda v:(v['total_planar_COM_travel_mm'],v['max_single_COM_shift_mm'],v['order']))
    return {'status':'PLANAR_GEOMETRY_CERTIFIED_ONLY','n_orders_checked':len(results),
            'n_geometrically_accepted':len(valid),'best':valid[0],
            'order_summaries':[{'order':x['order'],'total_COM_travel_mm':x['total_planar_COM_travel_mm'],
                                'max_shift_mm':x['max_single_COM_shift_mm']} for x in valid]}


def run_fixture(path:Path,mm_step:float,mm_margin:float,order:list[str]|None)->dict:
    obj=json.loads(path.read_text(encoding='utf-8'))
    feet={k:(float(v[0]),float(v[1])) for k,v in obj['feet_world_xyz_m'].items()}
    com=tuple(float(v) for v in obj['COM_xyz_m'][:2])
    if order:
        return certificate(feet,com,tuple(order),mm_step*.001,mm_margin*.001)
    return search_orders(feet,com,mm_step*.001,mm_margin*.001)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--fixture',type=Path,required=True)
    parser.add_argument('--step-mm',type=float,default=20)
    parser.add_argument('--margin-mm',type=float,default=10)
    parser.add_argument('--order',nargs=4)
    parser.add_argument('--out',type=Path)
    args=parser.parse_args()
    result=run_fixture(args.fixture,args.step_mm,args.margin_mm,args.order)
    if args.out:args.out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'status':result['status'],'best':{k:v for k,v in result.get('best',result).items()
                     if k in ('order','total_planar_COM_travel_mm','min_four_support_segment_bound_mm',
                              'minimum_swing_tripod_margin_mm','max_single_COM_shift_mm')},
                      'tested_orders':result.get('n_orders_checked',1)},ensure_ascii=False,indent=2))