"""Bounded 2D *necessary* periodic gait search on observed CAD30 supports.
This is not a physics gait or a proof of existence. No root teleport during dynamics.
"""
import json,itertools,time, math,argparse
from pathlib import Path
import numpy as np
from scipy.optimize import linprog
from scipy.spatial import ConvexHull
import mujoco as mj
import sys,os
CAD_ROOT=Path(os.environ.get("CAD30_ROOT",str(Path(__file__).parent/"cad"))).resolve()
PHYSICAL_ROOT=Path(os.environ.get("CAD30_BASELINE_DIR",str(Path(__file__).parent/"physical"))).resolve()
sys.path.insert(0,str(CAD_ROOT))
from hexlab.stand_geometry_v04 import load_legs,tip_world
WALK=('ml','mr','bl','br')
BASE=Path(__file__).parent

def support_ineq(xys,exclude=None,margin=.010):
    legs=[k for k in WALK if k!=exclude]
    pts=np.stack([xys[k][:2] for k in legs]);h=ConvexHull(pts)
    return np.array([np.r_[eq[:2],-margin-eq[2]] for eq in h.equations])

def lp_period(foothold,order,com_actual,Delta=.020,margin=.010,prepare_rear_limit_m=.009):
    # decision x = (com0.x,com0.y, com_i.x,com_i.y i=1...4, d, prex, prey)
    # sampled exact 3-support COM margin at each planned lift; L∞ discrepancy to period progress <=d
    n=13; A=[];b=[];e=[];eb=[]
    feet={k:p.copy() for k,p in foothold.items()}
    def polygon(idx,poly):
        for a,b_,rhs in poly:
            row=np.zeros(n);row[2*idx:2*idx+2]=[a,b_];A.append(row);b.append(rhs)
    polygon(0,support_ineq(feet,margin=margin))
    for phase,k in enumerate(order,1):
        polygon(phase,support_ineq(feet,k,margin=margin))
        feet[k][1]+=Delta
        # ensure each COM waypoint can also remain stable in 4-foot touchdown pose
        polygon(phase,support_ineq(feet,margin=margin))
    for i in range(1,5):
        progress=i*Delta/4
        for axis in [0,1]:
            for sign in (-1,1):
                row=np.zeros(n);row[2*i+axis]=sign;row[axis]=-sign;row[10]=-1.
                A.append(row);b.append(sign*(progress if axis==1 else 0))
    # bound COM phase-to-phase differences <=6cm (Linf) using d, no extra arbitrary jump
    for i in range(1,5):
        for axis in [0,1]:
            for sign in [-1,1]:
                row=np.zeros(n);row[2*i+axis]=sign;row[2*(i-1)+axis]=-sign;A.append(row);b.append(.09)
    # original six COM y=+0.008, require after 3*20mm >=0.028; COM0Y>= -0.032
    bounds=[(float(com_actual[0]-.04),float(com_actual[0]+.04)),(float(max(com_actual[1]-prepare_rear_limit_m,-.032)),float(com_actual[1]+.04))]
    bounds+= [(float(com_actual[0]-.12),float(com_actual[0]+.12)),(float(com_actual[1]-.13),float(com_actual[1]+.13))]*4
    bounds +=[(0,.12),(0,.04),(0,.05)]
    # prep COM measured diff X/Y <= prep slack vars to minimize
    for axis,slack in [(0,11),(1,12)]:
        for sign in [-1,1]:
            row=np.zeros(n);row[axis]=sign;row[slack]=-1.; A.append(row);b.append(sign*com_actual[axis])
    obj=np.zeros(n);obj[10]=1.;obj[11]=.30;obj[12]=.6
    res=linprog(obj,A_ub=np.asarray(A),b_ub=np.asarray(b),bounds=bounds,method='highs')
    if not res.success:return None
    xc=res.x
    return {'order':order,'score_m':float(res.fun),'max_Linf_deviation_m':float(xc[10]),'pre_shift_abs_xy_m':float(xc[11]+xc[12]),'COM0_xy_m':xc[:2].tolist(),'COM_waypoints_xy_m':[xc[2*i:2*i+2].tolist() for i in range(1,5)], 'start_foot_xy_m':{k:p[:2].tolist() for k,p in foothold.items()}}


def run(limit=0):
    m=mj.MjModel.from_xml_path(str(CAD_ROOT/'source/mjcf/scene_local30_nominal.xml'));d=mj.MjData(m)
    st=json.loads((PHYSICAL_ROOT/'front_pair_shift35_p0.json').read_text());d.qpos[:]=st['final_qpos'];mj.mj_forward(m,d)
    leg={x['suffix']:x for x in load_legs(m,d,mj)}
    feet={k:tip_world(m,d,leg[k]).copy() for k in WALK}
    com=np.average(d.xipos,weights=m.body_mass,axis=0)
    base_feet={k:feet[k].copy() for k in WALK}
    records=[];stats={'tested':0,'feasible_2d':0,'order_counts':{}}
    start=time.time()
    # Foot relocations tested as simultaneously selected 4-foot stance, NOT physical prep yet.
    middle_in=[0,.02,.04,.06]
    middle_y=[-.02,0,.02]
    rear_out=[0,.02]
    rear_forward=[.03,.06,.09]
    orders=list(itertools.permutations(WALK))
    for mi,my,ro,rf in itertools.product(middle_in,middle_y,rear_out,rear_forward):
        target={k:feet[k].copy() for k in WALK}
        for k in ('ml','mr'):
            target[k][0]+=(mi if k=='ml' else -mi);target[k][1]+=my
        for k in ('bl','br'):
            target[k][0]+=(-ro if k=='bl' else ro);target[k][1]+=rf
        for order in orders:
            stats['tested']+=1
            r=lp_period(target,order,com[:2]);
            if r is None:continue
            stats['feasible_2d']+=1;stats['order_counts']['→'.join(order)]=stats['order_counts'].get('→'.join(order),0)+1
            r['adjustments_mm']={'middle_in':round(mi*1000),'middle_y':round(my*1000),'rear_out':round(ro*1000),'rear_forward':round(rf*1000)}
            # penalty for total prep foot motion to avoid sacrificing progress to long reset
            r['prep_toe_motion_mm']=float((2*(mi**2+my**2)**.5+2*(ro**2+rf**2)**.5)*1000)
            r['rank_score_m']=r['score_m']+.12*(r['prep_toe_motion_mm']/1000)
            records.append(r)
        if limit and stats['tested']>=limit:break
    records.sort(key=lambda r:r['rank_score_m'])
    result={'scenario':'bounded-2D-triangle-and-global-period-LP-v1','status':'GEOMETRIC_2D_ONLY_NOT_IK_DYNAMICS','model_mass_kg':float(m.body_mass.sum()),'measured_six_com_m':com.tolist(),'original_start_com_y_m':.008035,'constraints':{'period_delta_mm':20,'period_count':3,'minimum_static_margin_mm':10,'initial_COM_Y_min_m':max(com[1]-.009,-.032),'first_foot_placement_preparation_not_certified':True},'search_grid_mm':{'middle_in':[0,20,40,60],'middle_y':[-20,0,20],'rear_out':[0,20],'rear_forward':[30,60,90]},'tested_2d':stats['tested'],'feasible_2d':stats['feasible_2d'],'search_elapsed_s':round(time.time()-start,3),'best_candidates':records[:120],'all_candidates':records,'top10':records[:10],'order_counts':stats['order_counts']}
    (BASE/'search_2d_results.json').write_text(json.dumps(result,indent=2,ensure_ascii=False))
    print(json.dumps({'tested':stats['tested'],'feasible':stats['feasible_2d'],'seconds':result['search_elapsed_s'],'top':[{k:r[k] for k in ['order','adjustments_mm','max_Linf_deviation_m','rank_score_m','prep_toe_motion_mm','COM0_xy_m']} for r in records[:6]]},indent=2))
if __name__=='__main__':run()