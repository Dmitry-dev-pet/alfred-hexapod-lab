"""CAD30 FNK0052: static contact-force feasibility under gravity and actuator limits.

This is a central *oracle*, not a decentralized walking controller. For one
held posture (six-leg settled stand), solve wrench balance with nonnegative
normal forces, conservative 4-facet friction pyramids and the model's nominal
actuator limits. A point force per tibia is placed at the force-weighted mean
of its current MuJoCo tibia-floor contact positions. All leg masses are kept.

mj_rne provides gravity generalized forces (NOT mj_inverse with contact
constraints included). The Jacobian converts each local contact-force vector
into generalized joint loads. The objective only chooses among equilibria;
all safety-relevant feasibility decisions come from linear constraints.

Require: the existing private CAD30 model at ROOT/source/mjcf/scene_local30_nominal.xml
and its first-party Python package ROOT/hexlab. This file contains NO CAD STL.
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import mujoco
import numpy as np
from scipy.optimize import linprog, minimize
from scipy.spatial import ConvexHull


def _root_path() -> Path:
    paths = [Path.cwd(), *Path(__file__).resolve().parents]
    for p in paths:
        if (p / 'hexlab' / 'stand_v04.py').is_file() and (p / 'source' / 'mjcf' / 'scene_local30_nominal.xml').is_file():
            return p
    raise FileNotFoundError('Restore private CAD30 source/mjcf and hexlab first')


def skew_cross(v: np.ndarray) -> np.ndarray:
    x,y,z = map(float, v)
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0]], dtype=float)


def support_margin_xy(pts: np.ndarray, com_xy: np.ndarray) -> float:
    """Signed closest-edge distance to convex support polygon; <0 means outside."""
    if len(pts) < 3:
        return float('-inf')
    # Qhull can fail on collinear support sets, which cannot support an area.
    try:
        hull=ConvexHull(np.asarray(pts,float))
        A=hull.equations[:,:2]; b=hull.equations[:,2]
        return float(np.min(-(A@com_xy+b)/np.linalg.norm(A,axis=1)))
    except Exception:
        return float('-inf')


@dataclass
class StandData:
    model: mujoco.MjModel
    data: mujoco.MjData
    codes: tuple[str,...]
    positions: np.ndarray
    measured: np.ndarray
    com: np.ndarray
    model_mass_kg: float
    h: np.ndarray
    tau_bound: np.ndarray
    G: np.ndarray
    nominal_tau: np.ndarray
    initial_torque_residual: float
    original_stand_proxy: dict


def load_stand(root: Path | None = None, duration_s: float=1.0) -> StandData:
    import sys
    root = root or _root_path()
    if str(root) not in sys.path:
        sys.path.insert(0,str(root))
    from hexlab.stand_geometry_v04 import load_legs
    from hexlab.stand_v04 import static_pose, hold_under_gravity
    model = mujoco.MjModel.from_xml_path(str(root / 'source/mjcf/scene_local30_nominal.xml'))
    data = mujoco.MjData(model)
    legs=load_legs(model,data,mujoco)
    static_pose(model,data,mujoco,legs,.145,.15)
    stand_pose=data.qpos.copy()
    proxy=hold_under_gravity(model,data,mujoco,stand_pose,duration_s)
    if proxy['last_0p4s_min_contact_legs']!=6:
        raise RuntimeError('Cannot create six-contact stand')
    codes=tuple(l['suffix'] for l in legs)
    com=(np.asarray(data.xipos)*np.asarray(model.body_mass)[:,None]).sum(axis=0)/model.body_mass.sum()
    contacts={c:[] for c in codes}
    floor_id=int(model.geom('floor').id)
    for k in range(data.ncon):
        c=data.contact[k]
        if int(c.geom1)==floor_id:
            gid=int(c.geom2)
        elif int(c.geom2)==floor_id:
            gid=int(c.geom1)
        else:
            continue
        gname=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,gid)
        if not gname.startswith('tibia') or not gname.endswith('_collision_0'):
            continue
        code=gname.removeprefix('tibia').removesuffix('_collision_0')
        f=np.zeros(6)
        mujoco.mj_contactForce(model,data,k,f)
        fw=c.frame.reshape(3,3).T@f[:3]
        contacts[code].append((np.asarray(c.pos).copy(),fw))
    if any(not contacts[c] for c in codes):
        raise RuntimeError('No ground contact for one or more tibia')
    positions=[];measured=[]
    for code in codes:
        fs=np.array([f for p,f in contacts[code]])
        pp=np.array([p for p,f in contacts[code]])
        F=fs.sum(axis=0)
        # Mean location weighted by Fz so the vertical support moment is kept.
        p=np.average(pp,axis=0,weights=np.maximum(fs[:,2],1e-8))
        positions.append(p);measured.append(F)
    positions=np.array(positions);measured=np.array(measured)
    # Removing explicit velocities and accelerations removes inertial torque.
    data.qvel[:]=0.;data.qacc[:]=0.
    mujoco.mj_forward(model,data)
    # This computes M(q)qdd+C(q,qdot) *without already subtracting contacts*.
    h_full=np.zeros(model.nv)
    mujoco.mj_rne(model,data,0,h_full)
    joint_ids=model.actuator_trnid[:,0].astype(int)
    dofs=model.jnt_dofadr[joint_ids].astype(int)
    if any(model.jnt_type[j]!=mujoco.mjtJoint.mjJNT_HINGE for j in joint_ids):
        raise RuntimeError('Unexpected motor joint type')
    bounds=model.actuator_ctrlrange.copy()
    if not np.allclose(bounds[:,0],-bounds[:,1]):
        raise RuntimeError('This research solver assumes symmetric motor limits')
    G=np.zeros((model.nu,3*len(codes)))
    for k,code in enumerate(codes):
        leg=next(l for l in legs if l['suffix']==code)
        jp=np.zeros((3,model.nv));jr=np.zeros((3,model.nv))
        mujoco.mj_jac(model,data,jp,jr,positions[k],int(leg['body']))
        G[:,3*k:3*k+3]=jp[:,dofs].T
    predicted=h_full[dofs]-G@measured.reshape(-1)
    actual=np.asarray(data.ctrl).copy()
    return StandData(model=model,data=data,codes=codes,positions=positions,measured=measured,
        com=com,model_mass_kg=float(model.body_mass.sum()),h=h_full[dofs].copy(),
        tau_bound=bounds[:,1].copy(),G=G,nominal_tau=predicted,
        initial_torque_residual=float(np.max(np.abs(predicted-actual))),
        original_stand_proxy=proxy)


def build_constraints(stand:StandData, lost:Sequence[str], mu:float, torque_cap:bool=True,
                      friction_approx:str='inner32') -> dict:
    if mu<0 or mu>5:raise ValueError('Unphysical friction coefficient')
    if friction_approx not in ('inner','inner32','outer32'):
        raise ValueError('friction_approx must be inner, inner32 or outer32')
    lost=tuple(lost)
    if len(set(lost))!=len(lost) or any(c not in stand.codes for c in lost):
        raise ValueError('Invalid leg subset')
    n=len(stand.codes);size=3*n
    eq=np.zeros((6,size));rhs=np.array([0.,0.,stand.model_mass_kg * -stand.model.opt.gravity[2],0.,0.,0.])
    for k in range(n):
        eq[:3,3*k:3*k+3]=np.eye(3)
        eq[3:,3*k:3*k+3]=skew_cross(stand.positions[k]-stand.com)
    # 'inner' is inscribed diamond |Fx|+|Fy| <= mu Fz. Feasible under
    # inner means physically Coulomb-cone-feasible in the frozen posture.
    # 'inner32' shrinks each tangent halfspace by cos(pi/32), so the
    # polygon is strictly inside the circular friction cone.
    # 'outer32' uses 32 circle tangent halfspaces. The true circular
    # cone is a subset of the outer approximation: if outer is infeasible,
    # NO exact Coulomb cone solution exists under the stated assumptions.
    ub=[];rhs_ub=[]
    for k in range(n):
        if friction_approx=='inner':
            directions=[(sx,sy) for sx in (-1.,1.) for sy in (-1.,1.)]
        else:
            directions=[(math.cos(2*math.pi*j/32),math.sin(2*math.pi*j/32)) for j in range(32)]
        pyramid_mu=mu*math.cos(math.pi/32) if friction_approx=='inner32' else mu
        for sx,sy in directions:
            a=np.zeros(size)
            a[3*k:3*k+3]=[sx,sy,-pyramid_mu]
            ub.append(a);rhs_ub.append(0.)
    if torque_cap:
        # tau = h - G f. Each actuator limit is ±model.actuator_ctrlrange[:,1].
        ub.extend(list(-stand.G));rhs_ub.extend(list(stand.tau_bound-stand.h))
        ub.extend(list( stand.G));rhs_ub.extend(list(stand.tau_bound+stand.h))
    bounds=[]
    for c in stand.codes:
        bounds.extend([(0.,0.)]*3 if c in lost else [(None,None),(None,None),(0,None)])
    return {'Aeq':eq,'beq':rhs,'Aub':np.array(ub),'bub':np.array(rhs_ub),
            'bounds':bounds,'lost':list(lost),'mu':mu,'torque_cap':torque_cap,'friction_approx':friction_approx}



def minimize_motor_fraction(stand:StandData,lost:Sequence[str],mu:float,
                            friction_approx:str='inner32') -> dict:
    """Find the smallest common scale t for all actuator limits.

    LP: minimize t such that -t*limit <= h - Gf <= t*limit,
    plus free-body wrench balance and conservative contact-friction polytope.
    t<1 means there exists a statically balanced force distribution with
    model motor margin; t>1 needs more nominal torque than our assumed cap.
    """
    p=build_constraints(stand,lost,mu,torque_cap=False,friction_approx=friction_approx)
    n=stand.G.shape[1];m=stand.model.nu
    base=np.asarray(p['Aub'])
    Aub=np.zeros((len(base)+2*m,n+1))
    Aub[:len(base),:n]=base
    Aub[len(base):len(base)+m,:n]=-stand.G
    Aub[len(base)+m:,:n]=stand.G
    Aub[len(base):,-1]=-np.r_[stand.tau_bound,stand.tau_bound]
    rhs=np.r_[p['bub'],-stand.h,stand.h]
    c=np.r_[np.zeros(n),1.]
    Aeq=np.c_[p['Aeq'],np.zeros(len(p['beq']))]
    res=linprog(c,A_eq=Aeq,b_eq=p['beq'],A_ub=Aub,b_ub=rhs,
      bounds=[*p['bounds'],(0,None)],method='highs')
    if not res.success:return {'feasible_for_some_motor_scale':False,'reason':res.message}
    scale=float(res.x[-1]);return {
      'feasible_for_some_motor_scale':True,
      'min_motor_fraction':round(scale,7),
      'optimistic_remaining_motor_fraction':round(1-scale,7),
      'torque_Nm_values_at_optimum':np.round(stand.h-stand.G@res.x[:-1],5).tolist(),
      'force_solution_world_N':np.round(res.x[:-1].reshape(-1,3),5).tolist()}


def solve_stand(stand:StandData, lost:Sequence[str], mu:float=1.1,
                torque_cap:bool=True, friction_approx:str='inner32') -> dict:
    problem=build_constraints(stand,lost,mu,torque_cap=torque_cap,friction_approx=friction_approx)
    lp=linprog(np.zeros(3*len(stand.codes)), A_eq=problem['Aeq'],b_eq=problem['beq'],
               A_ub=problem['Aub'], b_ub=problem['bub'],bounds=problem['bounds'],method='highs',
               options={'primal_feasibility_tolerance':1e-9,'dual_feasibility_tolerance':1e-9})
    output={'removed_CAD_legs':list(lost),'friction_mu':mu,'full_joint_torque_limits':torque_cap,
      'friction_approx':friction_approx,
      'support_polygon_margin_mm':round(1000*support_margin_xy(
              np.array([stand.positions[i,:2] for i,c in enumerate(stand.codes) if c not in lost]), stand.com[:2]),3),
      'LP_feasible':bool(lp.success),'status':lp.message}
    if torque_cap:
        output['minimum_required_motor_fraction']=minimize_motor_fraction(stand,lost,mu,friction_approx)
    if not lp.success:
        return output
    # Strictly convex QP, centered on measured intact-stand support forces.
    # SLSQP is used only for selecting a feasible *preferred* load distribution;
    # the LP feasibility certificate already exists independently.
    ref=stand.measured.flatten()
    weights=np.tile(np.array([2.,2.,1.]),len(stand.codes))
    def obj(x):return 0.5*float(np.dot(weights*(x-ref),x-ref))
    def grad(x):return weights*(x-ref)
    result=minimize(obj,lp.x,jac=grad,method='SLSQP',bounds=problem['bounds'],
        constraints=[{'type':'eq','fun':lambda x:problem['Aeq']@x-problem['beq'],
                      'jac':lambda x:problem['Aeq']},
                     {'type':'ineq','fun':lambda x:problem['bub']-problem['Aub']@x,
                      'jac':lambda x:-problem['Aub']}],
        options={'ftol':1e-10,'maxiter':450})
    x=result.x if result.success else lp.x
    r_eq=np.max(np.abs(problem['Aeq']@x-problem['beq']))
    r_ineq=max(0.,float(np.max(problem['Aub']@x-problem['bub'])))
    if r_eq>1e-5 or r_ineq>1e-6:
        raise RuntimeError(f'Invalid force result: eq {r_eq:.4g}, constraint {r_ineq:.4g}')
    force=x.reshape(-1,3)
    tau=stand.h-stand.G@x
    ratios=np.abs(tau)/stand.tau_bound
    friction_ratios=[float(np.linalg.norm(f[:2])/(mu*f[2])) if mu>0 and f[2]>1e-7 else None for f in force]
    output.update({'minimizer':'SLSQP' if result.success else 'linear_feasible_solution',
            'objective':round(obj(x),7),
            'forces_N':{code:[round(float(v),6) for v in force[k]] for k,code in enumerate(stand.codes)},
            'max_abs_actuator_tau_Nm':round(float(np.max(np.abs(tau))),6),
            'max_actuator_tau_fraction':round(float(np.max(ratios)),6),
            'min_tau_headroom_Nm':round(float(np.min(stand.tau_bound-np.abs(tau))),6),
            'max_friction_cone_utilization':round(max([v for v in friction_ratios if v is not None] or [0.]),6),
            'force_balance_error_N':round(float(np.max(np.abs(problem['Aeq']@x-problem['beq'])[:3])),10),
            'torque_balance_error_Nm':round(float(np.max(np.abs(problem['Aeq']@x-problem['beq'])[3:])),10),
            'constraint_max_violation':round(r_ineq,10),
            'vertical_forces_N':{code:round(float(force[k,2]),5) for k,code in enumerate(stand.codes)},
            'tau_Nm_by_actuator':{str(i):round(float(v),6) for i,v in enumerate(tau)},
            })
    return output



def solve_exact_circular_cone(stand:StandData, lost:Sequence[str],mu:float) -> dict:
    """SLSQP feasibility witness for circular Coulomb cone, seeded from outer LP.

    Only a NUMERICAL witness if residual tolerances pass; an SLSQP failure is
    inconclusive and is never presented as a proof of infeasibility.
    """
    problem=build_constraints(stand,lost,mu,friction_approx='outer32')
    lp=linprog(np.zeros(3*len(stand.codes)),A_eq=problem['Aeq'],b_eq=problem['beq'],
      A_ub=problem['Aub'],b_ub=problem['bub'],bounds=problem['bounds'],method='highs')
    if not lp.success:
        return {'numerically_feasible':False,'outer32_feasible':False,'reason':'outer LP infeasible'}
    x0=lp.x.copy()
    def f_cone(x):
        v=x.reshape(-1,3)
        return mu*v[:,2] - np.hypot(v[:,0],v[:,1])
    def j_cone(x):
        J=np.zeros((len(stand.codes),len(x)))
        for i,v in enumerate(x.reshape(-1,3)):
            r=math.hypot(v[0],v[1]);J[i,3*i:3*i+3]=[-v[0]/r if r>1e-12 else 0.,-v[1]/r if r>1e-12 else 0.,mu]
        return J
    res=minimize(lambda x: .5*float(np.sum((x-x0)**2)),x0,
      jac=lambda x: x-x0,method='SLSQP',bounds=problem['bounds'],
      constraints=[
        {'type':'eq','fun':lambda x:problem['Aeq']@x-problem['beq'],
         'jac':lambda x:problem['Aeq']},
        {'type':'ineq','fun':lambda x:problem['bub']-problem['Aub']@x,
         'jac':lambda x:-problem['Aub']},
        {'type':'ineq','fun':f_cone,'jac':j_cone}],
      options={'ftol':1e-12,'maxiter':800})
    x=res.x
    max_eq=float(np.max(np.abs(problem['Aeq']@x-problem['beq'])))
    max_ineq=max(0.,float(np.max(problem['Aub']@x-problem['bub'])))
    max_cone=max(0.,-float(np.min(f_cone(x))))
    feasible=all(v<1e-7 for v in (max_eq,max_ineq,max_cone))
    tau=stand.h-stand.G@x
    return {'numerically_feasible':feasible,'outer32_feasible':True,'SLSQP_success':bool(res.success),
      'force_balance_residual':round(max_eq,12),'linear_inequality_violation':round(max_ineq,12),
      'exact_cone_violation':round(max_cone,12),
      'max_tau_fraction':round(float(np.max(np.abs(tau)/stand.tau_bound)),8),
      'forces_world_N':{c:np.round(x.reshape(-1,3)[i],8).tolist() for i,c in enumerate(stand.codes)},
      'warning':'Boundary solution is not a robust safety margin' }


def run_all(root:Path|None=None,mus:tuple[float,...]=(1.1,0.6,0.3)) -> dict:
    stand=load_stand(root)
    combos=[(),*( (c,) for c in stand.codes),*itertools.combinations(stand.codes,2)]
    scenarios=[]
    for mu in mus:
        for lost in combos:
            case=solve_stand(stand,lost,mu)
            # Diagnose infeasible geometry vs motor constraints, at fixed point-of-contact.
            if not case['LP_feasible']:
                without_motors=solve_stand(stand,lost,mu,torque_cap=False)
                case['feasible_ignoring_motor_limits']=without_motors['LP_feasible']
                outer=solve_stand(stand,lost,mu,friction_approx='outer32')
                case['outer32_feasible']=outer['LP_feasible']
                case['diagnosis'] = ('infeasible_even_with_outer_friction_cone'
                    if not outer['LP_feasible'] else 'inconclusive_inner_vs_outer_cone')
                if outer['LP_feasible']:
                    exact=solve_exact_circular_cone(stand,lost,mu)
                    case['exact_cone_numerical_witness']=exact
                    if exact['numerically_feasible']:
                        case['diagnosis']='circular_cone_feasible_numerical_boundary'
            scenarios.append(case)
    out={
      'schema':'fnk0052_CAD30_stance_wrench_linear_feasibility/1',
      'model':'CAD30 nominal, 30 source meshes, 2 visual proxies, static freeze after 1-s torque-bounded stand',
      'physics':f'MuJoCo {mujoco.__version__}',
      'robot_mass_kg':round(stand.model_mass_kg,8),
      'g_ms2':float(-stand.model.opt.gravity[2]),
      'center_of_mass_xyz_m':np.round(stand.com,8).tolist(),
      'contact_point_world_xyz_m':{c:np.round(stand.positions[i],8).tolist() for i,c in enumerate(stand.codes)},
      'measured_intact_reaction_force_world_N':{c:np.round(stand.measured[i],6).tolist() for i,c in enumerate(stand.codes)},
      'initial_PD_torque_vs_inverse_dynamics_max_abs_error_Nm':round(stand.initial_torque_residual,6),
      'mu_values':list(mus),'scenario_count':len(scenarios),'scenarios':scenarios,
      'assumptions':[
        'STATIC force/wrench balance only in ONE fixed flat-ground stance; not walking stability',
        'Each tibia two MuJoCo contact points condensed into one force-weighted representative point',
        'Friction approximation uses 32-facet INSCRIBED polygon: (u_j dot F_t) <= mu cos(pi/32) Fz; guaranteed within Coulomb cone',
        'All masses retained even when specified foot cannot push on ground (loss of support, not amputation)',
        'Forces chosen by centralized quadratic minimization; local six-leg controller NOT implemented',
        'Joint moments τ = mj_rne_gravity - sum(J_tip^T F_tip), with model actuator cap ±0.7 Nm leg/±0.2 Nm head',
        'No motor speed, thermal, PWM, compliance, slip under motion, IMU delay or contact sensor uncertainty',
        'Feasible is necessary for quasi-static support (under this contact model), not sufficient for safe lift',
        'LP infeasible with inner 32-face polygon may be a false negative for exact Coulomb cone; outer32 infeasibility certifies under stated assumptions',
        'Exact-cone SLSQP witness, if found, is numerical evidence of feasibility at a possibly zero-margin boundary' 
      ]
    }
    return out


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,default=Path('results/cad30_static_support_force_report_2026-11-04.json'))
    p.add_argument('--mu',nargs='*',type=float,default=[1.1,0.6,0.3])
    args=p.parse_args()
    result=run_all(mus=tuple(args.mu))
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    for mu in result['mu_values']:
        cases=[c for c in result['scenarios'] if c['friction_mu']==mu]
        print(f'MU={mu}, of {len(cases)} support sets, feasible={sum(c["LP_feasible"] for c in cases)}')
        for case in cases:
            if len(case['removed_CAD_legs']) in [1,2]:
                print('/'.join(case['removed_CAD_legs']).ljust(7),case['LP_feasible'],str(case['support_polygon_margin_mm']).rjust(8),
                      f'tau_frac={case.get("max_actuator_tau_fraction",None)}',
                      f'ignore_motors={case.get("feasible_ignoring_motor_limits",None)}')
    print('OUTPUT',args.out)
if __name__=='__main__':main()