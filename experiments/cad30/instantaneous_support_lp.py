"""Research-only CAD30 instantaneous rigid point-contact inverse-dynamics LP.

This is a *local linear feasibility screen*, NOT a gait, MPC rollout,
contact-complementarity solver, or hardware certificate. In particular it
assumes manually selected feet are (hypothetically) unloaded at the given pose.
"""
from __future__ import annotations
import itertools, math, json
from pathlib import Path

import mujoco as mj
import numpy as np
from scipy.optimize import linprog

LEGS=('ml','mr','bl','br')

def solve_pose(model: mj.MjModel, qpos, support, *, foot_site_ids,
               ang_cap=12., motor_utilization=.9, joint_accel_cap=35.,
               base_linear_cap=1., swing_up_accel=.5, mu=1.1,
               forward_accel=0., min_fz=.2, minimize_ang=False):
    """Dynamics Mqdd+h=S*tau+J.T*f, stance Jqdd≈0, friction/limits.

    This is not a full contact solver: it models one normal+tangential point
    force per supporting tibia and uses Jdot*qvel≈0 for qvel=0 snapshots.
    """
    support=tuple(support)
    if not set(support).issubset(LEGS) or len(set(support))!=len(support):
        raise ValueError('Invalid support set')
    if len(support)<2 or len(support)>4:raise ValueError('Only 2–4 contact study')
    n=len(support);d=mj.MjData(model);d.qpos[:]=qpos;d.qvel[:]=0.;mj.mj_forward(model,d)
    nv=model.nv;nu=model.nu
    M=np.zeros((nv,nv));mj.mj_fullM(model,d,M)
    S=np.zeros((nv,nu))
    for i,j in enumerate(model.actuator_trnid[:,0].astype(int)):
        S[int(model.jnt_dofadr[j]),i]=model.actuator_gear[i,0]
    caps=np.maximum(abs(model.actuator_ctrlrange[:,0]),abs(model.actuator_ctrlrange[:,1]))
    J={}
    for k in LEGS:
        jp=np.zeros((3,nv));jr=np.zeros((3,nv))
        mj.mj_jacSite(model,d,jp,jr,foot_site_ids[k]);J[k]=jp
    # x := qdd(nv), torque(nu), world point contact forces(3*n), angle bound scalar(optional)
    iu=nv;iff=iu+nu;it=iff+3*n;dim=it+int(minimize_ang)
    eq=np.zeros((nv+3*n+1,dim));rhs=np.zeros(eq.shape[0]);eq[:nv,:nv]=M
    eq[:nv,iu:iff]=-S
    rhs[:nv]=d.qfrc_passive-d.qfrc_bias
    for i,k in enumerate(support):
        eq[:nv,iff+3*i:iff+3*i+3]=-J[k].T
        eq[nv+3*i:nv+3*i+3,:nv]=J[k]
    eq[-1,1]=1.;rhs[-1]=forward_accel
    aub=[];bub=[]
    for k in LEGS:
        if k in support:continue
        row=np.zeros(dim);row[:nv]=-J[k][2];aub.append(row);bub.append(-swing_up_accel)
    for i in range(n):
        for theta in np.linspace(0,2*math.pi,16,endpoint=False):
            row=np.zeros(dim);row[iff+3*i:iff+3*i+3]=[math.cos(theta),math.sin(theta),-mu*math.cos(math.pi/16)]
            aub.append(row);bub.append(0.)
    if minimize_ang:
        for axis in range(3,6):
            for sign in (-1.,1.):
                row=np.zeros(dim);row[axis]=sign;row[it]=-1
                aub.append(row);bub.append(0.)
    bounds=([(-base_linear_cap,base_linear_cap)]*3+
            [(-ang_cap,ang_cap)]*3+
            [(-joint_accel_cap,joint_accel_cap)]*(nv-6)+
            [(-motor_utilization*cap,motor_utilization*cap) for cap in caps]+
            sum(([(None,None),(None,None),(min_fz,None)] for _ in support),[]))
    if minimize_ang:bounds.append((0.,ang_cap))
    objective=np.zeros(dim)
    if minimize_ang:objective[it]=1.
    res=linprog(objective,A_eq=eq,b_eq=rhs,A_ub=np.asarray(aub),b_ub=np.asarray(bub),bounds=bounds,method='highs')
    if not res.success:
        return {'feasible':False,'status':int(res.status),'message':res.message}
    qdd=res.x[:nv];tau=res.x[iu:iff];fz=res.x[iff:it].reshape(-1,3)
    out={'feasible':True,'angular_accel_max_rad_s2':round(float(np.max(abs(qdd[3:6]))),5),
         'angular_accel_local_xyz_rad_s2':[round(float(x),5) for x in qdd[3:6]],
         'base_accel_world_xyz_m_s2':[round(float(x),5) for x in qdd[:3]],
         'max_abs_joint_accel_rad_s2':round(float(np.max(abs(qdd[6:]))),4),
         'max_motor_fraction':round(float(np.max(abs(tau)/caps)),5),
         'forces_N':{k:np.around(v,4).tolist() for k,v in zip(support,fz)},
         'dynamics_residual':float(np.max(abs(eq@res.x-rhs))),
         'contact_inequality_residual':float(np.max(np.asarray(aub)@res.x-np.asarray(bub)))}
    if minimize_ang:out['minimal_ang_accel_bound']=round(float(res.x[it]),5)
    return out


def make_survey(model, poses,foot_ids,**kwargs):
    return {str(shift):{
        '/'.join(s):solve_pose(model,q,s,foot_site_ids=foot_ids,**kwargs)
        for n in (4,3,2) for s in itertools.combinations(LEGS,n)
    } for shift,q in poses.items()}


def main():
    import argparse
    from mink_contact_preflight import load_context,StepPreflight
    parser=argparse.ArgumentParser()
    parser.add_argument('--model',type=Path,required=True)
    parser.add_argument('--state',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    ctx=load_context(args.model,args.state)
    ids={k:int(ctx.model.site('planning_contact_'+k).id) for k in LEGS}
    planner=StepPreflight(ctx)
    rows={}
    for shift in range(0,14,2):
        if shift:
            pre=planner.checkpoint(f'forward_{shift}mm',LEGS,base_delta=(0.,.002,0.))
            if not pre['gate_passed']:
                rows[str(shift)]={'status':'prebalance_failed','gate':pre['phase']}
                break
        cases={}
        for n in (4,3,2):
            for support in itertools.combinations(LEGS,n):
                x=solve_pose(ctx.model,planner.c.q,support,foot_site_ids=ids,ang_cap=40.,
                             base_linear_cap=1.,joint_accel_cap=35.,motor_utilization=.9,
                             swing_up_accel=.5,minimize_ang=True,mu=1.1)
                cases['/'.join(support)]=x
        rows[str(shift)]={'status':'virtual_four_foot_kinematics','max_joint_rad':float(np.max(abs(planner.c.q[ctx.joint_qpos_addrs]))),'cases':cases}
    report={'schema':'CAD30_2_3_4_instantaneous_point_contact_feasibility_v1',
            'model':'Freenove FNK0052 CAD30 nominal; mass/inertia/friction/servos not hardware-calibrated',
            'foot_contacts':'hypothetical one point force at center of actual tibia contact for each supporting foot',
            'planning':'virtual forward COM translation while all 4 feet fixed; Mink IK and gravity-only preflight',
            'assumptions':{'joint_accel_bound_rad_s2':35.,'body_linear_accel_bound_m_s2':1.,
              'body_angular_accel_search_limit_rad_s2':40.,'model_torque_utilization':.9,
              'swing_foot_min_up_accel_m_s2':.5,'friction_coefficient':1.1,
              'stance_site_accel_approx':'Jqdd=0, evaluated with qvel=0; no qdot Jacobian term',
              'not_a_gait':True,'no_dynamic_rollout':True},'poses':rows}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'output':str(args.output),'poses':len(rows),
           'middle_pair_min_angular_accel_by_forward_shift':{k:v.get('cases',{}).get('ml/mr',{}).get('minimal_ang_accel_bound') for k,v in rows.items()}},indent=2))

if __name__=='__main__':main()
