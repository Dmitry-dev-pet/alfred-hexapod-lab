"""Independent sampled inverse-dynamics/point-contact feasibility gate.

This NEVER executes a physical MuJoCo trajectory: it differentiates proposed
kinematic waypoints, computes required generalized forces using MuJoCo mj_inverse,
and solves for consistent nonnegative contact forces and bounded joint torques.
This is a necessary sampled rigid-body model check, not contact dynamics,
not an optimal gait, and not a claim about a physical Freenove.
"""
from __future__ import annotations
import argparse, json, os, sys, zipfile
import numpy as np
from scipy.optimize import linprog
from pathlib import Path

WALK=('ml','mr','bl','br');ALL=(*WALK,'fl','fr')

def read_trace(z):
    with zipfile.ZipFile(z) as f:
        return json.loads(f.read('math_first_four_crawl_trace.json'))['nodes']

def evaluate(model_path,trace_zip,dt,tau_cap=.7,mu=1.1,max_nodes=218):
    import mujoco as mj
    from hexlab.stand_geometry_v04 import load_legs, tip_world
    model=mj.MjModel.from_xml_path(str(model_path)); data=mj.MjData(model)
    nodes=read_trace(trace_zip)[:max_nodes]
    q=np.asarray([n['qpos'] for n in nodes],dtype=float)
    nv=model.nv; n=len(nodes)
    if q.shape[1]!=model.nq:raise ValueError(f'state nq={q.shape[1]} != model nq={model.nq}')
    if n<3:raise ValueError('too few nodes')
    legs=None
    # Actual finite-difference tangent velocities; quaternion differences through MuJoCo.
    vel=np.zeros((n,nv)); mid=np.zeros((n-1,nv))
    for i in range(n-1): mj.mj_differentiatePos(model,mid[i],dt,q[i],q[i+1])
    vel[0]=mid[0];vel[-1]=mid[-1];vel[1:-1]=(mid[:-1]+mid[1:])/2
    accel=np.zeros_like(vel)
    accel[1:-1]=(mid[1:]-mid[:-1])/dt
    # Deliberately assume terminal velocities zero only if a hold exists; otherwise endpoint derivatives one-sided.
    # Generalized control matrix exactly the MuJoCo coordinate mapping for joint motors.
    adr=model.jnt_dofadr[model.actuator_trnid[:,0]]
    gear=np.asarray(model.actuator_gear[:,0]); nu=model.nu
    S=np.zeros((nv,nu)); S[adr,np.arange(nu)]=gear
    fail=[];records=[];max_raw_accel=0.; max_velocity=0.
    counts={}
    for i,(row,proposal) in enumerate(zip(q,nodes)):
        data.qpos[:]=row; data.qvel[:]=vel[i];data.qacc[:]=accel[i]
        mj.mj_forward(model,data)
        if legs is None:legs={l['suffix']:l for l in load_legs(model,data,mj)}
        # mj_inverse computes inverse dynamics *including the MuJoCo contact solver*,
        # so qfrc_inverse can be ~0 in grounded equilibrium; those contacts must
        # not be counted *again*. Use the unconstrained generalized forces instead:
        # rhs = M(q) qdd + bias(q,qd) - passive(q,qd).
        # We then solve for contacts as independent unknowns in the LP.
        M=np.zeros((nv,nv));mj.mj_fullM(model,data,M)
        rhs=M@accel[i]+data.qfrc_bias.copy()-data.qfrc_passive.copy()
        grounded=proposal['grounded']
        nf=len(grounded)
        J=np.zeros((nv,3*nf)); foot_vel={}
        for j,k in enumerate(grounded):
            l=legs[k]; point=tip_world(model,data,l)
            jp=np.zeros((3,nv));jr=np.zeros((3,nv))
            mj.mj_jac(model,data,jp,jr,point,l['body']);J[:,3*j:3*j+3]=jp.T
            foot_vel[k]=float(np.linalg.norm(jp@vel[i]))
        max_raw_accel=max(max_raw_accel,float(np.linalg.norm(accel[i])))
        max_velocity=max(max_velocity,max(foot_vel.values()))
        Aeq=np.column_stack((J,S,np.zeros(nv)))
        bounds=[(None,None),(None,None),(.2,None)]*nf+[(None,None)]*nu+[(0,tau_cap)]
        aub=[]
        for j in range(nf):
            for sx in (-1.,1.):
                for sy in (-1.,1.):
                    one=np.zeros(len(bounds));one[3*j]=sx;one[3*j+1]=sy;one[3*j+2]=-mu;aub.append(one)
        for j in range(nu):
            for sign in (-1.,1.):
                one=np.zeros(len(bounds));one[3*nf+j]=sign;one[-1]=-1;aub.append(one)
        objective=np.zeros(len(bounds));objective[-1]=1.
        fit=linprog(objective,A_ub=np.asarray(aub),b_ub=np.zeros(len(aub)),A_eq=Aeq,b_eq=rhs,bounds=bounds,method='highs')
        counts[proposal['phase']]=counts.get(proposal['phase'],0)+1
        item={'index':i,'phase':proposal['phase'],'grounded':grounded,
              'feasible':bool(fit.success),
              'max_stance_toe_speed_mps':max(foot_vel.values()),
              'root_y_m':float(row[1]),
              'max_abs_q_rad':float(np.max(np.abs(row[7:25]))),
              'max_generalized_acc_norm':float(np.linalg.norm(accel[i])),
              'status':fit.message}
        if fit.success:
            item['min_required_peak_motor_tau_Nm']=float(fit.x[-1]);item['min_fz_N']=float(np.min(fit.x[:3*nf].reshape(nf,3)[:,2]))
        else:
            # One fail enough to disqualify this proposed full-period trajectory.
            fail.append({'index':i,'phase':proposal['phase'],'reason':fit.message})
        records.append(item)
    no_period_dynamic_pass= len(fail)==0 and max_velocity<.01
    return {
        'schema':'cad30-periodic-proposed-reference-inverse-dynamics-gate-v1',
        'method':'MuJoCo full mass matrix M(q) and bias h(q,qd) on finite-differenced virtual poses, with per-node LP contact forces and motor commands; NOT forward simulation',
        'dt_per_virtual_node_s':dt,'nodes_tested':n,'six_to_four_preparation_included':True,
        'phase_counts':counts,'feasible_samples':sum(x['feasible'] for x in records),
        'first_nonaccepted':fail[0] if fail else None,'infeasible_or_numerically_undecided_samples':len(fail),
        'first_15_nonaccepted_samples':fail[:15],
        'max_stance_toe_speed_mps':max_velocity,'max_generalized_acc_norm':max_raw_accel,
        'all_samples_dynamic_lp_feasible':len(fail)==0,
        'g1_verified_period':False,'g2_free_base_simulated':False,'hardware_validated':False,
        'status':'INVERSE_DYNAMICS_SCREEN_ONLY_NEVER_GAIT_PASS',
        'caveat':'Point-contact LP ignores true unilateral impact physics and MJ constraint solver; differentiated virtual IK nodes have arbitrary timing and may not be trackable by PD motors.',
        'records':records,
    }

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--model',required=True);p.add_argument('--zip',required=True);p.add_argument('--dt',type=float,default=.15);p.add_argument('--out',required=True)
    x=p.parse_args();result=evaluate(x.model,x.zip,x.dt)
    Path(x.out).write_text(json.dumps(result,indent=2)+'\n')
    print({k:v for k,v in result.items() if k not in ('records','phase_counts')})