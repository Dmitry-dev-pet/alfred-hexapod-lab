"""Independent free-base MuJoCo physical rollout of ONE Mink kinematic candidate.

This program intentionally DOES NOT permit autonomous execution of an unproven
4-step schedule. It can reveal contact failures after nominal snapshot gates.
No root pinning / kinematic teleport is performed during physics integration.
"""
from __future__ import annotations
import argparse,json
from pathlib import Path
import numpy as np
import mujoco as mj
from scipy.spatial import ConvexHull
from mink_contact_preflight import load_context,StepPreflight,STANCE,PREFIX,com_xyz


def physical_trial(ctx, leg='bl', node_duration=.18, hold_s=.35, mu=1.1, stride=.020):
    if leg not in ('bl','br'):raise ValueError('Choose one rear foot for this diagnostic')
    plan=StepPreflight(ctx).attempt_rear_step(leg,body_xyz_m=(0.,0.,.024),foothold_forward_m=stride)
    if plan['status']!='ONE_STEP_STATIC_NODES_PASS':
        return {'status':'MATH_PREFLIGHT_REJECTED','blocking_node':plan.get('blocking_node')}
    m=ctx.model
    m.opt.timestep=.002
    m.geom_friction[m.geom('floor').id,0]=mu
    for foot in (*STANCE,'fl','fr'):
        m.geom_friction[m.geom(f'tibia{foot}_collision_0').id,0]=mu
    d=mj.MjData(m);d.qpos[:]=ctx.qpos;d.qvel[:]=ctx.qvel;mj.mj_forward(m,d)
    jids=np.asarray(m.actuator_trnid[:,0],int)
    qadr=np.asarray(m.jnt_qposadr[jids],int)
    dadr=np.asarray(m.jnt_dofadr[jids],int)
    cap=np.asarray(m.actuator_ctrlrange[:,1])
    floor=int(m.geom('floor').id)
    geom_leg={int(m.geom(f'tibia{k}_collision_0').id):k for k in (*STANCE,'fl','fr')}
    foot_sites={k:int(m.site(PREFIX+k).id) for k in STANCE}
    initial_foot={k:d.site_xpos[i].copy() for k,i in foot_sites.items()}
    start_com=com_xyz(m,d).copy()
    min_loaded=6;min_margin=float('inf');min_normal=float('inf');max_tau=0.;max_absq=0.;saturation=0;loss_steps=0
    lift_observed=False;land_observed=False;all_four_end=False;trace=[]
    def step(target,phase):
        nonlocal min_loaded,min_margin,min_normal,max_tau,max_absq,saturation,loss_steps,lift_observed,land_observed
        u=7.*(target-d.qpos[qadr])-.32*d.qvel[dadr]
        saturation+=int(np.any(abs(u)>cap+1e-8))
        d.ctrl[:]=np.clip(u,-cap,cap)
        mj.mj_step(m,d)
        forces={k:0. for k in (*STANCE,'fl','fr')};pts={k:[] for k in forces}
        for idx in range(d.ncon):
            c=d.contact[idx]
            gid=int(c.geom2) if int(c.geom1)==floor else int(c.geom1) if int(c.geom2)==floor else -1
            if gid not in geom_leg:continue
            k=geom_leg[gid];w=np.zeros(6);mj.mj_contactForce(m,d,idx,w)
            f=c.frame.reshape(3,3).T@w[:3]
            forces[k]+=float(f[2]);pts[k].append((c.pos.copy(),max(float(f[2]),0.)))
        loaded=[k for k in STANCE if forces[k]>.08]
        if len(loaded)<3:loss_steps+=1
        com=com_xyz(m,d)
        if len(loaded)>=3:
            pos=np.array([np.average([p for p,w in pts[k]],axis=0,
                weights=[max(w,1e-8) for p,w in pts[k]])[:2] for k in loaded]);eq=ConvexHull(pos).equations
            margin=float(np.min(-(eq[:,:2]@com[:2]+eq[:,2]))*1000)
        else: margin=float('-inf')
        min_loaded=min(min_loaded,len(loaded));min_margin=min(min_margin,margin)
        if loaded:min_normal=min(min_normal,*(forces[k] for k in loaded))
        max_tau=max(max_tau,float(max(abs(d.ctrl))));max_absq=max(max_absq,float(max(abs(d.qpos[ctx.joint_qpos_addrs]))))
        lifted_now=forces[leg]<.08 and d.site_xpos[foot_sites[leg],2]-initial_foot[leg][2]>.003
        lift_observed|=bool(lifted_now)
        landed_now=forces[leg]>.08 and phase.startswith(('touchdown_','end_hold'))
        land_observed|=bool(landed_now)
        if int(round(d.time/.002))%25==0:
            trace.append({'t_s':round(float(d.time),3),'phase':phase,'support':loaded,
                'margin_mm':round(margin,5) if np.isfinite(margin) else None,
                'Fz_N':{k:round(forces[k],4) for k in STANCE},
                'COM_y_mm':round(com[1]*1000,4),
                'world_foot_z_mm':round(float(d.site_xpos[foot_sites[leg],2]*1000),4),
                'world_foot_y_mm':round(float(d.site_xpos[foot_sites[leg],1]*1000),4)})
        return loaded
    goal=d.qpos[qadr].copy()
    for _ in range(round(.15/.002)):step(goal,'settle')
    for node in plan['nodes']:
        desired=np.asarray(node['qpos'])[qadr]
        begin=goal.copy();n=round(node_duration/.002)
        for i in range(n):
            a=(i+1)/n;t=a*a*(3-2*a)
            step(begin+(desired-begin)*t,node['phase'])
        goal=desired
    for _ in range(round(hold_s/.002)):step(goal,'end_hold')
    end_com=com_xyz(m,d)
    end_loaded=trace[-1]['support'] if trace else []
    final_foot=d.site_xpos[foot_sites[leg]].copy()
    # This extra requirement is decisive for receding-horizon replanning:
    # all four feet must be loaded again before authorizing another step.
    passed=all((min_loaded>=3,min_margin>0, max_absq<=1.700001,saturation==0,
                 lift_observed,land_observed,len(end_loaded)==4))
    return {'status':'DYNAMIC_STEP_PASS' if passed else 'DYNAMIC_VETO_NEXT_STEP',
            'math_preflight_nodes':len(plan['nodes']),'physics_step_s':.002,
            'time_s':float(d.time),'min_loaded_stance_legs':min_loaded,
            'min_signed_margin_mm':min_margin,'min_positive_loaded_Fz_N':min_normal,
            'maximum_joint_abs_rad':max_absq,'peak_motor_command_Nm':max_tau,
            'saturation_steps':saturation,'steps_with_less_than_3_loaded':loss_steps,
            'lift_observed':lift_observed,'touchdown_observed':land_observed,
            'end_loaded_legs':end_loaded,'start_COM':start_com.tolist(),
            'end_COM':end_com.tolist(),'COM_forward_delta_mm':float((end_com[1]-start_com[1])*1000),
            'actual_foot_forward_delta_mm':float((final_foot[1]-initial_foot[leg][1])*1000),
            'end_contact_site_z_mm':{k:float(d.site_xpos[foot_sites[k],2]*1000) for k in STANCE},
            'end_qpos':d.qpos.copy().tolist(),
            'end_qvel':d.qvel.copy().tolist(),
            'autonomous_next_step_authorized':False,
            'notes':'Only one exploratory leg, no four-step lookahead. Dynamic feedback not used to correct unexpected contact loss. Not hardware safe.',
            'trace_50ms':trace}


def main():
    p=argparse.ArgumentParser();p.add_argument('--model',type=Path,required=True)
    p.add_argument('--safe-state-json',type=Path,required=True);p.add_argument('--leg',choices=('bl','br'),default='bl')
    p.add_argument('--node-duration',type=float,default=.18);p.add_argument('--out',type=Path)
    args=p.parse_args();ctx=load_context(args.model,args.safe_state_json)
    out=physical_trial(ctx,args.leg,args.node_duration)
    txt=json.dumps(out,ensure_ascii=False,indent=2)
    if args.out:args.out.parent.mkdir(parents=True,exist_ok=True);args.out.write_text(txt+'\n')
    print(json.dumps({k:v for k,v in out.items() if k!='trace_50ms'},ensure_ascii=False,indent=2))

if __name__=='__main__':main()