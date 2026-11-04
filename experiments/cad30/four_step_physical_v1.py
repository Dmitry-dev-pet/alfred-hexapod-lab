"""Independent measured free-base MuJoCo physical check for 4-foot math waypoints.

No root qpos targets or pins/teleports; only actuator PD commands. Autonomous
phase execution is vetoed on contact/torque/angle/margin violations, and after
an expected foot-touchdown if four load-bearing contacts fail to reappear.
"""
from __future__ import annotations
import argparse,json,math
from pathlib import Path
import numpy as np
import mujoco as mj
from scipy.spatial import ConvexHull
from mink_contact_preflight import STANCE,PREFIX,load_context,com_xyz


def rollout(ctx,plan,*,node_seconds=.18,mu=1.1,settle_seconds=.15,final_hold_seconds=.25):
    model=ctx.model
    model.opt.timestep=.002
    model.geom_friction[model.geom('floor').id,0]=mu
    for leg in (*STANCE,'fl','fr'):
        model.geom_friction[model.geom(f'tibia{leg}_collision_0').id,0]=mu
    data=mj.MjData(model);data.qpos[:]=ctx.qpos;data.qvel[:]=ctx.qvel;mj.mj_forward(model,data)
    mids=np.asarray(model.actuator_trnid[:,0],dtype=int)
    qadr=np.asarray(model.jnt_qposadr[mids],dtype=int)
    dadr=np.asarray(model.jnt_dofadr[mids],dtype=int)
    caps=np.asarray(model.actuator_ctrlrange[:,1])
    floor=int(model.geom('floor').id)
    leg_geom={int(model.geom(f'tibia{k}_collision_0').id):k for k in (*STANCE,'fl','fr')}
    root_id=model.body('base_link_2').id
    foot_sites={leg:int(model.site(PREFIX+leg).id) for leg in STANCE}
    initial_com=com_xyz(model,data).copy()
    planned_nodes=plan['prep_waypoints']+plan['cycle_waypoints']
    total_prep=len(plan['prep_waypoints']);total_cycle=len(plan['cycle_waypoints'])
    contacts_trace=[];sampled=[];counts={k:{'lift_seen':False,'recontact_seen':False} for k in STANCE}
    min_loaded=6;min_margin=float('inf');max_angle=0.;max_tau=0.;max_raw_tau=0.;sat_count=0
    min_loaded_end_phases={};stopped=None;start_cycle_com=None;end_cycle_com=None
    history=[]
    def actual_step(goal,phase,section,node):
        nonlocal min_loaded,min_margin,max_angle,max_tau,max_raw_tau,sat_count
        nonlocal stopped
        u=7.0*(goal-data.qpos[qadr])-.32*data.qvel[dadr]
        max_raw_tau=max(max_raw_tau,float(np.max(np.abs(u))))
        sat_count+=int(np.any(np.abs(u)>caps+1e-8))
        data.ctrl[:]=np.clip(u,-caps,caps)
        mj.mj_step(model,data)
        fz={k:0. for k in (*STANCE,'fl','fr')};pts={k:[] for k in fz}
        for i in range(data.ncon):
            c=data.contact[i]
            gid=int(c.geom2) if int(c.geom1)==floor else int(c.geom1) if int(c.geom2)==floor else -1
            leg=leg_geom.get(gid)
            if leg is None:continue
            w=np.zeros(6);mj.mj_contactForce(model,data,i,w)
            world=c.frame.reshape(3,3).T@w[:3]
            n=float(world[2]);fz[leg]+=n
            pts[leg].append((np.array(c.pos).copy(),max(n,1e-8)))
        loaded=[k for k in STANCE if fz[k]>.08]
        min_loaded=min(min_loaded,len(loaded))
        max_tau=max(max_tau,float(np.max(np.abs(data.ctrl))))
        max_angle=max(max_angle,float(np.max(np.abs(data.qpos[ctx.joint_qpos_addrs]))))
        c=com_xyz(model,data)
        if len(loaded)>=3:
            p=np.array([np.average([v for v,wt in pts[k]],axis=0,weights=[wt for v,wt in pts[k]])[:2] for k in loaded])
            try:
                eq=ConvexHull(p).equations
                margin=float(np.min(-(eq[:,:2]@c[:2]+eq[:,2]))*1000)
            except Exception:margin=float('-inf')
        else: margin=float('-inf')
        min_margin=min(min_margin,margin)
        row={'t_s':float(data.time),'node':node,'phase':phase,'section':section,
             'loaded':loaded,'fz_N':fz,'margin_mm':margin,
             'COM_y_m':float(c[1]),'root_height_m':float(data.xpos[root_id,2]),
             'qmax_rad':float(np.max(np.abs(data.qpos[ctx.joint_qpos_addrs]))),
             'tau_max_Nm':float(np.max(np.abs(data.ctrl))),
             'sites_z_m':{k:float(data.site_xpos[v,2]) for k,v in foot_sites.items()}}
        history.append(row)
        if int(round(float(data.time)/.002))%25==0:
            sampled.append({k:val for k,val in row.items() if k!='sites_z_m'}|{'sites_z_m':row['sites_z_m']})
        if stopped is None:
            if len(loaded)<3: stopped=f'<3 loaded supports at node {node} ({phase})'
            elif margin<=0: stopped=f'nonpositive actual support margin at node {node} ({phase})'
            elif max_angle>1.700001: stopped=f'research joint limit crossed at node {node} ({phase})'
            elif sat_count>0: stopped=f'raw torque command saturated at node {node} ({phase})'
            elif not np.isfinite(data.qpos).all(): stopped='nonfinite qpos'
            elif float(data.xpos[root_id,2])<.08: stopped=f'body too low at node {node} ({phase})'
        return row
    goal=data.qpos[qadr].copy()
    for i in range(round(settle_seconds/.002)):
        actual_step(goal,'settle','pre',-1)
        if stopped:break
    reached_prep_nodes=0;reached_cycle_nodes=0
    if not stopped:
        for node_idx,node in enumerate(planned_nodes):
            section='prep' if node_idx<total_prep else 'cycle'
            if node_idx==total_prep:start_cycle_com=com_xyz(model,data).copy()
            desired=np.asarray(node['qpos'],dtype=float)[qadr]
            initial_goal=goal.copy();N=round(node_seconds/.002)
            for i in range(N):
                a=(i+1)/N;a=a*a*(3-2*a)
                row=actual_step(initial_goal+(desired-initial_goal)*a,node['phase'],section,node_idx)
                if stopped:break
            if stopped:break
            goal=desired
            if section=='prep':reached_prep_nodes+=1
            else:reached_cycle_nodes+=1
            # Gate at every presumed touchdown: during the last 0.1s of the
            # node four actual feet must carry at least 0.08 N. This is a
            # stronger viability condition than a single touchdown event.
            if node['phase'].endswith('touchdown_4'):
                tail=history[-max(1,round(min(node_seconds,.1)/.002)):]
                if not all(len(row['loaded'])==4 for row in tail):
                    stopped=f'post-touchdown lost 4 supports at node {node_idx} {node["phase"]}'
                elif not all(min(row['fz_N'][k] for k in STANCE)>=.08 for row in tail):
                    stopped=f'post-touchdown insufficient support force at node {node_idx}'
                if stopped:break
    if not stopped:
        for i in range(round(final_hold_seconds/.002)):
            actual_step(goal,'final_hold','cycle',len(planned_nodes))
            if stopped:break
    actual_end=com_xyz(model,data)
    if start_cycle_com is not None:end_cycle_com=(actual_end-start_cycle_com)*1000
    return {
        'schema':'cad30_fourstep_dynamic_independent_v1',
        'status':'DYNAMIC_CHECK_ABORTED' if stopped else 'DYNAMIC_FULL_PATH_COMPLETED',
        'stopped_reason':stopped,
        'true_gait_verified':False,
        'servo_real_world_calibrated':False,
        'physics_dt_s':.002,
        'node_duration_s':node_seconds,
        'model_friction_mu':mu,
        'prep_waypoints_planned':total_prep,'prep_waypoints_executed':reached_prep_nodes,
        'cycle_waypoints_planned':total_cycle,'cycle_waypoints_executed':reached_cycle_nodes,
        'min_loaded_stance_legs':min_loaded,
        'min_physical_support_margin_mm':min_margin if np.isfinite(min_margin) else None,
        'max_joint_abs_rad':max_angle,'peak_commanded_motor_Nm':max_tau,
        'peak_unclipped_motor_Nm':max_raw_tau,'motor_saturation_count':sat_count,
        'start_nominal_COM_xyz_m':initial_com.tolist(),
        'end_physical_COM_xyz_m':actual_end.tolist(),
        'actual_COM_movement_from_initial_mm':((actual_end-initial_com)*1000).tolist(),
        'actual_cycle_start_COM_xyz_m':start_cycle_com.tolist() if start_cycle_com is not None else None,
        'actual_cycle_COM_movement_mm':end_cycle_com.tolist() if end_cycle_com is not None else None,
        'physics_duration_s':float(data.time),
        'last_phase':history[-1]['phase'] if history else None,
        'last_loaded':history[-1]['loaded'] if history else [],
        'sampled_trace_50ms':sampled,
        'last_qpos':data.qpos.copy().tolist(),
        'last_qvel':data.qvel.copy().tolist(),
        'notes':'The kinematic root waypoints are NOT enforced. Only actuated joint goals are commanded with torque-clipped PD; consecutive steps abort after loss of contact.'
    }


def main():
    p=argparse.ArgumentParser();p.add_argument('--model',type=Path,required=True)
    p.add_argument('--state',type=Path,required=True);p.add_argument('--plan',type=Path,required=True)
    p.add_argument('--node-duration',type=float,default=.18)
    p.add_argument('--out',type=Path)
    args=p.parse_args();ctx=load_context(args.model,args.state)
    trial=rollout(ctx,json.loads(args.plan.read_text()),node_seconds=args.node_duration)
    if args.out:args.out.parent.mkdir(parents=True,exist_ok=True);args.out.write_text(json.dumps(trial,ensure_ascii=False,separators=(',',':'))+'\n')
    compact={k:v for k,v in trial.items() if k not in ('sampled_trace_50ms','last_qpos','last_qvel')}
    print(json.dumps(compact,ensure_ascii=False,indent=2))

if __name__=='__main__':main()