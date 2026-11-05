#!/usr/bin/env python3
"""Independent free-base physics replay of a sampled *math-only* CAD30 crawl candidate.

This script does NOT set MuJoCo root qpos after initializing the measured
six-support state, nor does it mutate model bodies/actuators to assist walking.
The 218-node plan is joint position targets only; floor/tibia forces are
measured on every physical dt=0.002s. Strict FAIL on lost support, joint/motor
limits, missing touchdowns or non-physical root height. A PASS needs positive
travel from the original six-leg start and full repeated leg cycles.

The original Freenove CAD meshes are private on owner Google Drive; do not
commit those binaries with this script or any result archive.
"""
from __future__ import annotations
import argparse, json, gzip, math, sys, os
from pathlib import Path
from collections import defaultdict
import numpy as np
import mujoco as mj
from scipy.spatial import ConvexHull, QhullError

WALK=('ml','mr','bl','br')
ALL=(*WALK,'fl','fr')
FZ_LOADED=0.08
FZ_TOUCHDOWN=0.2
LIMIT_JOINT=1.7
LIMIT_TAU=0.7

def model_com(m,d):
    return np.sum(np.asarray(d.xipos)*np.asarray(m.body_mass)[:,None],axis=0)/float(np.sum(m.body_mass))

def read_contact(m,d,floor,leg_geoms):
    fz={k:0.0 for k in ALL}
    pos={k:[] for k in ALL}
    unexpected=[]
    for i in range(d.ncon):
        c=d.contact[i]
        floor_pair = (int(c.geom1)==floor or int(c.geom2)==floor)
        if not floor_pair:continue
        gid=int(c.geom2) if int(c.geom1)==floor else int(c.geom1)
        if gid not in leg_geoms:
            unexpected.append(gid)
            continue
        k=leg_geoms[gid];f=np.zeros(6)
        mj.mj_contactForce(m,d,i,f)
        world=c.frame.reshape(3,3).T@f[:3]
        fz[k]+=max(0.0,float(world[2]))
        pos[k].append(np.asarray(c.pos[:2],float).copy())
    xy={k:np.mean(v,axis=0) for k,v in pos.items() if v}
    return fz,xy,unexpected

def support_margin(xy,fz,com_xy):
    names=[k for k in ALL if fz[k]>FZ_LOADED and k in xy]
    if len(names)<3:return float('-inf')
    pts=np.asarray([xy[k] for k in names])
    try: eq=ConvexHull(pts).equations
    except QhullError:return float('-inf')
    return float(np.min(-(eq[:,:2]@com_xy+eq[:,2]))*1000)

def planned_supports(phase):
    if phase.startswith('six_support_reposition_'):
        return tuple(k for k in ALL if k!=phase.removeprefix('six_support_reposition_'))
    if phase.startswith('six_support_touchdown') or phase=='six_stance_observed':return ALL
    if phase.startswith('swing_'):
        return tuple(k for k in WALK if k!=phase.removeprefix('swing_'))
    return WALK

def run(cad_root,trace_file,six_file,output,mu=1.1,node_s=.15,hold_s=.25,
        max_nodes=218,finish_hold_s=.4,strict=True,support_policy="planned",br_down_mm=0.0):
    trace=json.loads(Path(trace_file).read_text())
    nodes=trace['nodes'][:max_nodes]
    src=json.loads(Path(six_file).read_text())
    model_path=Path(cad_root)/'source/mjcf/scene_local30_nominal.xml'
    m=mj.MjModel.from_xml_path(str(model_path))
    m.opt.timestep=.002
    floor=int(m.geom('floor').id)
    leg_geoms={int(m.geom(f'tibia{k}_collision_0').id):k for k in ALL}
    m.geom_friction[floor,0]=mu
    for idx in leg_geoms:m.geom_friction[idx,0]=mu
    d=mj.MjData(m)
    d.qpos[:]=src['final_qpos'];d.qvel[:]=src['final_qvel']
    actuator_joints=np.asarray(m.actuator_trnid[:,0],int)
    jadr=np.asarray(m.jnt_qposadr[actuator_joints],int)
    vadr=np.asarray(m.jnt_dofadr[actuator_joints],int)
    assert len(jadr)==20  # CAD30: 18 leg servos + 2 head servos
    initial_cmd=np.asarray(src['final_goals'],float).copy()
    assert len(initial_cmd)==20
    cmd=initial_cmd.copy()
    limits=np.asarray(m.actuator_ctrlrange)
    assert np.allclose(limits[:18,1],LIMIT_TAU) and np.allclose(limits[18:,1],0.2)
    d.ctrl[:]=np.clip(7*(cmd-d.qpos[jadr])-.32*d.qvel[vadr],limits[:,0],limits[:,1])
    mj.mj_forward(m,d)
    actual_start_root=np.asarray(d.qpos[:3]).copy()
    math_start_root=np.asarray(nodes[0]['qpos'][:3]).copy()
    tracked=[];raw_frames=[];progress=[]
    failure=None
    peak_tau=0.;peak_raw_tau=0.;max_joint=0.;saturation_ticks=0;low_support_ticks=0
    min_margin=float('inf');max_root_track_error=0.;min_fz=100.;unexpected_contacts=0
    min_loaded=6;min_intended_loaded=6
    fail_frame_index=None
    touchdown_stats={}
    ref_d=mj.MjData(m)
    # Position-control micro-adjustment of a *different* loaded foot. This is
    # only an open-loop diagnostic, not yet an Fz feedback controller.
    from hexlab.stand_geometry_v04 import load_legs,tip_world
    leg_infos={x['suffix']:x for x in load_legs(m,d,mj)}
    def adjusted_goal(q_math,base_goal,leg='br',dz=-0.001):
        ref_d.qpos[:]=q_math
        mj.mj_forward(m,ref_d)
        link=leg_infos[leg]
        point=tip_world(m,ref_d,link)
        J=np.zeros((3,m.nv));R=np.zeros((3,m.nv))
        mj.mj_jac(m,ref_d,J,R,point,link['body'])
        cols=link['dadrs']
        dq=np.linalg.lstsq(J[:,cols],np.array([0.,0.,dz]),rcond=None)[0]
        out=base_goal.copy()
        for q_idx,j_delta in zip(link['qadrs'],dq):
            index=int(np.flatnonzero(jadr==q_idx)[0]);out[index]+=j_delta
        return out

    def step(phase,plan_index,planned_root_y,planned_ground,goal,phase_tick,last_tick=False):
        nonlocal failure,peak_tau,peak_raw_tau,max_joint,saturation_ticks,low_support_ticks
        nonlocal min_margin,max_root_track_error,min_fz,unexpected_contacts,min_loaded,min_intended_loaded,fail_frame_index
        raw=7*(goal-d.qpos[jadr])-.32*d.qvel[vadr]
        applied=np.clip(raw,limits[:,0],limits[:,1])
        d.ctrl[:]=applied
        mj.mj_step(m,d)
        fz,xy,bad=read_contact(m,d,floor,leg_geoms)
        loaded=[k for k in ALL if fz[k]>FZ_LOADED]
        intended_loaded=[k for k in planned_ground if fz[k]>FZ_LOADED]
        com=model_com(m,d)
        mar=support_margin(xy,fz,com[:2])
        min_loaded=min(min_loaded,len(loaded)); min_intended_loaded=min(min_intended_loaded,len(intended_loaded))
        if mar!=float('-inf'):min_margin=min(min_margin,mar)
        if any(k in WALK for k in planned_ground):min_fz=min(min_fz,min(fz[k] for k in planned_ground))
        peak_tau=max(peak_tau,float(np.max(np.abs(applied))))
        peak_raw_tau=max(peak_raw_tau,float(np.max(np.abs(raw))))
        jmax=float(np.max(np.abs(d.qpos[jadr[:18]])));max_joint=max(max_joint,jmax)
        sat=bool(np.any(np.abs(raw)>limits[:,1]+1e-7))
        saturation_ticks+=int(sat)
        unexpected_contacts+=len(bad)
        root_y=float(d.qpos[1]);root_track_error=abs(root_y-planned_root_y)*1000
        max_root_track_error=max(max_root_track_error,root_track_error)
        if support_policy=='planned' and (phase.startswith('six_support') or phase=='six_stance_observed'): needed=5
        elif phase.startswith('front_pair_raise') or phase.startswith('four_stance_rebalance') or phase.startswith('before_') or phase=='closure' or phase.startswith('touchdown_'):needed=3
        else:needed=3
        if len(loaded)<needed:low_support_ticks+=1
        why=None
        if not np.all(np.isfinite(d.qpos)):why='nonfinite_state'
        elif float(d.qpos[2])<.08:why='body_root_too_low'
        elif len(loaded)<needed:why=f'loaded_support_loss_{len(loaded)}_needed_{needed}'
        elif jmax>LIMIT_JOINT+1e-5:why=f'joint_limit_exceeded_{jmax:.6f}'
        elif sat:why='motor_torque_saturation'
        elif not np.isfinite(mar):why='invalid_contact_margin'
        elif mar<-2 and phase not in ('six_stance_observed',):why=f'COM_outside_actual_support_polygon_{mar:.3f}_mm'
        frame={'t':round(float(d.time),5),'i':plan_index,'phase':phase,
            'root_y':root_y,'com_xyz':np.round(com,7).tolist(),'root_xyz':np.round(d.qpos[:3],7).tolist(),
            'fz_N':{k:round(float(v),5) for k,v in fz.items()},
            'loaded':loaded,'intended_loaded':intended_loaded,
            'margin_mm':round(mar,3) if np.isfinite(mar) else None,'qpos':np.round(d.qpos,7).tolist(),
            'qvel':np.round(d.qvel,6).tolist(),'peak_cmd_tau_Nm':round(float(np.max(abs(applied))),5),
            'motor_saturated':sat,'floor_unexpected_contacts':len(bad)}
        raw_frames.append(frame)
        if len(raw_frames)%25==0 or last_tick or why:
            tracked.append({k:v for k,v in frame.items() if k not in ('qpos','qvel')})
        if strict and why and failure is None:
            failure={'reason':why,'t_s':float(d.time),'node':plan_index,'phase':phase,
                    'planned_root_Y_m':planned_root_y,'actual_root_Y_m':root_y,
                    'COM_margin_mm':mar if np.isfinite(mar) else None,'loaded_leg_ids':loaded,
                    'Fz_N':fz, 'joint_max_rad':jmax,'raw_peak_Nm':float(np.max(abs(raw))) }
            fail_frame_index=len(raw_frames)-1
        return failure is None
    # Verify saved six-grounded state under the *saved*, not reset, motor targets.
    initial_fz,_,_=read_contact(m,d,floor,leg_geoms)
    for t in range(round(hold_s/m.opt.timestep)):
        if not step('six_stance_observed',0,float(nodes[0]['root_xy_m'][1]),ALL,cmd,t):break
    if failure is None:
        for i in range(1,len(nodes)):
            nd=nodes[i];p=nd['phase'];plan_goal=np.asarray(nd['qpos'],float)[jadr]
            if p=='swing_ml' and br_down_mm>0:
                plan_goal=adjusted_goal(np.asarray(nd['qpos'],float),plan_goal,leg='br',dz=-br_down_mm/1000)
            prev_cmd=cmd.copy()
            steps=max(1,round(node_s/m.opt.timestep))
            if p.startswith('six_support_touchdown') or p.startswith('touchdown_'):
                steps=max(steps,round(.3/m.opt.timestep))
            min_expected=0
            for j in range(steps):
                a=(j+1)/steps
                a=a*a*(3-2*a)
                goal=prev_cmd+(plan_goal-prev_cmd)*a
                if not step(p,i,float(nd['root_xy_m'][1]),planned_supports(p),goal,j,last_tick=j==steps-1):break
            cmd=plan_goal.copy()
            if failure:break
            fz,_,_=read_contact(m,d,floor,leg_geoms)
            if p.startswith('six_support_touchdown'):
                leg=p.removeprefix('six_support_touchdown_')
                if fz[leg]<FZ_TOUCHDOWN:failure={'reason':'prep_touchdown_not_loaded','node':i,'phase':p,'leg':leg,'fz_N':fz[leg]}
            elif p.startswith('touchdown_'):
                leg=p.removeprefix('touchdown_')
                touchdown_stats[leg]={'node':i,'Fz_N':fz[leg],'root_forward_from_prep_mm':(float(d.qpos[1])-actual_start_root[1])*1000}
                if fz[leg]<FZ_TOUCHDOWN or any(fz[k]<FZ_LOADED for k in WALK):
                    failure={'reason':'cycle_touchdown_not_four_loaded','node':i,'phase':p,'leg':leg,'fz_N':fz}
            if failure:break
    if failure is None:
        for j in range(round(finish_hold_s/m.opt.timestep)):
            if not step('final_hold',len(nodes)-1,float(nodes[-1]['root_xy_m'][1]),WALK,cmd,j):break
    final_fz,_,_=read_contact(m,d,floor,leg_geoms)
    final_com=model_com(m,d)
    original_root_y=np.asarray(json.loads((Path(six_file).parent/'raise_30mm_result.json').read_text())['initial_qpos'])[1] if (Path(six_file).parent/'raise_30mm_result.json').exists() else None
    results={'schema':'cad30-math-first-dynamic-replay-v1','status':'FAIL' if failure else ('PARTIAL_REPLAY_ONLY' if len(nodes)<218 else 'ONE_CYCLE_PHYSICAL_PRELIMINARY'),
        'physics_steps':len(raw_frames),'physics_dt_s':m.opt.timestep,'wall_time_not_measured':True,
        'nodes_attempted':int(raw_frames[-1]['i']+1),'nodes_total':218,
        'mu':mu,'seconds_per_waypoint':node_s,'support_policy':support_policy,'br_down_mm_during_ml_swing':br_down_mm,'torque_limit_nominal_Nm':LIMIT_TAU,
        'kinematic_angle_limit_research_rad':LIMIT_JOINT,'max_motor_command_Nm':peak_tau,
        'max_raw_PD_command_Nm':peak_raw_tau,'motor_saturation_ticks':saturation_ticks,
        'max_actual_joint_abs_rad':max_joint,
        'min_actual_contact_margin_mm':min_margin if np.isfinite(min_margin) else None,
        'min_loaded_leg_count':min_loaded,'min_intended_leg_count':min_intended_loaded,
        'unexpected_floor_collision_pairs':unexpected_contacts,
        'max_plan_root_Y_tracking_error_mm':max_root_track_error,
        'initial_measured_six_Fz_N':initial_fz,'final_measured_Fz_N':final_fz,
        'final_root_Y_m':float(d.qpos[1]),'final_COM_Y_m':float(final_com[1]),
        'root_Y_delta_from_saved_six_start_mm':float((d.qpos[1]-actual_start_root[1])*1000),
        'COM_Y_delta_from_saved_six_start_mm':float((final_com[1]-np.asarray(src['final_com_m'])[1])*1000),
        'actual_original_root_Y_m':float(original_root_y) if original_root_y is not None else None,
        'net_root_from_original_6leg_mm':float((d.qpos[1]-original_root_y)*1000) if original_root_y is not None else None,
        'failure':failure,'verified_touchdowns':touchdown_stats,
        'final_qpos':d.qpos.tolist(),'final_qvel':d.qvel.tolist(),
        'initial_state':'pupper_stance_prebalance_35mm.json saved physical qpos/qvel/final_goals',
        'commands':'PD joint target interpolation ONLY; no free-base root commands/teleports','limitations':[
            'Single nominal simulation: no robustness, no real motor calibration.',
            'Raw contact trace and contacts must be checked for true swing-liftoff and forward toe movement.',
            'It cannot be called full gait unless repeated cycles and positive net travel verified.'
        ]}
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    output.with_suffix('.json').write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n')
    output.with_name(output.stem+'_samples_40hz.json').write_text(json.dumps(tracked,ensure_ascii=False,indent=2)+'\n')
    with gzip.open(output.with_name(output.stem+'_trace_2ms.json.gz'),'wt') as f:json.dump({'schema':'cad30-dynamic-2ms-physical-v1','frames':raw_frames},f)
    return results

if __name__=='__main__':
    a=argparse.ArgumentParser()
    a.add_argument('--cad-root',required=True);a.add_argument('--math-trace',required=True);a.add_argument('--six-state',required=True)
    a.add_argument('--out',required=True);a.add_argument('--mu',type=float,default=1.1)
    a.add_argument('--node-s',type=float,default=.15);a.add_argument('--max-nodes',type=int,default=218)
    a.add_argument('--hold-s',type=float,default=.25);a.add_argument('--finish-hold-s',type=float,default=.4)
    a.add_argument('--support-policy',choices=['planned','three'],default='planned')
    a.add_argument('--br-down-mm',type=float,default=0.)
    opts=a.parse_args()
    r=run(opts.cad_root,opts.math_trace,opts.six_state,opts.out,mu=opts.mu,node_s=opts.node_s,
          hold_s=opts.hold_s,max_nodes=opts.max_nodes,finish_hold_s=opts.finish_hold_s, support_policy=opts.support_policy,br_down_mm=opts.br_down_mm)
    print(json.dumps({k:v for k,v in r.items() if k not in ('final_qpos','final_qvel','limitations')},ensure_ascii=False,indent=2))
