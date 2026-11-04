"""CAD30 controlled two-/three-/four-support unload diagnostic.

All variants start from the same *recorded* free-base MuJoCo state and do not
kinematically move the chassis. Uses the same model PD, **not** a balance MPC.
The two-leg lift is a NEGATIVE CONTROL, never a gait proof.

Source physical model and safe pose are separately restored by the owner from
Google Drive. Does not distribute third-party CAD meshes or use GitHub Actions.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
from collections import Counter

import numpy as np
import mujoco as mj
from scipy.spatial import ConvexHull
from hexlab.stand_geometry_v04 import load_legs, tip_world
from hexlab.step_one_leg_v04 import solve_waypoint

STANCE=('ml','mr','bl','br')
FRONT=('fl','fr')
LIFT_MM=5.0
FORCE_N=0.08
KP=7.0
KD=.32


def com_world(m,d):
    return np.sum(np.asarray(d.xipos)*np.asarray(m.body_mass)[:,None],axis=0)/float(np.sum(m.body_mass))


def contact_forces(m,d,floor,geom_leg):
    fz={k:0.0 for k in (*STANCE,*FRONT)}
    pts={k:[] for k in fz}
    for ix in range(d.ncon):
        c=d.contact[ix]
        other=int(c.geom2) if int(c.geom1)==floor else int(c.geom1) if int(c.geom2)==floor else -1
        if other not in geom_leg:
            continue
        k=geom_leg[other]
        wrench=np.zeros(6)
        mj.mj_contactForce(m,d,ix,wrench)
        force=np.asarray(c.frame).reshape(3,3).T @ wrench[:3]
        f=max(0.,float(force[2]))
        fz[k]+=f
        pts[k].append((np.asarray(c.pos).copy(),f))
    return fz,pts


def polygon_margin(com,pts,loaded):
    if len(loaded)<3:
        return None  # no static support polygon for 2 supports
    centers=[]
    for k in loaded:
        p=np.array([x for x,w in pts[k]])
        w=np.array([w for x,w in pts[k]])
        if len(p)==0:
            raise ValueError(f'Loaded leg {k} has no floor contacts')
        centers.append(np.average(p,axis=0,weights=w+1e-9)[:2])
    eq=ConvexHull(centers).equations
    return float(np.min(-(eq[:,:2] @ com[:2] + eq[:,2]))*1e3)


def prepare(model_file:Path,state_file:Path):
    m=mj.MjModel.from_xml_path(str(model_file))
    if m.nmesh!=30 or m.nu!=20 or m.nq!=27:
        raise ValueError(f'Unexpected CAD30 parameters {m.nmesh} meshes / {m.nu} actuators / {m.nq} qpos')
    state=json.loads(state_file.read_text())
    d=mj.MjData(m)
    d.qpos[:]=state['qpos'];d.qvel[:]=state['qvel'];mj.mj_forward(m,d)
    legs={l['suffix']:l for l in load_legs(m,d,mj)}
    a=np.asarray(m.actuator_trnid[:,0],dtype=int)
    qa=np.asarray(m.jnt_qposadr[a],dtype=int)
    da=np.asarray(m.jnt_dofadr[a],dtype=int)
    caps=np.asarray(m.actuator_ctrlrange[:,1],float)
    if np.any(caps[:18]<.69):
        raise ValueError('Expected model research torque caps >=.69 Nm')
    base_target=d.qpos[qa].copy()
    targets={}
    initial_tips={leg:tip_world(m,d,l).copy() for leg,l in legs.items()}
    for suffix in STANCE:
        ref=mj.MjData(m);ref.qpos[:]=d.qpos;ref.qvel[:]=d.qvel;mj.mj_forward(m,ref)
        leg=legs[suffix]
        q,err=solve_waypoint(m,ref,mj,leg,initial_tips[suffix]+[0.,0.,LIFT_MM/1000.],d.qpos[leg['qadrs']])
        if err>.0008 or max(abs(q))>1.700001:
            raise RuntimeError(f'IK candidate not valid for {suffix}: error={err}, max joint={max(abs(q))}')
        motor_goal=base_target.copy()
        for idx,jaddr in enumerate(leg['qadrs']):
            matches=np.flatnonzero(qa==jaddr)
            if len(matches)!=1:raise RuntimeError('Motor mapping ambiguous')
            motor_goal[int(matches[0])]=q[idx]
        targets[suffix]={'motor_goal':motor_goal,'ik_error_mm':float(err*1e3),'max_abs_ik_rad':float(max(abs(q)))}
    return m,d,legs,qa,da,caps,base_target,targets


def smooth(x):
    x=float(np.clip(x,0,1));return x*x*(3-2*x)


def motion(t):
    # settle, unload, hold, reload, post-hold
    # Return (interpolation alpha, phase)
    if t<.3:return 0.,'settle'
    if t<1.0:return smooth((t-.3)/.7),'unload'
    if t<1.4:return 1.,'hold_lift'
    if t<2.1:return 1.-smooth((t-1.4)/.7),'reload'
    return 0.,'hold_landed'


def simulate(model_file:Path,state_file:Path,lifted:tuple[str,...],tmax=2.4,capture_qpos=False):
    m,d,legs,qa,da,caps,goal,targets=prepare(model_file,state_file)
    m.opt.timestep=.002
    for suffix in (*STANCE,*FRONT):
        m.geom_friction[m.geom(f'tibia{suffix}_collision_0').id,0]=1.1
    m.geom_friction[m.geom('floor').id,0]=1.1
    floor=int(m.geom('floor').id)
    geomleg={int(m.geom(f'tibia{k}_collision_0').id):k for k in (*STANCE,*FRONT)}
    model_goal=goal.copy()
    for k in lifted:model_goal+=targets[k]['motor_goal']-goal
    base=int(m.body('base_link_2').id)
    R0=np.asarray(d.xmat[base]).reshape(3,3).copy()
    up_body=R0.T@np.array([0.,0.,1.])
    z0=float(d.qpos[2]);com0=com_world(m,d).copy()
    dt=float(m.opt.timestep);N=round(tmax/dt)
    counts=Counter();trace=[]
    min_height=1e9;max_tilt=0.;max_tau=0.;max_joint=0.;saturated=0
    lift_support_counts=Counter();lift_min=99;two_support_duration=0.;three_support_duration=0.;lost_to_under_two_steps=0
    phase_min={}
    final_fz={};hold_exact_matches=0;hold_steps=0;hold_best_streak=0;hold_streak=0
    hold_wrongly_unloaded=Counter();hold_commanded_unloaded=Counter();sample_foot_clearance_mm=[]
    front_unintended_contact_steps=0
    recordings=[] if capture_qpos else None
    if capture_qpos:
        initial_fz,_=contact_forces(m,d,floor,geomleg)
        recordings.append({'t_s':0.0,'qpos':[float(x) for x in d.qpos],
                           'contact_mask':[k for k in STANCE if initial_fz[k]>FORCE_N],
                           'fz_N':{k:float(initial_fz[k]) for k in STANCE},'phase':'initial'})
    for i in range(N):
        t=i*dt;alpha,phase=motion(t)
        command=goal+alpha*(model_goal-goal)
        tau=KP*(command-d.qpos[qa])-KD*d.qvel[da]
        saturated+=int(np.any(abs(tau)>caps+1e-8))
        d.ctrl[:]=np.clip(tau,-caps,caps)
        mj.mj_step(m,d)
        fz,pts=contact_forces(m,d,floor,geomleg)
        loaded=tuple(k for k in STANCE if fz[k]>FORCE_N)
        counts[len(loaded)]+=1
        phase_min[phase]=min(phase_min.get(phase,99),len(loaded))
        if phase=='hold_lift':
            lift_support_counts[len(loaded)]+=1
            lift_min=min(lift_min,len(loaded))
            hold_steps+=1
            actual_unloaded=set(STANCE)-set(loaded)
            for foot in lifted:
                if foot in actual_unloaded:hold_commanded_unloaded[foot]+=1
            for foot in actual_unloaded-set(lifted):hold_wrongly_unloaded[foot]+=1
            correct=(actual_unloaded==set(lifted))
            hold_exact_matches+=int(correct)
            hold_streak = hold_streak+1 if correct else 0
            hold_best_streak=max(hold_best_streak,hold_streak)
        if fz['fl']>FORCE_N or fz['fr']>FORCE_N:
            front_unintended_contact_steps+=1
        two_support_duration+=dt*(len(loaded)==2)
        three_support_duration+=dt*(len(loaded)==3)
        lost_to_under_two_steps+=len(loaded)<2
        root_rot=np.asarray(d.xmat[base]).reshape(3,3)
        up=root_rot@up_body
        tilt=float(math.degrees(math.acos(np.clip(float(up@np.array([0.,0.,1.])), -1,1))))
        max_tilt=max(max_tilt,tilt)
        min_height=min(min_height,float(d.qpos[2]))
        max_tau=max(max_tau,float(np.max(abs(d.ctrl))))
        max_joint=max(max_joint,float(max(abs(d.qpos[int(x)]) for l in legs.values() for x in l['qadrs'])))
        if i%25==0 or i==N-1:
            com=com_world(m,d)
            margin=polygon_margin(com,pts,loaded)
            trace.append({'t_s':round(float(d.time),3),'phase':phase,'loaded_stance':list(loaded),'n_loaded':len(loaded),
                          'fz_N':{k:round(fz[k],4) for k in STANCE},'body_z_m':round(float(d.qpos[2]),6),
                          'COM_x_m':round(float(com[0]),6),'COM_y_m':round(float(com[1]),6),
                          'tilt_deg':round(tilt,4),'static_COM_margin_mm':round(margin,4) if margin is not None else None,
                          'max_abs_tau_Nm':round(float(np.max(abs(d.ctrl))),4)})
            if phase=='hold_lift' and lifted:
                sample_foot_clearance_mm.append({k:round(float(tip_world(m,d,legs[k])[2]-
                                       (0.0 if k not in ('ml','mr') else .00035))*1000,3) for k in lifted})
        final_fz=fz
        if capture_qpos and ((i+1)%25==0 or i==N-1):
            recordings.append({'t_s':round(float(d.time),6),
                               'qpos':[float(x) for x in d.qpos],
                               'contact_mask':list(loaded),
                               'fz_N':{k:float(fz[k]) for k in STANCE},
                               'phase':phase})
    com_end=com_world(m,d)
    result={'schema':'cad30_three_vs_two_support_short_diagnostic_v1','requested_lifted_legs':list(lifted),
           'model':str(model_file),'initial_safe_snapshot':str(state_file),
           'initial_model_com_m':com0.tolist(),'final_model_com_m':com_end.tolist(),
           'physical_dt_s':dt,'duration_s':N*dt,'PD':{'Kp':KP,'Kd':KD,'nominal_torque_cap_Nm':.7},
           'tibia_floor_static_friction_mu':1.1,'Fz_loaded_threshold_N':FORCE_N,
           'IK_rear_leg':{x:{k:v for k,v in targets[x].items() if k!='motor_goal'} for x in lifted},
           'contact_count_full_2ms':{str(k):v for k,v in sorted(counts.items())},
           'contact_count_lift_hold_2ms':{str(k):v for k,v in sorted(lift_support_counts.items())},
           'min_loaded_during_lift_hold':lift_min if lifted else None,
           'hold_physics_steps':hold_steps,
           'hold_actual_contacts_match_requested_frames':hold_exact_matches,
           'hold_longest_consecutive_correct_contact_s':round(hold_best_streak*dt,6),
           'hold_commanded_leg_unloaded_physics_steps':dict(hold_commanded_unloaded),
           'hold_unintended_leg_unloaded_physics_steps':dict(hold_wrongly_unloaded),
           'front_pair_unexpected_ground_contact_physics_steps':front_unintended_contact_steps,
           'hold_sampled_requested_tip_clearance_mm':sample_foot_clearance_mm,
           'phase_min_loaded':phase_min,
           'time_exactly_two_stance_contacts_s':round(two_support_duration,6),
           'time_exactly_three_stance_contacts_s':round(three_support_duration,6),
           'steps_below_two_contacts':lost_to_under_two_steps,
           'max_body_tilt_deg':max_tilt,'body_height_min_m':min_height,
           'initial_body_height_m':z0,'final_body_height_m':float(d.qpos[2]),
           'max_commanded_motor_torque_Nm':max_tau,'max_abs_all_leg_joint_rad':max_joint,
           'saturated_physics_steps':saturated,
           'COM_forward_displacement_mm':float((com_end[1]-com0[1])*1000),
           'end_leg_fz_N':{k:float(final_fz[k]) for k in STANCE},
           'transient_two_support_seen':counts[2]>0,
           'evidence_of_four_step_gait':False,
           'controller_class':'PD hold + bounded two-leg toe raise; no balancing MPC, no planned foothold translation',
           'limitations':['Starting qpos comes from a prior 6->4 transition whose historical joints exceeded research bound',
                          'This only checks short unloading/landing, not walking or viability of repeated two-support cycles',
                          'Nominal mass, friction, servo torque and joint angle limits are uncalibrated hardware assumptions',
                          'Foot contact forces measured from MuJoCo; front pair must remain lifted',
                          'No dynamic balance optimizer: failure cannot disprove feasibility with MPC'],
           'trace_50ms':trace}
    if capture_qpos:
        result['recorded_qpos_20hz']=recordings
    return result


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--model',type=Path,default=Path('source/mjcf/scene_local30_nominal.xml'))
    p.add_argument('--state',type=Path,default=Path('results/cad30_safe_pose_B_nominal.json'))
    p.add_argument('--outdir',type=Path,default=Path('results/dynamic_2_3_4_probe'))
    args=p.parse_args();args.outdir.mkdir(parents=True,exist_ok=True)
    cases={
        'four': (),
        'three_rear_bl': ('bl',),
        'three_middle_ml': ('ml',),
        'two_middles_ml_mr': ('ml','mr'),
        'two_rears_bl_br': ('bl','br'),
        'two_left_ml_bl': ('ml','bl'),
        'two_right_mr_br': ('mr','br'),
        'two_diag_ml_br': ('ml','br'),
        'two_diag_mr_bl': ('mr','bl'),
    }
    outcomes={}
    for name,lifted in cases.items():
        print('running',name,lifted,flush=True)
        r=simulate(args.model,args.state,lifted)
        file=args.outdir / f'{name}.json'
        file.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
        outcomes[name]={k:v for k,v in r.items() if k not in ('trace_50ms','model','initial_safe_snapshot')}
        print(name,'min loaded',r['phase_min_loaded'],'contact counts',r['contact_count_full_2ms'],'max tilt',r['max_body_tilt_deg'],'z',r['body_height_min_m'],'torques',r['max_commanded_motor_torque_Nm'],flush=True)
    (args.outdir/'summary.json').write_text(json.dumps(outcomes,ensure_ascii=False,indent=2)+'\n')

if __name__=='__main__':main()
