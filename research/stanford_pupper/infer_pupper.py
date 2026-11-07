#!/usr/bin/env python3
"""Independent Stanford Pupper deploy-JSON CPU inference + MuJoCo physical rollout.

Pinned original policy is 960 -> 256 -> 128 -> 128 -> 128 -> 12 with ELU,
20 frames * 48 dimensions, latest-frame-first. This uses exactly the original
joint names, action scales and gait reference stored in deployed policy JSON.
Physics model: source-verified MJCF with 13 noncolliding visual STL omitted,
flat floor, 12 position servo gains from upstream constants. The optional
4 physics-step action delay matches the Stanford deterministic-delay variant.
This is an independent CPU reconstruction, not full mjlab parity.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from collections import deque
from pathlib import Path
import sys
import time

import numpy as np
import mujoco

WORK=Path(__file__).resolve().parent
sys.path.insert(0,str(WORK))
from reproduce_pupper import load_model,read_pinned_xml,DEFAULT_POSE,PUPPER_JOINTS,FOOT_GEOMS,FOOT_SITES,TORQUE_LIMIT,contact_snapshot
from history_contract import advance_history

POLICY_SHA256='ecf3a4ab8ea392dccda64977faa29d813dadc0d6099ae3abff5f84cc57156712'


def load_policy(path: Path):
    data=path.read_bytes()
    if hashlib.sha256(data).hexdigest()!=POLICY_SHA256:
        raise ValueError('Policy sha256 mismatch')
    p=json.loads(data)
    assert p['in_shape']==[None,960]
    assert p['observation_history']==20 and p['single_observation_size']==48
    assert p['obs_spec']=={'frame_order':'newest_first','history':20,'components':[
        {'name':'base_ang_vel','size':3},{'name':'projected_gravity','size':3},
        {'name':'command','size':3},{'name':'desired_world_z','size':3},
        {'name':'joint_pos_rel','size':12},{'name':'last_action','size':12},
        {'name':'gait_reference','size':12}]}
    assert p['gait_reference']['joint_names']==list(PUPPER_JOINTS)
    weights=[]
    dim=960
    for l in p['layers']:
        w=np.array(l['weights'][0],dtype=np.float32)
        b=np.array(l['weights'][1],dtype=np.float32)
        assert w.shape==(dim,b.size) and l['shape']==[None,b.size]
        weights.append((w,b,l['activation']))
        dim=b.size
    assert dim==12
    default=np.asarray(p['default_joint_pos'],np.float64)
    assert np.allclose(default,DEFAULT_POSE,atol=2e-8)
    assert float(p['kp'])==5.5 and float(p['kd'])==0.25
    return p,weights


def net(weights,inp):
    x=np.asarray(inp,dtype=np.float32)
    assert x.shape==(960,) and np.all(np.isfinite(x))
    for w,b,activation in weights:
        x=x@w+b
        if activation=='elu':
            x=np.where(x>=0,x,np.expm1(np.minimum(x,0.0))).astype(np.float32)
        else:
            assert not activation,activation
    assert x.shape==(12,) and np.isfinite(x).all()
    return x


def gait_ref(p,t,vx,vy,wz):
    g=p['gait_reference']
    fast=abs(vx)>=g['gallop_speed']
    f=g['frequency']*(g['gallop_freq_mult'] if fast else 1.)
    z=t*f
    phase=z-math.trunc(z)
    moving=abs(vx)>=g['dir_threshold']
    direction=vx if moving else wz
    eff=phase if direction>=0 else -phase
    pos=eff*g['n_samples']
    ip=math.floor(pos)
    i0=ip%g['n_samples'];i1=(i0+1)%g['n_samples']
    a=pos-ip
    table=(g['gallop_table'] if direction>=0 else g.get('gallop_back_table',g['gallop_table'])) if fast else (g['trot_table'] if moving else g.get('lift_table',g['trot_table']))
    row=(1-a)*np.asarray(table[i0],dtype=np.float64)+a*np.asarray(table[i1],dtype=np.float64)
    blend=np.clip(math.sqrt(vx*vx+vy*vy+wz*wz)/g['blend_speed'],0.,1.)
    return (blend*(row-np.asarray(p['default_joint_pos'],dtype=np.float64))).astype(np.float32)


def obs(data,model,prev_action,cmd,phase_t,p):
    body=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'base_link')
    R=data.xmat[body].reshape(3,3)
    gravity=R.T@np.array([0.,0.,-1.])
    vel=np.zeros(6)
    mujoco.mj_objectVelocity(model,data,mujoco.mjtObj.mjOBJ_BODY,body,vel,1)
    ang=vel[:3]
    jt=data.qpos[7:19]-np.asarray(p['default_joint_pos'])
    r=gait_ref(p,phase_t,*cmd)
    frame=np.concatenate((ang,gravity,cmd,np.array([0.,0.,1.]),jt,prev_action,r)).astype(np.float32)
    assert frame.shape==(48,)
    return frame


def launch(policy,weights,cmd_vx=0.20,duration=8.,delay=4,warmup=1.,fade=1.,seed=7,frameskip=5):
    model=load_model(read_pinned_xml(WORK/'upstream/pupper_v3.xml'))
    data=mujoco.MjData(model)
    mujoco.mj_resetData(model,data)
    data.qpos[:3]=[0,0,.18]
    data.qpos[3:7]=[1,0,0,0]
    data.qpos[7:19]=policy['default_joint_pos']
    data.ctrl[:]=policy['default_joint_pos']
    mujoco.mj_forward(model,data)
    steps=round(duration/.004)
    history=np.zeros((20,48),np.float32)
    history[:,5]=-1.
    last_action=np.zeros(12,np.float32)
    baseline=np.asarray(policy['default_joint_pos'],dtype=np.float64)
    low=np.asarray(policy['joint_lower_limits'])
    high=np.asarray(policy['joint_upper_limits'])
    action_scale=float(policy['action_scale'])
    current_command=baseline.copy()
    # actuator delay is in *physics* steps and applied after position target output
    servo_queue=deque([baseline.copy() for _ in range(delay+1)],maxlen=delay+1)
    records=[]
    policy_events=[]
    peak_torque=0.
    fall_guard_steps=0
    saturations=0
    contact_samples=0
    bodyid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'base_link')
    for k in range(steps):
        t=float(data.time)
        if k % 5==0:
            if t<warmup:
                current_command=baseline.copy(); action=np.zeros(12,np.float32)
                cmd=(0.,0.,0.)
            else:
                cmd=(cmd_vx,0.,0.) if t<duration-1.0 else (0.,0.,0.)
                frame=obs(data,model,last_action,cmd,t-warmup,policy)
                history[0]=frame
                action=net(weights,history.flatten())
                multiplier=min(1.,(t-warmup)/fade) if fade>0 else 1.
                scaled=multiplier*action*action_scale
                current_command=np.clip(baseline+scaled,low,high)
                last_action=(multiplier*action).astype(np.float32)
                advance_history(history,last_action)
                # Keep gait_reference in this frame: Stanford stores the next
                # last_action in the *next* frame at offsets 24:36 after rotate.
                policy_events.append({'t':round(t,5),'command':list(cmd),'mean_action':float(np.mean(action)),
                                       'max_abs_action':float(np.max(np.abs(action))),
                                       'mean_target_delta':float(np.mean(current_command-baseline))})
            assert np.isfinite(current_command).all()
        servo_queue.append(current_command.copy())
        data.ctrl[:]=servo_queue[0]
        mujoco.mj_step(model,data)
        peak_torque=max(peak_torque,float(np.max(np.abs(data.actuator_force))))
        saturations+=int(np.any(np.abs(data.actuator_force)>=(TORQUE_LIMIT-1e-5)))
        tilt=math.acos(np.clip(data.xmat[bodyid].reshape(3,3)[2,2],-1,1))
        fall_guard_steps+=int(tilt>math.radians(45.) or float(data.qpos[2])<0.05)
        if (k+1)%frameskip==0 or k+1==steps:
            c=contact_snapshot(model,data)
            contact_samples+=int(any(n>.2 for n in c.values()))
            records.append({'t':round(float(data.time),6),'qpos':data.qpos.round(8).tolist(),
                'qvel':data.qvel.round(8).tolist(),'ctrl':data.ctrl.round(6).tolist(),
                'foot_Fz':{z:round(v,4) for z,v in c.items()},
                'body_xpos':data.xpos[bodyid].round(8).tolist(),
                'roll_pitch_guard_deg':round(math.degrees(tilt),4),
                'max_servo_torque':round(float(np.max(np.abs(data.actuator_force))),4)})
    summary={'physics_steps':steps,'frames':len(records),'duration_s':duration,
        'vx_command_mps':cmd_vx,'initial_x':records[0]['qpos'][0],
        'final_x':records[-1]['qpos'][0],
        'forward_dx_m':records[-1]['qpos'][0]-records[0]['qpos'][0],
        'final_z_m':records[-1]['qpos'][2],
        'fall_guard_steps':fall_guard_steps,'peak_servo_torque_Nm':peak_torque,
        'torque_saturation_rate':saturations/steps,'loaded_foot_samples':contact_samples,
        'events':len(policy_events),'max_abs_raw_action':max(e['max_abs_action'] for e in policy_events)}
    return summary,records,policy_events


def main():
    a=argparse.ArgumentParser()
    a.add_argument('--duration',type=float,default=8.)
    a.add_argument('--vx',type=float,default=.2)
    a.add_argument('--delay',type=int,default=4)
    a.add_argument('--output',type=Path,default=Path('pupper_pretrained_rollout.json'))
    a.add_argument('--policy',type=Path,required=True)
    args=a.parse_args()
    p,w=load_policy(args.policy)
    t=time.perf_counter();s,r,ev=launch(p,w,cmd_vx=args.vx,duration=args.duration,delay=args.delay)
    result={'policy_sha256':POLICY_SHA256,'policy_type':'pretrained original Stanford Pupper default; no RL training',
            'physics':'reconstructed position servo MuJoCo CPU, original inertials and contacts',
            'parity':'not yet matched against upstream C++ outputs or native MJLab',
            'actuator_feedback_delay_steps':args.delay,'warmup_s':1.0,'fade_s':1.0,'summary':s,'frames':r,'policy_events':ev}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,separators=(',',':'))) 
    print('SUMMARY',json.dumps(s,ensure_ascii=False))
    print('RUNTIME',time.perf_counter()-t)

if __name__=='__main__': main()