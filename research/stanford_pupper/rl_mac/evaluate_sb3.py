#!/usr/bin/env python3
"""Physically evaluate one SB3 checkpoint on a fixed, multi-axis Pupper route.

No joystick policy is fabricated. Use genuine mj_step rollouts, including falls.
Exported qpos are for later viewer conversion, not auto-published in the player.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from physics import PupperCPU,DT,DECIMATION,XML_BLOB

# Change every three seconds, stopping for the last second.
SCHEDULE=[
    (0.0, (0.0,0.0,0.0)),
    (1.0, (.20,0,0)),
    (4.0, (0,.15,0)),
    (7.0, (0,0,.35)),
    (10.0, (-.15,0,0)),
    (13.0, (.12,-.10,-.20)),
    (16.0, (.08,0,.15)),
    (19.0, (0,0,0)),
]


def commanded(t):
    chosen=SCHEDULE[0][1]
    for ts,cmd in SCHEDULE:
        if t>=ts:chosen=cmd
        else:break
    return chosen


def evaluate(agent,seconds=20):
    sim=PupperCPU(delay_steps=4)
    ob=sim.reset()
    rows=[]
    yaw_prev=None
    yaw_unwrapped=0.0
    fall_ticks=0
    sse_linear=[]
    sse_yaw=[]
    for step in range(round(seconds/(DT*DECIMATION))):
        t=step*DT*DECIMATION
        cmd=commanded(t)
        sim.set_command(cmd)
        obs=sim.observation()
        action,_=agent.predict(obs,deterministic=True)
        ob,m=sim.step(action)
        error_xy=float(np.linalg.norm(np.asarray(m['vel_xy'])-np.asarray(cmd[:2])))
        error_yaw=float(abs(m['yaw_rate']-cmd[2]))
        sse_linear.append(error_xy**2)
        sse_yaw.append(error_yaw**2)
        fall_ticks+=int(m['fell'])
        q=sim.data.qpos[3:7]
        yaw=float(np.arctan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]**2+q[3]**2)))
        if yaw_prev is not None:yaw_unwrapped+=float(np.arctan2(np.sin(yaw-yaw_prev),np.cos(yaw-yaw_prev)))
        yaw_prev=yaw
        if step%2==1:  # 25Hz, measured states, no root rewrite
            rows.append({
                't':round(float(sim.data.time),4),
                'qpos':sim.data.qpos.round(8).tolist(),
                'qvel':sim.data.qvel.round(8).tolist(),
                'foot_Fz':sim.contact_forces().round(4).tolist(),
                'command_vx_vy_wz':list(cmd),
                'tilt_deg':round(float(np.degrees(m['tilt_rad'])),4),
                'fell':bool(m['fell']),
            })
    summary={
        'duration_s':seconds,
        'physics_steps':int(round(seconds/DT)),
        'policy_steps':int(round(seconds/(DT*DECIMATION))),
        'recorded_frames':len(rows),
        'end_root_xyz_m':sim.data.qpos[:3].round(6).tolist(),
        'net_root_yaw_degrees':round(float(np.degrees(yaw_unwrapped)),3),
        'fall_guard_policy_ticks':fall_ticks,
        'rmse_linear_xy_mps':round(float(np.sqrt(np.mean(sse_linear))),5),
        'rmse_yaw_rps':round(float(np.sqrt(np.mean(sse_yaw))),5),
        'status':'FALL' if fall_ticks else 'NO_FALL_GUARD',
        'limits':'One deterministic flat-ground route; no hardware qualification or original MJLab parity',
    }
    return {'schema':'pupper-sb3-evaluation/v1',
            'model_original_xml_git_blob':XML_BLOB,
            'scenario':'20s scripted joystick route, body is free to move in MuJoCo',
            'summary':summary,'frames':rows}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--checkpoint',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    a=parser.parse_args()
    if not a.checkpoint.is_file():parser.error('Checkpoint missing')
    from stable_baselines3 import PPO
    model=PPO.load(str(a.checkpoint),device='cpu')
    result=evaluate(model)
    result['checkpoint_sha256']=hashlib.sha256(a.checkpoint.read_bytes()).hexdigest()
    a.out.parent.mkdir(parents=True,exist_ok=True)
    a.out.write_text(json.dumps(result,ensure_ascii=False,separators=(',',':'))+'\n')
    print('PUPPER_EVAL',json.dumps(result['summary'],ensure_ascii=False))

if __name__=='__main__':main()
