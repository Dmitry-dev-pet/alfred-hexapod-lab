#!/usr/bin/env python3
"""Export ORIGINAL Rider2 MuJoCo states for offline player. No inferred animation.

This requires the upstream LuwuDynamics/xgorider2_rl checkout pinned at
f534986353ce3fe451caf0da72ba06b980a59114, MuJoCo and the
manufacturer's native C++ controller. No real robot or flashing.
"""
import argparse
import copy
import hashlib
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")
p=argparse.ArgumentParser()
p.add_argument("--upstream-training", required=True, type=Path)
p.add_argument("--output-dir", required=True, type=Path)
p.add_argument("--gym-shim", type=Path, help="Only for Python 3.13 offline test host")
args=p.parse_args()
sys.path[:0]=([str(args.gym_shim.resolve())] if args.gym_shim else []) + [str(args.upstream_training.resolve())]

import mujoco
from rider_sim.env2 import RiderEnv2
from rider_sim.native_policy import NativePolicy
from rider_sim.linear_policy import LinearPolicy
from rider_sim.firmware import FIRMWARE

SHA="f534986353ce3fe451caf0da72ba06b980a59114"
SCENARIOS={
 "drive": dict(domain=1., commands=True, pushes=True),
 "corner": dict(domain=1., commands=True, pushes=True, corner_fraction=1.0),
}

def capture(algorithm, scenario, seed, seconds, output_dir, filename):
    policy=NativePolicy(FIRMWARE/'src/policy/trained_weights.h') if algorithm=='rl' else None
    linear=LinearPolicy() if algorithm=='linear' else None
    env=RiderEnv2(episode_seconds=seconds,native_policy=policy,noise=1.0,**copy.deepcopy(SCENARIOS[scenario]))
    try:
        obs,_=env.reset(seed=seed)
        m,d=env.model,env.data
        left,right=m.geom('tyre11').id,m.geom('tyre21').id
        ground=m.geom('ground').id
        joints=['extension12','wheel11','extension22','wheel21']
        indices={j:int(m.jnt_qposadr[m.joint(j).id]) for j in joints}
        frames=[]
        maxpitch=maxroll=0.
        def save(info=None):
            nonlocal maxpitch,maxroll
            fz=[0.,0.]
            count=[0,0]
            for ci in range(d.ncon):
                c=d.contact[ci]
                pair=(int(c.geom1),int(c.geom2))
                for j, wheel in enumerate((left,right)):
                    if wheel in pair and ground in pair:
                        wrench=np.zeros(6,dtype=np.float64)
                        mujoco.mj_contactForce(m,d,ci,wrench)
                        # Normal-force vertical magnitude (not planned support).
                        fz[j]+=abs(float(wrench[0])*float(c.frame[2]))
                        count[j]+=1
            roll,pitch,yaw=env._angles()
            cmd=np.asarray(env.last_command)
            velocity=float(info['velocity']) if info else float(env.axle_velocity())
            vref=float(info['reference']) if info else float(env.out[10])
            yref=float(info['yaw_reference']) if info else float(env.out[11])
            drift=float(info['position_error']) if info else 0.
            f=[round(float(d.time),6)]+[round(float(x),8) for x in d.qpos]+[round(float(x),6) for x in d.qvel]
            f += [round(float(x),4) for x in fz]+count
            f += [round(float(x),7) for x in (roll,pitch,yaw)]
            f += [round(float(x),6) for x in (velocity,vref,yref,drift)]
            f += [int(x) for x in cmd]+[int(env.elapsed<env.push_end)]
            assert len(f)==36
            frames.append(f)
            maxpitch=max(maxpitch,abs(pitch))
            maxroll=max(maxroll,abs(roll))
        save()
        target=.05
        done=truncated=False
        info=None
        ticks=0
        while not (done or truncated):
            action=policy.action.copy() if policy else linear.action(env.raw_observation)
            obs,_,done,truncated,info=env.step(action)
            ticks+=1
            if env.elapsed >= target-1e-9:
                save(info)
                target+=.05
        if frames[-1][0] < env.elapsed-1e-6:
            save(info)
        assert abs(frames[-1][0]-env.elapsed)<1e-4
        rec={
            'format':'xgorider2-mujoco-physical-replay-v1',
            'robot':'XGO-Rider2','source':'LuwuDynamics/xgorider2_rl',
            'upstream_commit':SHA,
            'model':'training/models/rider_measured.xml',
            'trained_policy':'rider_balance_v6' if algorithm=='rl' else None,
            'algorithm':'native_trained_rl' if algorithm=='rl' else 'upstream_linear_policy',
            'physics_engine':'MuJoCo','physics_version':mujoco.__version__,
            'python_version':sys.version.split()[0],
            'scenario':scenario,'seed':seed,'domain':SCENARIOS[scenario]['domain'],
            'duration_s':round(float(env.elapsed),6),'sample_interval_target_s':.05,
            'frame_count':len(frames),'sample_stride':36,'integration_step_s':float(m.opt.timestep),
            'sim_steps':int(env.ticks),'controller_ticks':ticks,
            'terminated':bool(done),'truncated':bool(truncated),
            'qpos_layout':['root_xyz(3)','root_quaternion_wxyz(4)','leg12','wheel11','leg22','wheel21'],
            'joint_qpos_indices':indices,
            'frame_layout':['t_s','qpos_11','qvel_10','contact_vertical_normal_load_N_2',
                'contact_count_2','roll_pitch_yaw_rad_3','actual_forward_velocity_m_s',
                'target_forward_velocity_m_s','target_yaw_velocity_rad_s','position_error_m',
                'wheel_command_units_2','push_active_bool'],
            'f_z_definition':'Actual mj_contactForce normal-force vertical magnitudes from tyre-ground collision pairs at recorded frames. Not inferred planned support.',
            'coordinate_system':'MuJoCo world Z-up (meters), root quaternion wxyz; wheel hinge axis local +Y.',
            'assumptions':['MuJoCo version may differ from upstream pinned release',
                'Mass/inertia properties contain upstream CAD-derived estimates',
                'Physical qpos/qvel sampled, not interpolated at export',
                'No real robot validation'],
            'max_abs_pitch_deg':round(math.degrees(maxpitch),4),
            'max_abs_roll_deg':round(math.degrees(maxroll),4),
            'total_net_x_m':round(float(d.qpos[0]-frames[0][1]),5),
            'frames':frames
        }
        payload=(json.dumps(rec,ensure_ascii=False,separators=(',',':'))+'\n').encode()
        dest=output_dir/filename
        dest.write_bytes(payload)
        return {k:rec[k] for k in ('algorithm','scenario','seed','duration_s','frame_count','sample_stride','terminated','truncated','max_abs_pitch_deg','max_abs_roll_deg','total_net_x_m')} | {'bytes':len(payload),'sha256':hashlib.sha256(payload).hexdigest()}
    finally:
        env.close()
        if policy:policy.close()

def main():
    output=args.output_dir.resolve()
    output.mkdir(parents=True,exist_ok=True)
    reports=[
        capture('rl','drive',51001,12.,output,'rider2-rl-drive-12s.json'),
        capture('linear','drive',51001,12.,output,'rider2-linear-drive-12s.json'),
        capture('rl','corner',51001,15.,output,'rider2-rl-corner-fail.json')]
    (output/'export_receipt.json').write_text(json.dumps(reports,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(reports,indent=2,ensure_ascii=False))

if __name__=='__main__':
    main()
