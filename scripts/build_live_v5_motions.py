#!/usr/bin/env python3
"""Compile experimentally checked *kinematic* body/leg control limits for CAD30.

No real-time physics; no generated movements are physical PASS. Loads/torque,
foot-ground contacts, and combined motions are not solved by this exporter.
"""
from __future__ import annotations
import argparse, json, math, hashlib
from pathlib import Path
import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation
from hexlab.stand_geometry_v04 import load_legs, tip_world
from hexlab.step_one_leg_v04 import solve_waypoint

POSE_LIMITS={'forward_m':.012,'left_m':.012,'height_m':.012,'roll_rad':math.radians(5),'pitch_rad':math.radians(5),'yaw_rad':math.radians(7)}
LEG_LIMITS={'forward_m':.018,'left_m':.012,'lift_m':.013}
HEAD_LIMITS={'yaw_rad':math.radians(18),'pitch_rad':math.radians(16)}


def build(scene:Path, gait_kit:Path)->dict:
    k=json.loads(gait_kit.read_text(encoding='utf-8'))
    if k['format']!='fnk0052-cad30-kinematic-gaits-v4' or len(k['skeleton'])!=46:raise ValueError('Invalid base kit')
    model=mj.MjModel.from_xml_path(str(scene))
    if (model.nq,model.nu,model.nbody)!=(27,20,48):raise ValueError('Expected owner CAD30 + 20 actuated joints')
    data=mj.MjData(model)
    qstand=np.asarray(k['q_stand'],dtype=float)
    data.qpos[:3]=k['root_position'];data.qpos[3:7]=k['root_quat_wxyz'];data.qpos[7:]=qstand
    mj.mj_forward(model,data)
    rootp=np.array(k['root_position']);basequat=k['root_quat_wxyz']
    R0=Rotation.from_quat([basequat[1],basequat[2],basequat[3],basequat[0]]).as_matrix()
    f=np.array(k['forward_world'],float);l=np.array(k['left_world'],float);up=np.array([0.,0.,1.])
    assert abs(f@l)<1e-4 and abs(f@up)<1e-3 and abs(l@up)<1e-3
    legs=load_legs(model,data,mj)
    tips={z['suffix']:tip_world(model,data,z).copy() for z in legs}
    leg_idx={z['suffix']:[adr-7 for adr in z['qadrs']] for z in legs}
    errs=[];angles=[]
    def solve(leg,goal):
        ref=mj.MjData(model)
        ref.qpos[:]=data.qpos
        ref.qvel[:]=0
        mj.mj_forward(model,ref)
        ids=leg_idx[leg['suffix']]
        q,err=solve_waypoint(model,ref,mj,leg,goal,qstand[ids])
        if err>.0008:raise RuntimeError(f'IK error {leg["suffix"]}: {err*1e3:.3f}mm')
        if abs(q).max()>1.7+1e-8:raise RuntimeError(f'Out of joint bound {leg["suffix"]}')
        errs.append(float(err));angles.append(float(np.abs(q).max()))
        return [round(float(v),8) for v in q]
    poses={}
    for axis,amp in POSE_LIMITS.items():
        direction={}
        for side in ('neg','pos'):
            sign=-1 if side=='neg' else 1
            move=np.zeros(3);rotate=np.zeros(3)
            if axis=='forward_m':move=f*amp*sign
            if axis=='left_m':move=l*amp*sign
            if axis=='height_m':move=up*amp*sign
            if axis=='roll_rad':rotate=f*amp*sign
            if axis=='pitch_rad':rotate=l*amp*sign
            if axis=='yaw_rad':rotate=up*amp*sign
            newrot=Rotation.from_rotvec(rotate).as_matrix()@R0
            target_root=rootp+move
            q=qstand.copy()
            for leg in legs:
                # Keep this foot in its original *world* position while moving
                # the body, solve IK in the original reference-root frame.
                localgoal=newrot.T@(tips[leg['suffix']]-target_root)
                refgoal=rootp+R0@localgoal
                q[leg_idx[leg['suffix']]]=solve(leg,refgoal)
            direction[side]=[round(float(x),8) for x in q]
        poses[axis]=direction
    feet={}
    for leg in legs:
        block={}
        for axis,amp in LEG_LIMITS.items():
            block[axis]={}
            for side in ('neg','pos'):
                sign=1 if side=='pos' else -1
                direction={'forward_m':f,'left_m':l,'lift_m':up}[axis]
                goal=tips[leg['suffix']]+direction*sign*amp
                block[axis][side]=solve(leg,goal)
        feet[leg['suffix']]=block
    # The head has two genuine servo joints; home-to-command signs are CAD
    # model signs, not verified Freenove PWM degrees/signs.
    kit={
      'format':'fnk0052-cad30-offline-motions-v5','study':'issue62-body-leg-head-motions',
      'base_kit_sha256':hashlib.sha256(gait_kit.read_bytes()).hexdigest(),
      'source':{'cad_commit':k['upstream_cad_commit'],'model':'scene_local30_nominal.xml','kinematic_reference':'live-v4-gaits-kit.json'},
      'pose_limits':POSE_LIMITS,'leg_limits':LEG_LIMITS,'head_limits':HEAD_LIMITS,
      'basis':{'forward_world':k['forward_world'],'left_world':k['left_world'],'up_world':[0,0,1]},
      'leg_joint_indices':leg_idx,'pose_axes':poses,'leg_axes':feet,
      'head_joint_indices':{'yaw':18,'pitch':19},'head_joint_signs':{'yaw':-1,'pitch':1},
      'max_single_axis_ik_error_mm':round(max(errs)*1000,6),
      'max_single_axis_abs_joint_rad':round(max(angles),6),
      'limits_note':'Single-axis targets solved by owner CAD30 IK. Mixed axes only blended/interpolated; NOT certified physically feasible.',
      'physical_guarantee':False,
      'no_realtime_physics':True,
      'original_assets_included':False,
    }
    return kit

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--scene',type=Path,required=True)
    p.add_argument('--gaits',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();kit=build(a.scene,a.gaits)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(kit,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({'status':'KINEMATIC_ONLY','bytes':a.output.stat().st_size,'sha256':hashlib.sha256(a.output.read_bytes()).hexdigest(),'max_ik_error_mm':kit['max_single_axis_ik_error_mm'],'max_abs_joint_rad':kit['max_single_axis_abs_joint_rad']},ensure_ascii=False))
if __name__=='__main__':main()
