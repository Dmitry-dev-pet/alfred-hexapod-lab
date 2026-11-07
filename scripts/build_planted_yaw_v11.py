#!/usr/bin/env python3
"""Offline six-foot planted body yaw table for Freenove CAD30. No mj_step/physics.

Emits 31 independent IK solutions for -15..+15 degrees, anchored to OEM stand.
"""
import json, math, hashlib, argparse
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
import mujoco as mj
from hexlab.stand_geometry_v04 import load_legs, tip_world
from hexlab.step_one_leg_v04 import solve_waypoint
import hexlab.step_one_leg_v04 as kmod

ROOT=Path(__file__).resolve().parent
SCENE=ROOT/'source/mjcf/scene_local30_nominal.xml'
KIT=ROOT/'docs/hexapod/data/freenove-original-v7-kit.json'
REF=ROOT/'docs/hexapod/data/live-v4-gaits-kit.json'
OUT=ROOT/'docs/hexapod/freenove-grounded-v11/planted-body-yaw-kit.json'

def run(scene=SCENE,kit=KIT,refkit=REF,output=OUT):
 k=json.loads(kit.read_text());r=json.loads(refkit.read_text());m=mj.MjModel.from_xml_path(str(scene));d=mj.MjData(m)
 origin=np.array(k['cad_adapter']['neutral_root_xyz_m'],float)
 q0=np.array(k['q_home'],float)
 rot0=R.from_quat([*r['root_quat_wxyz'][1:],r['root_quat_wxyz'][0]])
 def setroot(deg):
  d.qpos[:3]=origin
  rq=(R.from_euler('z',deg,degrees=True)*rot0).as_quat()
  d.qpos[3:7]=[rq[3],*rq[:3]]
  d.qpos[7:]=q0
  d.qvel[:]=0
  mj.mj_forward(m,d)
 setroot(0)
 legs=load_legs(m,d,mj)
 anchors={l['suffix']:tip_world(m,d,l).copy() for l in legs}
 meta={l['suffix']:{'body':int(l['body']),'tip_local_xyz_m':[round(float(x),10) for x in l['tip_local']]} for l in legs}
 rows={}
 kmod.MAX_IK_ANGLE=2.4
 worst=0.;greatest=0.
 for deg in range(-15,16):
  setroot(deg)
  pose=q0.copy()
  for leg in legs:
   ids=np.asarray(leg['qadrs'])-7
   sol,err=solve_waypoint(m,d,mj,leg,anchors[leg['suffix']],q0[ids]);pose[ids]=sol
   worst=max(worst,err)
   greatest=max(greatest,np.max(np.abs(sol)))
   if err>.0003:raise ValueError(f'body yaw {deg} leg {leg["suffix"]} foot error {err*1000:.3f}mm')
  d.qpos[7:]=pose;mj.mj_forward(m,d)
  error=max(float(np.linalg.norm(tip_world(m,d,l)-anchors[l['suffix']])) for l in legs)
  assert error<.0003
  rows[str(deg)]=[round(float(x),8) for x in pose]
 result={
  'format':'fnk0052-cad30-planted-yaw-v11',
  'status':'OFFLINE_IK_GEOMETRY_ONLY_NOT_PHYSICS',
  'model_sha256':hashlib.sha256(scene.read_bytes()).hexdigest(),
  'factory_kit_sha256':hashlib.sha256(kit.read_bytes()).hexdigest(),
  'axis':'body_yaw_deg','values_degrees':list(range(-15,16)),
  'root_xyz_m':[round(float(x),10) for x in origin],
  'ground_anchors_xyz_m':{n:[round(float(x),8) for x in p] for n,p in anchors.items()},
  'cad_tip_local':meta,
  'rows':rows,
  'max_world_toe_err_mm':round(worst*1000,6),
  'max_abs_joint_rad':round(float(greatest),6),
  'limitations':['No dynamic contacts, torque, friction or self-collision verified','Only isolated stationary torso yaw footplant; arbitrary simultaneous walking and yaw require contact solver','q angles not hardware-servo-calibrated']
 }
 output.parent.mkdir(parents=True,exist_ok=True)
 output.write_text(json.dumps(result,separators=(',',':'),ensure_ascii=False)+'\n')
 print('ROWS',len(rows),'MAX TOE MM',worst*1000,'MAX Q RAD',greatest,'BYTES',output.stat().st_size)
 return result
if __name__=='__main__':
 ap=argparse.ArgumentParser()
 ap.add_argument('--scene',type=Path,required=True)
 ap.add_argument('--factory-kit',type=Path,default=KIT)
 ap.add_argument('--v4-kit',type=Path,default=REF)
 ap.add_argument('--output',type=Path,default=OUT)
 args=ap.parse_args()
 run(args.scene,args.factory_kit,args.v4_kit,args.output)
