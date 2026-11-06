#!/usr/bin/env python3
"""Offline geometric walking envelope scan for pinned owner Freenove CAD30.

No gait physics or load/contacts/torques; original model and binary STL MUST stay
outside GitHub. Converts desired stance/swing toe offsets to 18 joint angles
using the exact pinned MuJoCo CAD30 forward kinematics and SciPy bounded IK.
The reported maximum is ONLY the largest successful grid point, not a proof
of the maximum geometrically possible stride or any stable walk.
"""
from __future__ import annotations
import argparse, csv, hashlib, json, math
from pathlib import Path
import numpy as np
import mujoco as mj
from hexlab.stand_geometry_v04 import load_legs, tip_world
from hexlab.step_one_leg_v04 import solve_waypoint

DIRECTIONS={
    'fwd':(1.,0.), 'back':(-1.,0.), 'left':(0.,1.), 'right':(0.,-1.),
    'fwd_left':(2**-.5,2**-.5), 'fwd_right':(2**-.5,-2**-.5),
    'back_left':(-2**-.5,2**-.5),'back_right':(-2**-.5,-2**-.5),
    'ccw':'yaw_ccw','cw':'yaw_cw',
}
SWING_U=(0,.25,.5,.75,1.)
STANCE_U=(.25,.5,.75)
ABS_Q=1.7   # pre-existing software IK envelope, not calibrated hardware end stop
MAX_FOOT_ERR_M=.0008


def foot_curve(u:float,swing:bool)->tuple[float,float]:
    """Horizontal toe offset scaled by stride; relative vertical clearance."""
    if not (0<=u<=1):raise ValueError('Phase must be within [0,1]')
    if swing:
        h=u*u*(3-2*u)-.5
        return h, math.sin(math.pi*u)
    return .5-u,0.


def direction_offsets(label,mm:float,anchors:dict[str,np.ndarray],radius:float)->dict:
    stride=mm/1000.
    direct=DIRECTIONS[label]
    if isinstance(direct,tuple):
        return {k:stride*np.asarray(direct) for k in anchors}
    sign=1. if label=='ccw' else -1.
    out={}
    for name,anchor in anchors.items():
        vector=sign*(stride/radius)*np.array([-anchor[1],anchor[0]])
        length=np.linalg.norm(vector)
        if length>stride:vector*=stride/length
        out[name]=vector
    return out


def sample_one(model, ref, leg, home_q, baseline_tip,
               root_qpos, vector_xy, forward_world,left_world,lift_m)->tuple[bool,float,float,str]:
    """Test swing AND stance trajectory; measured IK residual, soft joint range."""
    maxerr=0.;maxq=0.
    for swing,us in ((True,SWING_U),(False,STANCE_U)):
        for u in us:
            h,z=foot_curve(u,swing)
            target=baseline_tip+h*(vector_xy[0]*forward_world+vector_xy[1]*left_world)+np.array([0.,0.,lift_m*z])
            ref.qpos[:]=root_qpos
            ref.qvel[:]=0.
            mj.mj_forward(model,ref)
            q,err=solve_waypoint(model,ref,mj,leg,target,home_q)
            maxerr=max(maxerr,float(err));maxq=max(maxq,float(np.max(np.abs(q))))
            if not np.all(np.isfinite(q)) or not np.isfinite(err):return False,maxerr,maxq,'nan'
            if err>MAX_FOOT_ERR_M:return False,maxerr,maxq,'ik_residual'
            if maxq>ABS_Q+1e-9:return False,maxerr,maxq,'joint_soft_limit'
    return True,maxerr,maxq,'geometry_only'


def run(scene:Path,kit_path:Path,outdir:Path,strides:list[int],heights_mm:list[int],lifts_mm:list[int]):
    m=mj.MjModel.from_xml_path(str(scene))
    if (m.nq,m.nmesh,m.nu)!=(27,30,20):raise ValueError('Unexpected pinned CAD30 model')
    k=json.loads(kit_path.read_text())
    if k['format']!='fnk0052-cad30-kinematic-gaits-v4':raise ValueError('Unexpected kit')
    d=mj.MjData(m);d.qpos[:3]=k['root_position'];d.qpos[3:7]=k['root_quat_wxyz'];d.qpos[7:]=k['q_stand'];mj.mj_forward(m,d)
    legs=load_legs(m,d,mj)
    f=np.asarray(k['forward_world'],float);l=np.asarray(k['left_world'],float)
    origin=np.asarray(k['root_position'],float)
    baseline={z['suffix']:tip_world(m,d,z).copy() for z in legs}
    anchors={name:np.array([np.dot(pt-origin,f),np.dot(pt-origin,l)]) for name,pt in baseline.items()}
    radius=float(np.median([np.linalg.norm(z) for z in anchors.values()]))
    original=d.qpos.copy();ref=mj.MjData(m)
    raw=[]
    for height in heights_mm:
      root=original.copy();root[2]+=height/1000.
      for lift in lifts_mm:
       for direction in DIRECTIONS:
        for stride in strides:
         offsets=direction_offsets(direction,stride,anchors,radius)
         legs_result={}
         for leg in legs:
          label=leg['suffix']
          result=sample_one(m,ref,leg,original[leg['qadrs']],baseline[label],root,
                      offsets[label],f,l,lift/1000.)
          legs_result[label]={'ok':result[0], 'max_ik_error_mm':round(result[1]*1000,5),
             'max_abs_joint_rad':round(result[2],5),'reason':result[3]}
         raw.append({'body_height_offset_mm':height,'foot_lift_mm':lift,
          'direction':direction,'stride_mm':stride,'all_leg_ik_pass':all(r['ok'] for r in legs_result.values()),
          'legs':legs_result})
      print('done body height offset',height,'mm',flush=True)
    summary=[]
    for height in heights_mm:
      for lift in lifts_mm:
       for direction in DIRECTIONS:
        rows=[r for r in raw if r['body_height_offset_mm']==height and r['foot_lift_mm']==lift and r['direction']==direction]
        passed=[r['stride_mm'] for r in rows if r['all_leg_ik_pass']]
        summary.append({'body_height_offset_mm':height,'foot_lift_mm':lift,'direction':direction,
          'max_tested_pass_stride_mm':max(passed) if passed else None,
          'passes_mm':passed,'failed_mm':[r['stride_mm'] for r in rows if not r['all_leg_ik_pass']],
          'pass_monotone':not any(rows[i]['all_leg_ik_pass'] and any(not r['all_leg_ik_pass'] for r in rows[:i]) for i in range(len(rows)))})
    o={'schema':'cad30-offline-walking-ik-grid-v6','status':'PRELIMINARY_GEOMETRY_ONLY',
      'physical_pass':False,'model':'owner-provided scene_local30_nominal.xml',
      'model_sha256':hashlib.sha256(scene.read_bytes()).hexdigest(),
      'reference_kit_sha256':hashlib.sha256(kit_path.read_bytes()).hexdigest(),
      'candidate_toe':'farthest original tibia mesh vertex; physical toe contact not calibrated',
      'method':'8 sampled IK configurations for each leg: 5 swing points + 3 stance points, 10 directions including yaw',
      'kinematic_soft_bound_rad':ABS_Q,'max_ik_error_m':MAX_FOOT_ERR_M,
      'scan':{'strides_mm':strides,'body_height_offsets_mm':heights_mm,'swing_heights_mm':lifts_mm,
              'directions':list(DIRECTIONS)},
      'notes':['Joint max 1.7rad is a project software bound, NOT validated servo limits',
       'Max tested pass is grid evidence, NOT proof of maximum possible stride',
       'No self collision, link/floor penetration, support margin, contact force, friction, torque, velocity or dynamics checks',
       'Gait wave/ripple/tripod share single-leg geometric curve; their support timing and dynamic feasibility MUST be checked separately',
       'Mixed velocity/yaw, dynamic state transitions and stopping are NOT validated by this grid'],
       'summary':summary,'raw':raw}
    outdir.mkdir(parents=True,exist_ok=True)
    (outdir/'walking_ik_grid_v6.json').write_text(json.dumps(o,ensure_ascii=False,separators=(',',':'))+'\n')
    with (outdir/'walking_ik_summary_v6.csv').open('w',newline='') as fh:
     w=csv.DictWriter(fh,['body_height_offset_mm','foot_lift_mm','direction','max_tested_pass_stride_mm','passes_mm','failed_mm','pass_monotone']);w.writeheader()
     for item in summary:w.writerow({**item,'passes_mm':'/'.join(map(str,item['passes_mm'])),'failed_mm':'/'.join(map(str,item['failed_mm']))})
    return o


def main():
 p=argparse.ArgumentParser();p.add_argument('--scene',type=Path,required=True);p.add_argument('--kit',type=Path,required=True);p.add_argument('--outdir',type=Path,required=True)
 p.add_argument('--strides',default='10,20,30,40,50,60,70,80,90,100')
 p.add_argument('--heights',default='-15,0,15')
 p.add_argument('--lifts',default='16,40')
 p.add_argument('--directions',default=','.join(DIRECTIONS))
 a=p.parse_args()
 wanted=a.directions.split(',')
 if any(x not in DIRECTIONS for x in wanted):raise ValueError('Invalid direction')
 saved=dict(DIRECTIONS)
 try:
  DIRECTIONS.clear();DIRECTIONS.update({k:saved[k] for k in wanted})
  o=run(a.scene,a.kit,a.outdir,[int(x) for x in a.strides.split(',')],[int(x) for x in a.heights.split(',')],[int(x) for x in a.lifts.split(',')])
 finally:
  DIRECTIONS.clear();DIRECTIONS.update(saved)
 print(json.dumps({'status':o['status'],'n_cases':len(o['raw']),'n_summary':len(o['summary']),
 'max_ik_grid_only':max([s['max_tested_pass_stride_mm'] or 0 for s in o['summary']])},ensure_ascii=False))
if __name__=='__main__':main()
