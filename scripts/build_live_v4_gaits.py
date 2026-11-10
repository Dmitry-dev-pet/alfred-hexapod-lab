#!/usr/bin/env python3
"""Compile 3 stride lengths x 3 gaits x 6 body-velocity axes, offline only.

Runs independent MuJoCo forward kinematics and SciPy IK on the pinned CAD30
model, with no runtime dynamics or physical success claims. Does not copy CAD.
"""
from __future__ import annotations
import argparse, hashlib, json, math
from pathlib import Path
import numpy as np
import mujoco as mj
from hexlab.stand_geometry_v04 import load_legs,tip_world
from hexlab.step_one_leg_v04 import solve_waypoint

LEG_ORDER=('ml','bl','br','mr','fr','fl')
GAITS={'wave':1/6,'ripple':1/3,'tripod':1/2}
STRIDES=(15,25,35)  # mm full horizontal foot excursion in one gait cycle
MODES=('fwd','back','left','right','ccw','cw')
SAMPLES=48
PERIOD=2.4
FOOT_LIFT=.016

def phase_for(gait,i,phase):
    if gait not in GAITS:raise ValueError('Invalid gait')
    return round((phase+((i%2)*.5 if gait=='tripod' else i/6))%1,12)%1

def smooth(u):
    u=max(0.,min(1.,u))
    return u*u*(3-2*u)

def foot_curve(gait, i, phase):
    """Nominal support-count phase schedules, not a force/contact estimator."""
    duty=GAITS[gait]
    p=phase_for(gait,i,phase)
    if p<duty-1e-10:
        u=p/duty
        return smooth(u)-.5, math.sin(math.pi*u), True
    return .5-(p-duty)/(1-duty),0.,False

def body_step(mode, length, anchor, nominal_radius):
    if mode=='fwd':return np.array([length,0.])
    if mode=='back':return np.array([-length,0.])
    if mode=='left':return np.array([0.,length])
    if mode=='right':return np.array([0.,-length])
    if mode=='ccw':angle= length/nominal_radius
    elif mode=='cw':angle=-length/nominal_radius
    else:raise ValueError('Invalid mode')
    x,y=anchor
    direction=np.array([-angle*y,angle*x])
    # A foot at an outer radius must not exceed the declared stride cap.
    mag=np.linalg.norm(direction)
    if mag>length:direction*=length/mag
    return direction

def build(source_dir:Path,base_kit:Path,output:Path):
    original=base_kit.read_bytes();k=json.loads(original)
    assert k['format']=='fnk0052-cad30-kinematic-control-v1' and k['samples']==48
    model_path=source_dir/'mjcf'/'scene_local30_nominal.xml'
    model=mj.MjModel.from_xml_path(str(model_path))
    assert (model.nq,model.nbody,model.nu)==(27,48,20)
    data=mj.MjData(model)
    data.qpos[:7]=[*k['root_position'],*k['root_quat_wxyz']]
    data.qpos[7:]=k['q_stand']; data.qvel[:]=0
    mj.mj_forward(model,data)
    root_id=model.body('base_link_2').id
    root=np.asarray(data.xpos[root_id]).copy()
    R0=np.asarray(data.xmat[root_id]).reshape(3,3).copy()
    forward=np.asarray(k['forward_world'],float);forward/=np.linalg.norm(forward)
    left=np.asarray(k['left_world'],float);left/=np.linalg.norm(left)
    legs=load_legs(model,data,mj)
    assert [l['suffix'] for l in legs]==list(LEG_ORDER)
    anchors={l['suffix']:np.array([(tip_world(model,data,l)-root)@forward,(tip_world(model,data,l)-root)@left]) for l in legs}
    radii=[np.linalg.norm(a) for a in anchors.values()]
    radius=float(np.median(radii))
    qref=data.qpos.copy()
    tables={}; maxerr={}; maxangle={}; max_drift={}
    ref=mj.MjData(model)
    total=len(GAITS)*len(STRIDES)*len(MODES)
    run=0
    for gait in GAITS:
      tables[gait]={};maxerr[gait]={};maxangle[gait]={};max_drift[gait]={}
      for stride in STRIDES:
        key=str(stride);length=stride/1000
        tables[gait][key]={};maxerr[gait][key]={};maxangle[gait][key]={};max_drift[gait][key]={}
        for mode in MODES:
          cycle=[];error=0.;angle_max=0.; max_abs_tip_delta=0.
          offsets={n:body_step(mode,length,anchors[n],radius) for n in LEG_ORDER}
          for j in range(SAMPLES):
            q=qref.copy();p=j/SAMPLES
            for i,leg in enumerate(legs):
              n=leg['suffix']; horiz,lift,_=foot_curve(gait,i,p)
              desired=tip_world(model,data,leg)+horiz*(offsets[n][0]*forward+offsets[n][1]*left)+np.array([0.,0.,lift*FOOT_LIFT])
              ref.qpos[:]=qref;ref.qvel[:]=0
              mj.mj_forward(model,ref)
              q_leg,err=solve_waypoint(model,ref,mj,leg,desired,qref[leg['qadrs']])
              if not math.isfinite(err) or err>.003:
                raise RuntimeError(f'IK {gait}/{stride}/{mode} sample{j} {n}: {err*1000:.3f}mm')
              q[leg['qadrs']]=q_leg
              error=max(error,err)
              angle_max=max(angle_max,float(np.max(np.abs(q_leg))))
              max_abs_tip_delta=max(max_abs_tip_delta,float(np.linalg.norm(offsets[n])))
            if not np.isfinite(q).all() or angle_max>1.70:
              raise RuntimeError(f'IK limit {gait}/{stride}/{mode} sample{j}: {angle_max:.5f}rad')
            cycle.append([round(float(v),8) for v in q[7:]])
          tables[gait][key][mode]=cycle
          maxerr[gait][key][mode]=round(error*1000,5)
          maxangle[gait][key][mode]=round(angle_max,5)
          max_drift[gait][key][mode]=round(max_abs_tip_delta*1000,5)
          run+=1
          print(f'[{run}/{total}] {gait}/{stride}mm/{mode} IKerr={error*1000:.4f}mm |q|={angle_max:.4f}',flush=True)
    kit={
      'format':'fnk0052-cad30-kinematic-gaits-v4',
      'study':'issue62-v4-stride-gaits-offline',
      'notice':'KINEMATIC PREVIEW ONLY: body travel is commanded, no measured Fz/contacts/torques, no real-time physics',
      'source_base_kit_sha256':hashlib.sha256(original).hexdigest(),
      'upstream_cad_commit':k['upstream_cad_commit'],
      'stride_mm_levels':list(STRIDES),'gaits':list(GAITS),
      'nominal_swing_duty':GAITS,'period_s':PERIOD,'foot_lift_mm':int(FOOT_LIFT*1000),'samples':SAMPLES,
      'nominal_turn_radius_m':round(radius,8),
      'q_stand':k['q_stand'],'root_position':k['root_position'],'root_quat_wxyz':k['root_quat_wxyz'],
      'forward_world':k['forward_world'],'left_world':k['left_world'],
      'leg_order':k['leg_order'],'skeleton':k['skeleton'],
      'joint_tables':tables,'max_ik_error_mm':maxerr,'max_abs_joint_rad':maxangle,'max_foot_horizontal_excursion_mm':max_drift,
      'limitations':['NO realtime dynamics, contact/Fz, or physical gait PASS in the viewer',
         'Mixed vx/vy/wz blends and midcycle gait switches are not physically proven',
         'Uncalibrated servo PWM, joint ranges and torque-speed curves',
         'Nominal support duty does not prove real foot contact',
         'STL assets remain private/referenced from existing verified offline player']}
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(kit,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n',encoding='utf8')
    print('SAVED',output,'bytes',output.stat().st_size,'sha256',hashlib.sha256(output.read_bytes()).hexdigest())
    return kit

if __name__=='__main__':
    a=argparse.ArgumentParser()
    a.add_argument('--source-dir',type=Path,required=True)
    a.add_argument('--base-kit',type=Path,required=True)
    a.add_argument('--output',type=Path,required=True)
    args=a.parse_args();build(args.source_dir,args.base_kit,args.output)
