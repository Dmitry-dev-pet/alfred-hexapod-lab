#!/usr/bin/env python3
"""Offline Freenove FNK0052 manufacturer-command -> owner's CAD30 joint preview.

Source: Freenove .../Code/Server/control.py pinned commit b7d228cc870b0a802d3768bfcf742f85fa8695f6
- CMD_MOVE: clamp x/y to +-35mm, run_gait with Z=40mm, alternating tripod and 6-leg wave.
- CMD_POSITION: clamp x/y +-40mm and z +-20mm
- CMD_ATTITUDE: clamp roll/pitch/yaw +-15 degrees

Exact vendor increment formulas are implemented below for OEM gait sample cycles.
The mapping of original vendor foot XY to CAD30 legs, the reference root height,
MuJoCo IK, and interpolation are *adapters*, not the manufacturer's PWM calibration.
No mj_step, physics dynamics or servo commands; all calculations are offline.
"""
from __future__ import annotations
import argparse, hashlib,json,math,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import mujoco as mj
from hexlab.stand_geometry_v04 import load_legs,tip_world
from hexlab.step_one_leg_v04 import solve_waypoint
from hexlab import step_one_leg_v04 as ik_module
from hexlab.freenove_servo_map_v04 import CAD_TO_VENDOR_LEG

VENDOR_COMMIT='b7d228cc870b0a802d3768bfcf742f85fa8695f6'
VENDOR_POINTS=((137.1,189.4),(225.,0.),(137.1,-189.4),(-137.1,-189.4),(-225.,0.),(-137.1,189.4))
VENDOR_LEGS=('fr','mr','br','bl','ml','fl')
CMD_LIMIT_MOVE_MM=35
SWING_LIFT_MM=40
FACTORY_BODY_XY_MM=40
FACTORY_BODY_Z_MM=20
FACTORY_ATTITUDE_DEG=15
ADAPTER_ROOT_PLUS_MM=15  # chosen via CAD-only IK scan; not a manufacturer's measured root value
ADAPTER_Q_LIMIT=2.4  # CAD preview only; no verified real servo mapping
IK_ERR_MAX=.0008
OUTPUT_SAMPLES=48
DIRECTION_COMMANDS={
 'forward':(0,35,0), 'back':(0,-35,0), 'left':(35,0,0),'right':(-35,0,0),
 'forward_left':(35/2**.5,35/2**.5,0),
 'forward_right':(-35/2**.5,35/2**.5,0),
 'back_left':(35/2**.5,-35/2**.5,0),
 'back_right':(-35/2**.5,-35/2**.5,0),
 'ccw':(0,0,10),'cw':(0,0,-10),
 'lift':(0,0,0),
}

def vendor_gait(mode:str, x_mm:float,y_mm:float,angle_deg:float)->tuple[list,int]:
    """Vendor run_gait increment formulas, using F for OEM speed indices 8/6.

    mode 1: tripod, F=48 from original speed index=8.
    mode 2: wave, F=108 from original speed index=6.
    Source is original code; no motor PWM, no sleeping. The lift-only series
    is extracted from the gait branches even though the original zero command
    short-circuits before a gait cycle.
    """
    if mode not in ('1','2'):raise ValueError('Unknown Freenove gait')
    speed_idx=8 if mode=='1' else 6
    F=round((22-126)*(speed_idx-2)/8+126) if mode=='1' else round((45-171)*(speed_idx-2)/8+171)
    z=SWING_LIFT_MM/F
    angle=math.radians(angle_deg)
    x_step=[((p[0]*math.cos(angle)+p[1]*math.sin(angle)-p[0])+x_mm)/F for p in VENDOR_POINTS]
    y_step=[((-p[0]*math.sin(angle)+p[1]*math.cos(angle)-p[1])+y_mm)/F for p in VENDOR_POINTS]
    pts=[[float(x),float(y),-25.] for x,y in VENDOR_POINTS]
    frames=[]
    if mode=='1':
      for j in range(F):
       for i in range(3):
        even,odd=2*i,2*i+1
        if j<F/8:
            pts[even][0]-=4*x_step[even];pts[even][1]-=4*y_step[even]
            pts[odd][0]+=8*x_step[odd];pts[odd][1]+=8*y_step[odd];pts[odd][2]=SWING_LIFT_MM-25.
        elif j<F/4:
            pts[even][0]-=4*x_step[even];pts[even][1]-=4*y_step[even];pts[odd][2]-=z*8
        elif j<3*F/8:
            pts[even][2]+=z*8;pts[odd][0]-=4*x_step[odd];pts[odd][1]-=4*y_step[odd]
        elif j<5*F/8:
            pts[even][0]+=8*x_step[even];pts[even][1]+=8*y_step[even]
            pts[odd][0]-=4*x_step[odd];pts[odd][1]-=4*y_step[odd]
        elif j<3*F/4:
            pts[even][2]-=z*8;pts[odd][0]-=4*x_step[odd];pts[odd][1]-=4*y_step[odd]
        elif j<7*F/8:
            pts[even][0]-=4*x_step[even];pts[even][1]-=4*y_step[even];pts[odd][2]+=z*8
        else:
            pts[even][0]-=4*x_step[even];pts[even][1]-=4*y_step[even]
            pts[odd][0]+=8*x_step[odd];pts[odd][1]+=8*y_step[odd]
       frames.append([p.copy() for p in pts])
    else:
      number=(5,2,1,0,3,4)
      for i in range(6):
       for j in range(int(F/6)):
        for k in range(6):
         if number[i]==k:
          if j<int(F/18):pts[k][2]+=18*z
          elif j<int(F/9):pts[k][0]+=30*x_step[k];pts[k][1]+=30*y_step[k]
          else:pts[k][2]-=18*z
         else:pts[k][0]-=2*x_step[k];pts[k][1]-=2*y_step[k]
        frames.append([p.copy() for p in pts])
    if len(frames)!=F:raise AssertionError('Gait F mismatch')
    return frames,F

def q_to_list(q):return [round(float(x),8) for x in q]

def build(scene:Path,v4path:Path)->dict:
    v4=json.loads(v4path.read_text(encoding='utf-8'))
    m=mj.MjModel.from_xml_path(str(scene))
    if (m.nq,m.nmesh,m.nu)!=(27,30,20):raise RuntimeError('Pinned CAD30 expected')
    if v4.get('format')!='fnk0052-cad30-kinematic-gaits-v4':raise RuntimeError('Wrong V4 reference')
    d=mj.MjData(m)
    home_root=np.asarray(v4['root_position']).copy();home_root[2]+=ADAPTER_ROOT_PLUS_MM/1000
    d.qpos[:3]=home_root;d.qpos[3:7]=v4['root_quat_wxyz'];d.qpos[7:]=v4['q_stand'];mj.mj_forward(m,d)
    legs=load_legs(m,d,mj)
    ref=mj.MjData(m)
    old=ik_module.MAX_IK_ANGLE;ik_module.MAX_IK_ANGLE=ADAPTER_Q_LIMIT
    initial=np.asarray(v4['q_stand']).copy()
    q_home=initial.copy()
    errors=[];max_joint=0.;miss=[]
    vendor_index={label:i for i,label in enumerate(VENDOR_LEGS)}
    def solve_one(leg,xyz,seed):
      nonlocal max_joint
      q,err=solve_waypoint(m,ref,mj,leg,xyz,seed)
      errors.append(float(err));max_joint=max(max_joint,float(np.max(np.abs(q))))
      if err>IK_ERR_MAX:miss.append((leg['suffix'],round(float(err*1000),4)))
      return q
    def ref_pose(translation=None,rotation=None):
      ref.qpos[:3]=home_root+(np.asarray(translation) if translation is not None else 0)
      mat=Rotation.from_quat([*v4['root_quat_wxyz'][1:],v4['root_quat_wxyz'][0]])
      if rotation is not None:mat=Rotation.from_rotvec(np.asarray(rotation))*mat
      q=mat.as_quat();ref.qpos[3:7]=[q[3],q[0],q[1],q[2]]
      ref.qpos[7:]=v4['q_stand'];ref.qvel[:]=0
      mj.mj_forward(m,ref)
    def world_foot(vpoint):return np.array([-vpoint[0],vpoint[1],vpoint[2]+25],dtype=float)/1000
    try:
      ref_pose()
      for leg in legs:
        ix=vendor_index[CAD_TO_VENDOR_LEG[leg['suffix']]]
        q=solve_one(leg,world_foot([*VENDOR_POINTS[ix],-25]),initial[np.array(leg['qadrs'])-7])
        q_home[np.array(leg['qadrs'])-7]=q
      if miss:raise RuntimeError('Home unreachable: '+str(miss))
      home_toes={leg['suffix']:world_foot([*VENDOR_POINTS[vendor_index[CAD_TO_VENDOR_LEG[leg['suffix']]]],-25]) for leg in legs}
      gait_tables={};max_gait_err=0
      max_gait_q=0
      for gait in ('1','2'):
        key='tripod' if gait=='1' else 'wave'
        table={};counts={}
        for direction,(cmd_x,cmd_y,yaw) in DIRECTION_COMMANDS.items():
          frames,native_F=vendor_gait(gait,cmd_x,cmd_y,yaw)
          rows=[];joint_prev=q_home.copy()
          for sample in range(OUTPUT_SAMPLES):
            # native-time interpolation without altering manufacturer stroke amplitudes
            index=(sample/OUTPUT_SAMPLES)*native_F
            lower=min(native_F-1,int(index));higher=min(lower+1,native_F-1);ratio=index-lower
            native_points=[np.asarray(frames[lower][j])*(1-ratio)+np.asarray(frames[higher][j])*ratio for j in range(6)]
            q20=q_home.copy();ref_pose()
            for leg in legs:
              ix=vendor_index[CAD_TO_VENDOR_LEG[leg['suffix']]]
              target=world_foot(native_points[ix]);ids=np.array(leg['qadrs'])-7
              solved=solve_one(leg,target,q_home[ids])
              # Stable IK selection: V4 stance reference avoids wrong knee branch
              q20[ids]=solved
            if any(x[1]>.8 for x in miss):raise RuntimeError('Bad samples '+str(miss[-2:]))
            rows.append(q_to_list(q20))
          table[direction]=rows
          counts[direction]=native_F
          print('Gait',key,direction,'F',native_F,'maxerr',round(max(errors[-OUTPUT_SAMPLES*6:])*1000,3),flush=True)
        gait_tables[key]={'rows':table,'native_F':counts['forward'],'native_speed_index':8 if gait=='1' else 6,
           'cycle_s':round(counts['forward']*.01,5),'factory_full_stroke_mm':70 if gait=='1' else round(35*5/3,6)}
      # Body translation axes in CAD world (original vendor x positive -> model left negative X)
      base_modes={
       'left_m':(np.array([-FACTORY_BODY_XY_MM/1000,0,0]),np.zeros(3)),
       'forward_m':(np.array([0,FACTORY_BODY_XY_MM/1000,0]),np.zeros(3)),
       'height_m':(np.array([0,0,FACTORY_BODY_Z_MM/1000]),np.zeros(3)),
       'roll_rad':(np.zeros(3),np.array([0,math.radians(FACTORY_ATTITUDE_DEG),0])),
       'pitch_rad':(np.zeros(3),np.array([-math.radians(FACTORY_ATTITUDE_DEG),0,0])),
       'yaw_rad':(np.zeros(3),np.array([0,0,math.radians(FACTORY_ATTITUDE_DEG)]))
      }
      body_axes={}
      for name,(shift,rot) in base_modes.items():
        body_axes[name]={}
        for label,sign in [('pos',1),('neg',-1)]:
          ref_pose(sign*shift,sign*rot)
          row=q_home.copy()
          for leg in legs:
            ix=vendor_index[CAD_TO_VENDOR_LEG[leg['suffix']]]
            ids=np.asarray(leg['qadrs'])-7
            row[ids]=solve_one(leg,home_toes[leg['suffix']],q_home[ids])
          body_axes[name][label]=q_to_list(row)
          print('Body',name,label,'maxerr',round(max(errors[-6:])*1000,4),flush=True)
      if miss:raise RuntimeError('IK mismatch count '+str(len(miss))+' first '+str(miss[:7]))
      return {
        'format':'fnk0052-factory-cad30-kinematic-v7', 'study':'original-amplitudes-preview',
        'source':{'vendor_repo':'Freenove/Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_Pi',
          'vendor_control_py_commit':VENDOR_COMMIT,'model_sha256':hashlib.sha256(scene.read_bytes()).hexdigest(),
          'v4_ref_sha256':hashlib.sha256(v4path.read_bytes()).hexdigest(),
          'vendor_legs':list(VENDOR_LEGS), 'cad_to_vendor_leg':CAD_TO_VENDOR_LEG},
        'factory_commands':{'move_xy_clamp_mm':CMD_LIMIT_MOVE_MM,'lift_mm':SWING_LIFT_MM,
          'position_xy_clamp_mm':FACTORY_BODY_XY_MM,'position_z_clamp_mm':FACTORY_BODY_Z_MM,
          'attitude_clamp_deg':FACTORY_ATTITUDE_DEG,
          'vendor_angle_deg_profile':10},
        'cad_adapter':{'neutral_root_plus_mm':ADAPTER_ROOT_PLUS_MM,
         'neutral_root_xyz_m':q_to_list(home_root),
         'vendor_foot_xy_to_cad_world':[-1,1],
         'joint_ik_bound_rad':ADAPTER_Q_LIMIT,
         'unverified_real_motor_bounds':True},
        'phase_samples':OUTPUT_SAMPLES,'joint_count':20,'q_home':q_to_list(q_home),
        'neutral_vendor_points_mm':[[*p,-25] for p in VENDOR_POINTS],
        'gaits':gait_tables,'body_axes':body_axes,
        'max_ik_error_mm':round(max(errors)*1000,5), 'max_abs_joint_rad':round(max_joint,6),
        'limitations':['Factory formula/command scales, but not factory PWM motor mapping',
          'CAD root reference height +15mm selected to fit trajectories, not a measured factory height',
          '2.4rad is CAD IK preview bound only; physical servos uncalibrated',
          'One-axis tables do not prove combined directions/body poses or stable contact',
          'No MuJoCo at runtime; rendered XY/yaw dead-reckoning not measured travel',
          'No physics Fz torque collisions or body stability tested',
          'Only manufacturer 2 gait types; Ripple is separate experimental mode',
          'No manufacturer head/leg arbitrary control claimed']}
    finally:
      ik_module.MAX_IK_ANGLE=old

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--scene',type=Path,required=True);ap.add_argument('--v4-kit',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args();res=build(a.scene,a.v4_kit)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(res,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n')
    print(json.dumps({'status':'CAD_KINEMATIC_PREVIEW','kit':str(a.output),'bytes':a.output.stat().st_size,'sha256':hashlib.sha256(a.output.read_bytes()).hexdigest(),'max_ik_error_mm':res['max_ik_error_mm'],'max_abs_q_rad':res['max_abs_joint_rad']}))
if __name__=='__main__':main()
