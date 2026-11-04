"""Minimal interface: Stanford Pupper's unchanged swing targets -> Mink IK -> CAD30 PD.

Research on nominal Freenove geometry; one rear-foot swing from a four-contact stance.
Pupper source and Stanford MIT LICENSE are vendored separately. No custom gait.
"""
from __future__ import annotations
import argparse,json,sys,os
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import mujoco as mj
import mink
ROOT=Path(__file__).resolve().parent
VENDOR=ROOT/'vendor'/'stanford_pupper'
if not VENDOR.is_dir():
 VENDOR=ROOT.parent/'vendor'/'stanford_pupper'
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(VENDOR))
from SwingLegController import SwingController
from StanceController import StanceController
from Gaits import GaitController
from six_contact_raise import contacts, total_com
from front_pair_unload import run as four_start
CAD30_ROOT=Path(os.environ.get('CAD30_ROOT',str(Path.cwd()))).resolve()
sys.path.insert(0,str(CAD30_ROOT))
from hexlab.stand_geometry_v04 import load_legs, tip_world
from scipy.spatial import ConvexHull
LEGS=('ml','mr','bl','br','fl','fr')
STANCES=('ml','mr','bl','br')
SWING_ORDER=('bl','mr','br','ml')

class Config:
 def __init__(self,world_feet, foot_forward=.04,swing_s=2.0,clearance=.012):
  self.dt=.05
  self.swing_ticks=round(swing_s/self.dt)
  self.stance_ticks=20
  self.alpha=0.5
  self.beta=0.
  self.z_clearance=clearance
  self.z_time_constant=1.
  self.default_stance=np.stack([world_feet[k] for k in STANCES],axis=1)
  self.target_forward=foot_forward


def record(model,d,original_tip):
 pts=contacts(model,d)
 rootid=int(model.body('base_link_2').id)
 root_xyz=np.asarray(d.xpos[rootid]).copy()
 return {'loaded':[k for k,v in pts.items() if v>.08],
         'fz_N':{k:round(float(v),5) for k,v in pts.items()},
         'root_xyz_m':np.round(root_xyz,7).tolist(),
         'com_xyz_m':np.round(total_com(model,d),7).tolist()}


def create_model_with_foot_site(path, qpos,leg):
 base=mj.MjModel.from_xml_path(str(path));d=mj.MjData(base)
 d.qpos[:]=qpos;mj.mj_forward(base,d);legs=load_legs(base,d,mj)
 body=int(base.body('tibia'+leg).id)
 toe=tip_world(base,d,next(l for l in legs if l['suffix']==leg))
 mat=np.asarray(d.xmat[body]).reshape(3,3);local=mat.T@(toe-d.xpos[body])
 spec=mj.MjSpec.from_file(str(path));spec.body('tibia'+leg).add_site(name='pupper_toe_'+leg,pos=local,type=mj.mjtGeom.mjGEOM_SPHERE,size=(.0005,),rgba=(.1,.6,.9,0.))
 m=spec.compile();return m,local


def current_geometry(model,data):
 from scipy.spatial import ConvexHull
 toe_by_leg={}
 leg_items=load_legs(model,data,mj)
 for k in STANCES:
  l=next(x for x in leg_items if x['suffix']==k)
  toe_by_leg[k]=tip_world(model,data,l).copy()
 com=total_com(model,data)
 out={}
 for lifted in STANCES:
  pts=np.array([toe_by_leg[k][:2] for k in STANCES if k!=lifted]);eq=ConvexHull(pts).equations
  out[lifted]=float(np.min(-(eq[:,:2]@com[:2]+eq[:,2]))*1000)
 return {'com_xyz_m':com.tolist(),'toe_world_xyz_m':{k:v.tolist() for k,v in toe_by_leg.items()},'static_three_toe_margins_mm':out}

def measured_support_margin(model,data,fz):
 """Signed COM edge distance for actual loaded tibia contacts, in mm."""
 from scipy.spatial import ConvexHull
 if sum(fz[k]>.08 for k in STANCES)<3:return float('-inf')
 floor=int(model.geom('floor').id)
 reverse={int(model.geom(f'tibia{k}_collision_0').id):k for k in STANCES}
 points={k:[] for k in STANCES}
 for ci in range(data.ncon):
  c=data.contact[ci];gid=int(c.geom2) if int(c.geom1)==floor else int(c.geom1) if int(c.geom2)==floor else -1
  if gid in reverse:points[reverse[gid]].append(np.asarray(c.pos[:2],float).copy())
 pts=np.array([np.mean(points[k],axis=0) for k in STANCES if fz[k]>.08 and points[k]])
 if len(pts)<3:return float('-inf')
 eq=ConvexHull(pts).equations;com=total_com(model,data)[:2]
 return float(np.min(-(eq[:,:2]@com+eq[:,2]))*1000.)

def run_trial(shift_forward=.04,mu=1.1):
 raised=four_start(shift_mm=35.,lift_mm=80.,rear_push_mm=0.,mu=mu)
 if raised['status']!='FOUR_CONTACT_STAND_FROM_SIX':raise RuntimeError('four starting contacts did not validate')
 path=CAD30_ROOT/'source/mjcf/scene_local30_nominal.xml'
 m,_=create_model_with_foot_site(path,np.array(raised['final_qpos']),'bl')
 m.opt.timestep=.002
 m.geom_friction[int(m.geom('floor').id),0]=mu
 for k in LEGS:m.geom_friction[int(m.geom(f'tibia{k}_collision_0').id),0]=mu
 d=mj.MjData(m);d.qpos[:]=raised['final_qpos'];d.qvel[:]=raised['final_qvel'];mj.mj_forward(m,d)
 legs=load_legs(m,d,mj);world={leg['suffix']:tip_world(m,d,leg).copy() for leg in legs}
 cfg=Config(world,shift_forward,2.,.012)
 source=SwingController(cfg);stance=StanceController(cfg)
 legidx=STANCES.index('bl')
 command=SimpleNamespace(horizontal_velocity=np.array([0.,shift_forward/(cfg.alpha*cfg.stance_ticks*cfg.dt)]),yaw_rate=0.,height=world['bl'][2])
 state=SimpleNamespace(foot_locations=cfg.default_stance.copy(),height=world['bl'][2]);
 pose=mink.Configuration(m);pose.update(d.qpos)
 task=mink.FrameTask('pupper_toe_bl','site',position_cost=12.,orientation_cost=0.)
 posture=mink.PostureTask(m,cost=1e-5);posture.set_target_from_configuration(pose)
 joints=[int(m.body(f'{a}{leg}').jntadr[0]) for leg in LEGS for a in ('coxa','femur','tibia')]
 for j in joints:m.jnt_range[j]=[-1.7,1.7]
 # only BL joints participate, other legs and floating root are frozen for the local Mink IK
 moving=[int(m.jnt_dofadr[int(m.body(a+'bl').jntadr[0])]) for a in ('coxa','femur','tibia')]
 freeze=[mink.DofFreezingTask(m,dof_indices=[i for i in range(m.nv) if i not in moving])]
 names=[str(m.joint(int(m.body(a+'bl').jntadr[0])).name) for a in ('coxa','femur','tibia')]
 limits=[mink.ConfigurationLimit(m),mink.VelocityLimit(m,{n:np.array([0.15]) for n in names})]
 jid=np.asarray(m.actuator_trnid[:,0],int);qadr=np.asarray(m.jnt_qposadr[jid],int);dof=np.asarray(m.jnt_dofadr[jid],int)
 limits_tau=m.actuator_ctrlrange.copy()
 ugoal=np.asarray(raised['final_motor_goal'],float).copy();root0=d.qpos[:3].copy();toe0=world['bl'].copy()
 target_error_mm=[];trajectory=[];physical=[];max_torque=0;max_raw_torque=0;saturated=0;loaded_min=6;max_abs_joint=0;min_physical_margin_mm=float("inf")
 def physics_drive(steps,tick,label):
  nonlocal max_torque,max_raw_torque,saturated,loaded_min,max_abs_joint,min_physical_margin_mm
  for j in range(steps):
   raw=7.*(ugoal-d.qpos[qadr])-.32*d.qvel[dof]
   limited=np.clip(raw,limits_tau[:,0],limits_tau[:,1]);d.ctrl[:]=limited
   mj.mj_step(m,d)
   max_torque=max(max_torque,float(np.max(np.abs(limited))))
   max_raw_torque=max(max_raw_torque,float(np.max(np.abs(raw))))
   saturated+=int(np.any(np.abs(raw)>limits_tau[:,1]+1e-7))
   max_abs_joint=max(max_abs_joint,float(np.max(np.abs(d.qpos[qadr]))))
   frame_fz=contacts(m,d);loaded=sum(x>.08 for x in frame_fz.values());loaded_min=min(loaded_min,loaded)
   if loaded>=3:min_physical_margin_mm=min(min_physical_margin_mm,measured_support_margin(m,d,frame_fz))
   if loaded<3 or not np.all(np.isfinite(d.qpos)) or d.qpos[2]<.08:raise RuntimeError('lost three support legs or fell/nonfinite')
  r=record(m,d,toe0);r.update({'time_s':float(d.time),'phase':label,'tick':tick});physical.append(r)

 for k in range(cfg.swing_ticks):
  new=source.next_foot_location(k/cfg.swing_ticks,legidx,state,command)
  state.foot_locations[:,legidx]=new
  pose.update(d.qpos)
  for _ in range(20):
   trans=pose.get_transform_frame_to_world('pupper_toe_bl','site').as_matrix();trans[:3,3]=new
   task.set_target(mink.SE3.from_matrix(trans))
   vel=mink.solve_ik(pose,[task,posture],dt=1.,solver='daqp',damping=1e-8,limits=limits,constraints=freeze)
   pose.update(pose.integrate(vel,dt=1.))
   err=np.linalg.norm(pose.get_transform_frame_to_world('pupper_toe_bl','site').translation()-new)
   if err<.0005:break
  trajectory.append({'t':k*cfg.dt,'target_xyz_m':new.tolist(),'max_joint_rad':float(np.max(np.abs(pose.q[qadr]))),'error_mm':float(err*1000)})
  target_error_mm.append(err*1000)
  for ja in ('coxa','femur','tibia'):
   jn=int(m.body(ja+'bl').jntadr[0]);qa=int(m.jnt_qposadr[jn]);ix=int(np.where(qadr==qa)[0][0]);ugoal[ix]=pose.q[qa]
  physics_drive(round(cfg.dt/m.opt.timestep),k,'pupper_swing')
 # Touchdown: height reverts to baseline from same stock path, hold full joint targets
 state.foot_locations[2,legidx]=world['bl'][2]
 final_world=state.foot_locations[:,legidx].copy()
 pose.update(d.qpos)
 for _ in range(30):
  trans=pose.get_transform_frame_to_world('pupper_toe_bl','site').as_matrix();trans[:3,3]=final_world
  task.set_target(mink.SE3.from_matrix(trans))
  v=mink.solve_ik(pose,[task,posture],dt=1.,solver='daqp',damping=1e-8,limits=limits,constraints=freeze)
  pose.update(pose.integrate(v,dt=1.))
  if np.linalg.norm(pose.get_transform_frame_to_world('pupper_toe_bl','site').translation()-final_world)<.0005:break
 for ja in ('coxa','femur','tibia'):
  jn=int(m.body(ja+'bl').jntadr[0]);qa=int(m.jnt_qposadr[jn]);ix=int(np.where(qadr==qa)[0][0]);ugoal[ix]=pose.q[qa]
 physics_drive(round(1.0/m.opt.timestep),cfg.swing_ticks,'hold_after_touchdown')
 tip=tip_world(m,d,next(l for l in legs if l['suffix']=='bl'))
 result={
  'schema':'pupper_swing_cad30_physical_one_leg_test_v1',
  'source':'Stanford Pupper SwingLegController (MIT) motion equations + Mink local IK + free-base MuJoCo; source vendored in GitHub',
  'model':f'Freenove CAD30 nominal, 30 meshes, μ={mu}, torque cap .7 Nm',
  'four_support_preparation':{k:raised[k] for k in ('status','initial_Pupper_stance_status','min_stance_loaded_at_each_2ms_step','min_support_margin_mm_when_four_grounded','max_joint_abs_rad','max_model_torque_nm','saturation_steps','front_toe_heights_mm')},
  'swing_leg':'bl','pupper_gait_type':'one-leg test on 4 support, not complete crawl','target_rear_forward_mm':shift_forward*1000.,
  'pupper_swing_nodes':cfg.swing_ticks,'pupper_swing_duration_s':cfg.swing_ticks*cfg.dt,
  'max_mink_target_error_mm':max(target_error_mm),'swing_max_joint_abs_rad':max_abs_joint,
  'minimum_loaded_leg_count':loaded_min,'minimum_signed_physical_support_margin_mm':min_physical_margin_mm,'max_torque_Nm':max_torque,'max_raw_torque_Nm':max_raw_torque,'saturation_steps':saturated,
  'original_tip_world_xyz_m':toe0.tolist(),'physical_final_tip_world_xyz_m':tip.tolist(),
  'physical_tip_forward_mm':(tip[1]-toe0[1])*1000.,
  'root_forward_mm':(d.qpos[1]-root0[1])*1000.,
  'end_contacts':physical[-1]['fz_N'],'end_loaded':physical[-1]['loaded'],
  'min_loaded_on_swing_nodes':min(len(v['loaded']) for v in physical if v['phase']=='pupper_swing'),
  'first_half_time_loaded':[len(v['loaded']) for v in physical[:cfg.swing_ticks//2]],
  'trajectory_targets':trajectory,'physics_snapshots':physical,
  'not_a_gait':True,'full_cycle_walked':False,
  'swing_liftoff_logged':any(x['fz_N']['bl']<.01 for x in physical if x['phase']=='pupper_swing'),
  'four_legs_reloaded_at_end':all(physical[-1]['fz_N'][k]>.2 for k in STANCES),
  'actual_one_step_gate_pass':bool(loaded_min>=3 and min_physical_margin_mm>0 and all(physical[-1]['fz_N'][k]>.2 for k in STANCES) and
       any(x['fz_N']['bl']<.01 for x in physical if x['phase']=='pupper_swing') and
       abs(float(tip[1]-toe0[1]))>=.015 and saturated==0 and max_abs_joint<=1.7 and
       all(physical[-1]['fz_N'][k]<.08 for k in ('fl','fr'))),
  'ending_com_m':total_com(m,d).tolist(),
  'final_qpos':d.qpos.copy().tolist(),'final_qvel':d.qvel.copy().tolist(),'final_motor_goal':ugoal.copy().tolist(),
  'geometry_after_one_step':current_geometry(m,d),
 }
 return result

if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--mm',type=float,default=40.);ap.add_argument('--out',type=Path)
 ap.add_argument('--mu',type=float,default=1.1);a=ap.parse_args();r=run_trial(shift_forward=a.mm/1000,mu=a.mu)
 if a.out:a.out.write_text(json.dumps(r,indent=2)+'\n')
 print(json.dumps({k:v for k,v in r.items() if k not in ('trajectory_targets','physics_snapshots','first_half_time_loaded')},indent=2))
