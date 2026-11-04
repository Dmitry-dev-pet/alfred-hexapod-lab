"""Existing Stanford Pupper StanceController -> Mink -> free-base CAD30 MuJoCo.

A 6-contact body-prebalance experiment from a physically raised, stable
6-grounded stance. Not a 4-leg crawl nor a force-based WBC.
"""
from __future__ import annotations
import argparse,json,sys,os
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import mujoco as mj
import mink
HERE=Path(__file__).resolve().parent
VENDOR=HERE/'vendor'/'stanford_pupper'
if not VENDOR.is_dir():
 VENDOR=HERE.parent/'vendor'/'stanford_pupper'
sys.path.insert(0,str(HERE));sys.path.insert(0,str(VENDOR))
from StanceController import StanceController
from six_contact_raise import run as six_raise,contacts,total_com
CAD30_ROOT=Path(os.environ.get('CAD30_ROOT',str(Path.cwd()))).resolve()
sys.path.insert(0,str(CAD30_ROOT))
from hexlab.stand_geometry_v04 import load_legs,tip_world
CODES=('ml','mr','bl','br','fr','fl');PFX='pupper_toe_'

def run_prebalance(shift_back_mm=25.,duration_s=2.,mu=1.1,body_raise_mm=30.):
 raised=six_raise(raise_m=body_raise_mm/1000.,mu=mu)
 if not raised['success_6support_raise']:
  raise RuntimeError('Six-support raised stance not valid at selected friction; veto subsequent motions')
 model_path=CAD30_ROOT/'source/mjcf/scene_local30_nominal.xml'
 base=mj.MjModel.from_xml_path(str(model_path));db=mj.MjData(base);db.qpos[:]=raised['final_qpos'];mj.mj_forward(base,db)
 legs=load_legs(base,db,mj);orig={l['suffix']:tip_world(base,db,l).copy() for l in legs}
 spec=mj.MjSpec.from_file(str(model_path))
 for l in legs:
  k=l['suffix'];bid=int(base.body('tibia'+k).id)
  rot=np.asarray(db.xmat[bid]).reshape(3,3)
  loc=rot.T@(orig[k]-db.xpos[bid]);spec.body('tibia'+k).add_site(name=PFX+k,pos=loc,type=mj.mjtGeom.mjGEOM_SPHERE,size=(.0005,),rgba=(.2,.8,.3,0.))
 m=spec.compile();m.opt.timestep=.002
 m.geom_friction[int(m.geom('floor').id),0]=mu
 for k in CODES:m.geom_friction[int(m.geom(f'tibia{k}_collision_0').id),0]=mu
 for k in CODES:
  for part in ('coxa','femur','tibia'):
   m.jnt_range[int(m.body(part+k).jntadr[0])]=[-1.7,1.7]
 d=mj.MjData(m);d.qpos[:]=raised['final_qpos'];d.qvel[:]=raised['final_qvel'];mj.mj_forward(m,d)
 saved_q=d.qpos.copy();saved_v=d.qvel.copy();q=np.asarray(saved_q)
 cfg=SimpleNamespace(dt=.05,z_time_constant=1.)
 stance=StanceController(cfg)
 state=SimpleNamespace(foot_locations=np.array([orig[k] for k in CODES]).T.copy(),height=float(np.mean([orig[k][2] for k in CODES])))
 command=SimpleNamespace(horizontal_velocity=np.array([0.,-shift_back_mm/1000./duration_s]),yaw_rate=0.,height=state.height)
 conf=mink.Configuration(m);conf.update(saved_q)
 feet_tasks={k:mink.FrameTask(PFX+k,'site',position_cost=20.,orientation_cost=0.) for k in CODES}
 posture=mink.PostureTask(m,cost=1e-5);posture.set_target_from_configuration(conf)
 joints=[int(m.body(part+k).jntadr[0]) for k in CODES for part in ('coxa','femur','tibia')]
 free=set(int(m.jnt_dofadr[j]) for j in joints)
 constraints=[mink.DofFreezingTask(m,dof_indices=[i for i in range(m.nv) if i not in free])]
 speed={str(m.joint(j).name):np.array([.09]) for j in joints}
 mink_limits=[mink.ConfigurationLimit(m),mink.VelocityLimit(m,speed)]
 ajid=np.asarray(m.actuator_trnid[:,0],int);qa=np.asarray(m.jnt_qposadr[ajid],int);da=np.asarray(m.jnt_dofadr[ajid],int)
 torque_cap=m.actuator_ctrlrange.copy();goals=d.qpos[qa].copy()
 n=int(duration_s/cfg.dt);samples=[];min_loaded=6;maxq=0;peaktau=0;saturated=0;ikmax=0.;stopped=None
 def drive(ns):
  nonlocal min_loaded,maxq,peaktau,saturated,stopped
  for _ in range(ns):
   u=7.*(goals-d.qpos[qa])-.32*d.qvel[da]
   clipped=np.clip(u,torque_cap[:,0],torque_cap[:,1]);d.ctrl[:]=clipped;mj.mj_step(m,d)
   fz=contacts(m,d);count=sum(z>.08 for z in fz.values());min_loaded=min(min_loaded,count)
   maxq=max(maxq,float(np.max(np.abs(d.qpos[qa]))));peaktau=max(peaktau,float(np.max(abs(clipped))))
   saturated+=int(np.any(abs(u)>torque_cap[:,1]+1e-7))
   if count<5 or d.qpos[2]<.08:stopped='insufficient_ground_contacts_or_fall';break
  return contacts(m,d)
 for tick in range(n):
  for j,k in enumerate(CODES):state.foot_locations[:,j]=stance.next_foot_location(j,state,command)
  targets={k:state.foot_locations[:,i].copy() for i,k in enumerate(CODES)}
  for k,task in feet_tasks.items():
   T=conf.get_transform_frame_to_world(PFX+k,'site').as_matrix();T[:3,3]=targets[k]
   task.set_target(mink.SE3.from_matrix(T))
  for it in range(25):
   v=mink.solve_ik(conf,[*feet_tasks.values(),posture],dt=1.,solver='daqp',damping=1e-8,limits=mink_limits,constraints=constraints)
   conf.update(conf.integrate(v,dt=1.))
   maxerr=max(np.linalg.norm(conf.get_transform_frame_to_world(PFX+k,'site').translation()-targets[k]) for k in CODES)
   if maxerr<.0003:break
  ikmax=max(ikmax,float(maxerr*1000))
  goals[:]=conf.q[qa]
  fz=drive(round(cfg.dt/m.opt.timestep))
  samples.append({'phase':'stance','tick':tick,'time':float(d.time),'root_xy_m':np.asarray(d.qpos[:2]).tolist(),'COM_xyz_m':total_com(m,d).tolist(),'Fz':fz,'min_loaded':sum(z>.08 for z in fz.values()),'ik_error_mm':float(maxerr*1000)})
  if stopped:break
 if not stopped:
  for tick in range(round(1./m.opt.timestep)):
   drive(1)
   if stopped:break
   if tick%25==0:samples.append({'phase':'hold','tick':tick,'time':float(d.time),'root_xy_m':np.asarray(d.qpos[:2]).tolist(),'Fz':contacts(m,d)})
 result={'schema':'pupper_stance_six_support_physical_v1','status':'PHYSICAL_PREBALANCE_PASS' if not stopped and min_loaded>=6 and saturated==0 and maxq<=1.7 else 'PHYSICAL_PREBALANCE_NOT_VERIFIED','blocked':stopped,'target_body_back_mm':shift_back_mm,'measured_root_displacement_Y_mm':float((d.qpos[1]-saved_q[1])*1000),'min_loaded_supports_all_physics_ticks':min_loaded,'max_joint_rad':maxq,'peak_torque_Nm':peaktau,'saturation_ticks':saturated,'max_mink_site_error_mm':ikmax,'initial_com_m':raised['final_COM_xyz_m'],'final_com_m':total_com(m,d).tolist(),'input_first_6raise':{k:raised[k] for k in ('success_6support_raise','actual_root_raise_mm','min_loaded_six_legs_all_timesteps','max_model_abs_joint_rad')},'final_fz_N':contacts(m,d),'samples':samples,'final_qpos':d.qpos.tolist(),'final_qvel':d.qvel.tolist(),'final_goals':goals.tolist(),'original_full_4leg_gait':False}
 return result

if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--shift-mm',type=float,default=25.);ap.add_argument('--out',type=Path);args=ap.parse_args()
 r=run_prebalance(args.shift_mm)
 if args.out:args.out.write_text(json.dumps(r,indent=2)+'\n')
 print(json.dumps({k:v for k,v in r.items() if k not in ('samples','final_qpos','final_qvel','final_goals')},indent=2))
