"""Physical 6-to-4 front unload from Pupper stance-prebalanced, raised six-contact CAD30.

Mink handles front-manipulator IK only. PD motors control all joints; MuJoCo
free base, physical contacts and full mass. No custom gait, no root pinning.
"""
from __future__ import annotations
import argparse,json,sys,os
from pathlib import Path
import numpy as np
import mujoco as mj
import mink
from pupper_stance_prebalance import run_prebalance
from six_contact_raise import contacts,total_com
CAD30_ROOT=Path(os.environ.get('CAD30_ROOT',str(Path.cwd()))).resolve()
sys.path.insert(0,str(CAD30_ROOT))
from hexlab.stand_geometry_v04 import load_legs,tip_world,geom_mesh_world_vertices
LEGS=('ml','mr','bl','br','fl','fr');STANCE=('ml','mr','bl','br');FRONT=('fl','fr');REAR=('bl','br');ADJUST=FRONT+REAR;PREFIX='pupper_front_'

def com_margin_4(model,data,contact_map):
 from scipy.spatial import ConvexHull
 com=total_com(model,data)
 pts={k:[] for k in STANCE};floor=int(model.geom('floor').id)
 inv={int(model.geom(f'tibia{k}_collision_0').id):k for k in STANCE}
 for i in range(data.ncon):
  c=data.contact[i];other=int(c.geom2) if int(c.geom1)==floor else int(c.geom1) if int(c.geom2)==floor else -1
  if other in inv:pts[inv[other]].append(np.asarray(c.pos)[:2].copy())
 if len([k for k,v in pts.items() if v])<4:return None
 xy=np.array([np.mean(pts[k],axis=0) for k in STANCE]);eq=ConvexHull(xy).equations
 return float(np.min(-(eq[:,:2]@com[:2]+eq[:,2])))

def run(shift_mm=25.,lift_mm=100.,phase_s=3.0,hold_s=1.0,body_raise_mm=30.,rear_push_mm=0.,mu=1.1):
 prev=run_prebalance(shift_mm,body_raise_mm=body_raise_mm,mu=mu)
 if prev['status'] != 'PHYSICAL_PREBALANCE_PASS':
  raise RuntimeError('Six-contact prebalance gate failed; front liftoff forbidden')
 path=str(CAD30_ROOT/'source/mjcf/scene_local30_nominal.xml');old=mj.MjModel.from_xml_path(path);od=mj.MjData(old);od.qpos[:]=prev['final_qpos'];mj.mj_forward(old,od)
 legs=load_legs(old,od,mj);toe={l['suffix']:tip_world(old,od,l).copy() for l in legs}
 spec=mj.MjSpec.from_file(path)
 for k in ADJUST:
  bid=int(old.body('tibia'+k).id);loc=np.asarray(od.xmat[bid]).reshape(3,3).T@(toe[k]-od.xpos[bid]);spec.body('tibia'+k).add_site(name=PREFIX+k,pos=loc,type=mj.mjtGeom.mjGEOM_SPHERE,size=(.0005,),rgba=(1.,.6,.2,0.))
 model=spec.compile();model.opt.timestep=.002
 model.geom_friction[int(model.geom('floor').id),0]=mu
 for k in LEGS:model.geom_friction[int(model.geom(f'tibia{k}_collision_0').id),0]=mu
 for k in LEGS:
  for part in ('coxa','femur','tibia'):model.jnt_range[int(model.body(part+k).jntadr[0])]=[-1.7,1.7]
 data=mj.MjData(model);data.qpos[:]=prev['final_qpos'];data.qvel[:]=prev['final_qvel']
 # Preserve PD feedforward on restore; mj_forward with zero ctrl would be a different physical state.
 restore_jids=np.asarray(model.actuator_trnid[:,0],int)
 restore_q=np.asarray(model.jnt_qposadr[restore_jids],int)
 restore_d=np.asarray(model.jnt_dofadr[restore_jids],int)
 prev_targets=np.asarray(prev['final_goals'],float)
 data.ctrl[:]=np.clip(7*(prev_targets-data.qpos[restore_q])-.32*data.qvel[restore_d],model.actuator_ctrlrange[:,0],model.actuator_ctrlrange[:,1])
 mj.mj_forward(model,data)
 jids=np.asarray(model.actuator_trnid[:,0],int);qa=np.asarray(model.jnt_qposadr[jids],int);da=np.asarray(model.jnt_dofadr[jids],int);cmd=np.array(prev['final_goals'])
 mcfg=mink.Configuration(model);mcfg.update(data.qpos)
 tasks={k:mink.FrameTask(PREFIX+k,'site',position_cost=20.,orientation_cost=0.) for k in ADJUST}
 post=mink.PostureTask(model,cost=1e-5);post.set_target_from_configuration(mcfg)
 movable=[int(model.jnt_dofadr[int(model.body(part+k).jntadr[0])]) for k in ADJUST for part in ('coxa','femur','tibia')]
 frozen=[mink.DofFreezingTask(model,dof_indices=[j for j in range(model.nv) if j not in movable])]
 joint_names=[str(model.joint(int(model.body(part+k).jntadr[0])).name) for k in ADJUST for part in ('coxa','femur','tibia')]
 limits=[mink.ConfigurationLimit(model),mink.VelocityLimit(model,{n:np.array([0.1]) for n in joint_names})]
 index=[int(np.where(qa==int(model.jnt_qposadr[int(model.body(part+k).jntadr[0])]))[0][0]) for k in ADJUST for part in ('coxa','femur','tibia')]
 recorded=[];min_stance=4;peak=0;saturations=0;maxq=0.;min_margin=float('inf');max_target_error=0.;initial_com=total_com(model,data);initial_fz=contacts(model,data);failed=None
 tau_limit=np.asarray(model.actuator_ctrlrange).copy();steps=round(phase_s/.08)
 def step_physics(n):
  nonlocal peak,saturations,maxq,min_stance,min_margin,failed
  for i in range(n):
   raw=7*(cmd-data.qpos[qa])-.32*data.qvel[da]
   u=np.clip(raw,tau_limit[:,0],tau_limit[:,1]);data.ctrl[:]=u;mj.mj_step(model,data)
   peak=max(peak,float(np.max(abs(u))));saturations+=int(np.any(np.abs(raw)>tau_limit[:,1]+1e-7));maxq=max(maxq,float(np.max(abs(data.qpos[qa]))))
   fz=contacts(model,data);cnt=sum(fz[k]>.08 for k in STANCE);min_stance=min(min_stance,cnt)
   if cnt>=4:
    margin=com_margin_4(model,data,fz)
    if margin is not None:min_margin=min(min_margin,margin*1000)
   if not np.all(np.isfinite(data.qpos)) or data.qpos[2]<.08 or cnt<4:
    failed='fall or loss of four stance legs';break
  return contacts(model,data)
 for tick in range(steps):
  alpha=(tick+1)/steps;alpha=alpha*alpha*(3-2*alpha)
  for k,task in tasks.items():
   z=(lift_mm if k in FRONT else -rear_push_mm)/1000*alpha
   T=mcfg.get_transform_frame_to_world(PREFIX+k,'site').as_matrix();T[:3,3]=toe[k]+[0.,0.,z];task.set_target(mink.SE3.from_matrix(T))
  for it in range(35):
   v=mink.solve_ik(mcfg,[*tasks.values(),post],dt=1.,solver='daqp',damping=1e-8,limits=limits,constraints=frozen)
   mcfg.update(mcfg.integrate(v,dt=1.))
   e=max(np.linalg.norm(mcfg.get_transform_frame_to_world(PREFIX+k,'site').translation()-(toe[k]+[0,0,(lift_mm if k in FRONT else -rear_push_mm)/1000*alpha])) for k in ADJUST)
   if e<.0003:break
  max_target_error=max(max_target_error,e*1000)
  cmd[index]=mcfg.q[qa[index]]
  fz=step_physics(round((phase_s/steps)/model.opt.timestep))
  recorded.append({'phase':'front_lift','t':float(data.time),'fz':fz,'stance_loaded':sum(fz[k]>.08 for k in STANCE),'front_loaded':sum(fz[k]>.08 for k in FRONT),'COM':total_com(model,data).tolist(),'margin_mm':com_margin_4(model,data,fz)})
  if failed:break
 if not failed:
  for i in range(round(hold_s/model.opt.timestep)):
   fz=step_physics(1)
   if failed:break
   if i%50==0:recorded.append({'phase':'hold','t':float(data.time),'fz':fz,'front_loaded':sum(fz[k]>.08 for k in FRONT),'COM':total_com(model,data).tolist(),'margin_mm':com_margin_4(model,data,fz)})
 last=contacts(model,data);front_tips={k:tip_world(model,data,next(l for l in load_legs(model,data,mj) if l['suffix']==k))[2] for k in FRONT}
 out={'schema':'pupper_stance_mink_front_pair_unload_dynamic_v1','mu':mu,'status':'FOUR_CONTACT_STAND_FROM_SIX' if failed is None and all(last[k]<.08 for k in FRONT) and all(last[k]>.2 for k in STANCE) and min_stance==4 and min_margin>=5. and maxq<=1.7 and saturations==0 else 'FRONT_UNLOAD_NOT_CERTIFIED','failed':failed,'initial_six_Fz_N':initial_fz,'final_Fz_N':last,'front_toe_heights_mm':{k:v*1000 for k,v in front_tips.items()},'min_stance_loaded_at_each_2ms_step':min_stance,'min_support_margin_mm_when_four_grounded':min_margin,'max_joint_abs_rad':maxq,'max_model_torque_nm':peak,'saturation_steps':saturations,'max_Mink_target_error_mm':max_target_error,'body_shift_mm':shift_mm,'rear_push_mm':rear_push_mm,'command_front_raise_mm':lift_mm,'starting_COM_xyz_m':initial_com.tolist(),'final_COM_xyz_m':total_com(model,data).tolist(),'initial_six_raise':prev['input_first_6raise'],'initial_Pupper_stance_status':prev['status'],'trace_80ms':recorded,'final_qpos':data.qpos.tolist(),'final_qvel':data.qvel.tolist(),'final_motor_goal':cmd.tolist(),'four_leg_walking_verified':False}
 return out

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--body-shift',type=float,default=25.);p.add_argument('--lift',type=float,default=100.);p.add_argument('--rear-push',type=float,default=0.);p.add_argument('--mu',type=float,default=1.1);p.add_argument('--raise-mm',type=float,default=30.);p.add_argument('--out',type=Path);args=p.parse_args()
 r=run(args.body_shift,args.lift,body_raise_mm=args.raise_mm,rear_push_mm=args.rear_push,mu=args.mu)
 if args.out:args.out.write_text(json.dumps(r,indent=2)+'\n')
 print(json.dumps({k:v for k,v in r.items() if k not in ('trace_80ms','final_qpos','final_qvel','final_motor_goal')},indent=2))
