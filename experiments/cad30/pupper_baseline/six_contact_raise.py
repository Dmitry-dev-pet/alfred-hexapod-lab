"""Real free-body MuJoCo 6-support body raise for adapting a ready-made crawl.

No gait or model rewriting; geometry from user CAD30. All actuators PD at
nominal ±0.7Nm leg limit, no pinning of base, dt 0.002s.
"""
from __future__ import annotations
import argparse, json, sys, math, os
from pathlib import Path
import numpy as np
import mujoco as mj
ROOT=Path(os.environ.get('CAD30_ROOT', str(Path.cwd()))).resolve()
if not (ROOT/'source/mjcf/scene_local30_nominal.xml').is_file():
 raise FileNotFoundError('Set CAD30_ROOT to restored private CAD30 directory')
sys.path.insert(0,str(ROOT))
from hexlab.stand_geometry_v04 import load_legs,tip_world
from hexlab.stand_v04 import static_pose,hold_under_gravity
from hexlab.step_one_leg_v04 import solve_waypoint
LEGS=['ml','mr','bl','br','fr','fl']

def contacts(model,data):
 floor=int(model.geom('floor').id)
 legmap={int(model.geom(f'tibia{leg}_collision_0').id):leg for leg in LEGS}
 force={leg:0.0 for leg in LEGS}
 for i in range(data.ncon):
  c=data.contact[i]
  gid=int(c.geom2) if c.geom1==floor else int(c.geom1) if c.geom2==floor else -1
  if gid not in legmap:continue
  f=np.zeros(6);mj.mj_contactForce(model,data,i,f)
  world=c.frame.reshape(3,3).T@f[:3]
  force[legmap[gid]]+=float(world[2])
 return {k:max(0,v) for k,v in force.items()}

def total_com(m,d):return (d.xipos*m.body_mass[:,None]).sum(axis=0)/m.body_mass.sum()

def run(raise_m=.03, duration=2.5,hold_s=1.,mu=1.1):
 m=mj.MjModel.from_xml_path(str(ROOT/'source/mjcf/scene_local30_nominal.xml'));m.opt.timestep=.002
 m.geom_friction[int(m.geom('floor').id),0]=mu
 for leg in LEGS:m.geom_friction[int(m.geom(f'tibia{leg}_collision_0').id),0]=mu
 d=mj.MjData(m);legs=load_legs(m,d,mj);static_pose(m,d,mj,legs,.145,.15)
 stand=hold_under_gravity(m,d,mj,d.qpos.copy(),1.0)
 if stand['last_0p4s_min_contact_legs']<6:raise RuntimeError('six stance lost')
 origin=d.qpos.copy();q0=origin.copy();qtarget=origin.copy()
 toes={l['suffix']:tip_world(m,d,l).copy() for l in legs}
 for l in legs:
  ref=mj.MjData(m);ref.qpos[:]=origin;ref.qpos[2]+=raise_m;mj.mj_forward(m,ref)
  q,e=solve_waypoint(m,ref,mj,l,toes[l['suffix']],origin[l['qadrs']]);
  if e>.0004:raise RuntimeError('target IK high error '+l['suffix']+' '+str(e))
  if np.max(abs(q))>1.700001:raise RuntimeError('joint limit unreachable '+l['suffix'])
  qtarget[l['qadrs']]=q
 jid=np.array(m.actuator_trnid[:,0],int);qa=np.array(m.jnt_qposadr[jid],int);da=np.array(m.jnt_dofadr[jid],int)
 cmd_initial=origin[qa];cmd_target=qtarget[qa]
 cap=m.actuator_ctrlrange.copy(); init_com=total_com(m,d);root_start=d.qpos[:3].copy()
 log=[];min_loaded=6;peak=0;peak_raw=0;saturations=0;max_q=0
 for step in range(round((duration+hold_s)/m.opt.timestep)):
  t=(step+1)*m.opt.timestep
  alpha=min(1.,t/duration);alpha=alpha**2*(3-2*alpha)
  command=cmd_initial+(cmd_target-cmd_initial)*alpha
  raw=7*(command-d.qpos[qa])-.32*d.qvel[da]
  u=np.clip(raw,cap[:,0],cap[:,1]);d.ctrl[:]=u;mj.mj_step(m,d)
  measured=contacts(m,d);n=sum(f>.08 for f in measured.values())
  min_loaded=min(min_loaded,n);peak=max(peak,float(np.max(abs(u))));peak_raw=max(peak_raw,float(np.max(abs(raw))));max_q=max(max_q,float(max(abs(d.qpos[qa]))));saturations+=int(np.any(np.abs(raw)>cap[:,1]+1e-9))
  if step%10==0 or step==round((duration+hold_s)/m.opt.timestep)-1:
   com=total_com(m,d);log.append({'t':round(t,3),'loaded':n,'Fz_N':{k:round(v,4) for k,v in measured.items()},'root_z':float(d.qpos[2]),'com':com.tolist(),'peak_tau_Nm':float(np.max(abs(u)))})
  if not np.all(np.isfinite(d.qpos)) or d.qpos[2]<.07:break
 result={'schema':'cad30_six_grounded_raised_body_dynamic_v1','dt_s':m.opt.timestep,'mu':mu,'nominal_mass_kg':float(m.body_mass.sum()),'target_body_raise_mm':raise_m*1000,'duration_seconds':duration+hold_s,'initial_root_z_m':float(root_start[2]),'end_root_z_m':float(d.qpos[2]),'actual_root_raise_mm':(float(d.qpos[2])-float(root_start[2]))*1000,'min_loaded_six_legs_all_timesteps':min_loaded,'max_model_abs_joint_rad':max_q,'max_actuator_tau_Nm':peak,'max_raw_torque_Nm':peak_raw,'torque_saturation_timesteps':saturations,'tail_min_loaded_legs':min(x['loaded'] for x in log if x['t']>=duration+hold_s-.4),'final_Fz_N':log[-1]['Fz_N'],'initial_COM_xyz_m':init_com.tolist(),'final_COM_xyz_m':total_com(m,d).tolist(),'initial_qpos':origin.tolist(),'final_qpos':d.qpos.tolist(),'final_qvel':d.qvel.tolist(),'initial_motor_targets':cmd_initial.tolist(),'final_motor_targets':cmd_target.tolist(),'samples_50hz':log,'success_6support_raise':False}
 result['success_6support_raise']=bool(min_loaded>=6 and saturations==0 and max_q<=1.7 and (d.qpos[2]-root_start[2])>.018)
 return result

if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--raise-mm',type=float,default=30.);ap.add_argument('--out',type=Path)
 a=ap.parse_args();r=run(raise_m=a.raise_mm/1000.)
 if a.out:a.out.write_text(json.dumps(r,indent=2)+'\n')
 print(json.dumps({k:v for k,v in r.items() if k not in ('samples_50hz','final_qpos','initial_qpos','initial_motor_targets','final_motor_targets','final_Fz_N')},indent=2))
 print('forces',r['final_Fz_N']);print('min loaded',r['min_loaded_six_legs_all_timesteps'],'tail',r['tail_min_loaded_legs'])
