"""Diagnostic: high front legs, one rear leg step under 3 remaining supports."""
from pathlib import Path
import sys,math,json
import numpy as np
import mujoco
from front_pair_trick import FRONT,SUPPORT,contact_points,margin
from shadowbox_front_pair import interpolate_keys
from hexlab.stand_geometry_v04 import load_legs,tip_world,geom_mesh_world_vertices
from hexlab.stand_v04 import static_pose,hold_under_gravity
from hexlab.step_one_leg_v04 import solve_waypoint,smoothstep
ROOT=Path(__file__).resolve().parent

def run(foot='bl',step_m=.020,lift_m=.025,mu=1.1,mass='nominal',shift=.040, record=False):
 assert foot in SUPPORT
 model=mujoco.MjModel.from_xml_path(str(ROOT/'source/mjcf'/f'scene_local30_{mass}.xml'))
 model.geom_friction[model.geom('floor').id,0]=mu
 for c in SUPPORT+FRONT:model.geom_friction[model.geom(f'tibia{c}_collision_0').id,0]=mu
 data=mujoco.MjData(model);legs=load_legs(model,data,mujoco);look={l['suffix']:l for l in legs}
 static_pose(model,data,mujoco,legs,.145,.15)
 s=hold_under_gravity(model,data,mujoco,data.qpos.copy(),1.)
 assert s['stable_by_proxy_thresholds']
 q0=data.qpos.copy();jid=np.array(model.actuator_trnid[:,0],dtype=int)
 qa=np.array([model.jnt_qposadr[j] for j in jid]);da=np.array([model.jnt_dofadr[j] for j in jid]);idx={c:np.array([np.where(qa==adr)[0][0] for adr in look[c]['qadrs']]) for c in FRONT+SUPPORT}
 init=data.qpos[qa].copy();ref=mujoco.MjData(model)
 goals={};errors={}
 for c in FRONT+SUPPORT:
  l=look[c];ref.qpos[:]=q0;ref.qvel[:]=0;mujoco.mj_forward(model,ref)
  tip=tip_world(model,ref,l).copy()
  additions={}
  if c in FRONT:additions={'raised':np.array([0,.05,.105])}
  else:additions={'shift':np.array([0,shift,0])}
  if c==foot:
   additions.update({'step_up':np.array([0,shift,lift_m]),'step_fw':np.array([0,shift+step_m,lift_m]),'step_down':np.array([0,shift+step_m,0])})
  goals[c]={}
  for k,off in additions.items():
   qq,err=solve_waypoint(model,ref,mujoco,l,tip+off,q0[l['qadrs']]);goals[c][k]=qq
   errors[c+':'+k]=float(err)
 if max(errors.values())>.008:return {'fail':'unreachable_ik','errors':errors}
 keys={}
 for c in SUPPORT:
  baseline=init[idx[c]];g=goals[c]['shift']
  if c==foot:
   keys[c]=[(0.,baseline),(2.,g),(3.5,g),(4.2,goals[c]['step_up']),(4.8,goals[c]['step_fw']),(5.5,goals[c]['step_down']),(6.3,goals[c]['step_down']),(7.5,goals[c]['step_down'])]
  else:keys[c]=[(0.,baseline),(2.,g),(7.5,g)]
 for c in FRONT:
  b=init[idx[c]];g=goals[c]['raised'];keys[c]=[(0.,b),(2.,b),(3.1,g),(7.5,g)]
 dt=float(model.opt.timestep);N=round(7.5/dt);tr=[];frames=[];sat=0;maxtau=0.;failure='';leave_one_snapshot=None;root=model.body('base_link_2').id;start=data.xpos[root].copy()
 up0=data.xmat[root].reshape(3,3).T@np.array([0,0,1])
 for k in range(N):
  t=k*dt;target=init.copy()
  for c in FRONT+SUPPORT:target[idx[c]]=interpolate_keys(t,keys[c])
  unclip=7.*(target-data.qpos[qa])-.32*data.qvel[da]
  bounds=model.actuator_ctrlrange;sat+=int(np.any(unclip<bounds[:,0]) or np.any(unclip>bounds[:,1]));data.ctrl[:]=np.clip(unclip,bounds[:,0],bounds[:,1]);maxtau=max(maxtau,float(np.max(abs(data.ctrl))))
  mujoco.mj_step(model,data)
  if k%10==0:
   com=data.subtree_com[root];cp=contact_points(model,data);support_now=[c for c in SUPPORT if cp[c]];fr_now=[c for c in FRONT if cp[c]]
   mar=margin(com[:2],[p for c in SUPPORT if cp[c] for p in cp[c]]) if sum(bool(cp[c]) for c in SUPPORT)>=3 else None
   if leave_one_snapshot is None and t>=3.20 and all(cp[c] for c in SUPPORT):
    leave_one_snapshot={c:float(margin(com[:2],[p for other in SUPPORT if other!=c for p in cp[other]])) for c in SUPPORT}
   z=geom_mesh_world_vertices(model,data,look[foot]['geom'])[:,2].min();rot=data.xmat[root].reshape(3,3); tilt=math.degrees(math.acos(np.clip((rot@up0)[2],-1,1)))
   tr.append({'t':round(t,3),'contacts':support_now,'front_contacts':fr_now,'step_min_z':round(float(z),5),'margin':None if mar is None else round(mar,5),'tilt':round(tilt,4),'base_y':round(float(data.xpos[root][1]),5),'com_y':round(float(com[1]),5),'stepping_foot_y':round(float(tip_world(model,data,look[foot])[1]),5)})
   if tilt>30 or data.xpos[root][2]<.07:failure='tip/drop';break
  if record and k%50==0:frames.append({'t':round(t,3),'qpos':np.round(data.qpos,7).tolist()})
 interval=[r for r in tr if 4.05<=r['t']<5.20]
 report={'foot':foot,'step':step_m,'lift':lift_m,'shift':shift,'mass':mass,'mu':mu,'complete':not failure and k==N-1,'failure':failure,
  'under_3_contacts':sum(len(r['contacts'])<3 for r in interval),'all_front_clear':all(not r['front_contacts'] for r in interval),
  'step_clearance_max':max(r['step_min_z'] for r in interval),'step_clearance_min':min(r['step_min_z'] for r in interval),
  'min_margin':min((r['margin'] or -1.) for r in interval),'max_tilt':max(r['tilt'] for r in interval),
  'base_advance':tr[-1]['base_y']-tr[0]['base_y'],'leave_one_support_COM_margins_m_at_3p2s':leave_one_snapshot,'saturation_frac':sat/N,'max_tau':maxtau,'trace':tr,'qpos_frames':frames}
 return report
if __name__=='__main__':
 import argparse
 p=argparse.ArgumentParser();p.add_argument('--foot',default='bl');p.add_argument('--step',type=float,default=.01);p.add_argument('--lift',type=float,default=.01);p.add_argument('--shift',type=float,default=.04);p.add_argument('--record',action='store_true');p.add_argument('--out')
 a=p.parse_args();r=run(a.foot,a.step,lift_m=a.lift,shift=a.shift,record=a.record); print(json.dumps({k:v for k,v in r.items() if k not in ('trace','qpos_frames')},indent=2));
 if a.out:Path(a.out).write_text(json.dumps(r,indent=2))
