"""Diagnostic test only: cyclic quadrupedal walk with two front legs raised.
Physical MuJoCo with four remaining supports, same CAD30 model; first try.
Open-loop phases and commanded posture, NOT neuroreflex/adaptive controller.
"""
from __future__ import annotations
import json, math, argparse
from pathlib import Path
import mujoco, numpy as np
from front_pair_trick import FRONT,SUPPORT,contact_points,margin
from hexlab.stand_geometry_v04 import load_legs,tip_world,geom_mesh_world_vertices
from hexlab.stand_v04 import static_pose,hold_under_gravity
from hexlab.step_one_leg_v04 import solve_waypoint,smoothstep
ROOT=Path(__file__).resolve().parent

def run(mass='nominal',mu=1.1,shift=.035,step=.025,lift=.018,period=3.2,seconds=9.6,duty=.15,mode='wave',record=False):
 assert 0<step<=.05 and 0<lift<=.05
 m=mujoco.MjModel.from_xml_path(str(ROOT/'source/mjcf'/f'scene_local30_{mass}.xml'))
 m.geom_friction[m.geom('floor').id,0]=mu
 for c in FRONT+SUPPORT:m.geom_friction[m.geom(f'tibia{c}_collision_0').id,0]=mu
 d=mujoco.MjData(m);legs=load_legs(m,d,mujoco);ls={l['suffix']:l for l in legs}
 static_pose(m,d,mujoco,legs,.145,.15);pre=hold_under_gravity(m,d,mujoco,d.qpos.copy(),1.)
 if not pre['stable_by_proxy_thresholds']:return {'error':'stand_failed'}
 q0=d.qpos.copy();bid=m.body('base_link_2').id;initxyz=d.xpos[bid].copy();up0=d.xmat[bid].reshape(3,3).T@np.array([0,0,1]);jids=m.actuator_trnid[:,0];qa=m.jnt_qposadr[jids];da=m.jnt_dofadr[jids];qi=d.qpos[qa].copy()
 idx={c:np.array([np.where(qa==adr)[0][0] for adr in ls[c]['qadrs']]) for c in FRONT+SUPPORT}
 ref=mujoco.MjData(m);goals={}; errors=[]
 for c in FRONT+SUPPORT:
  ref.qpos[:]=q0;ref.qvel[:]=0;mujoco.mj_forward(m,ref);p=tip_world(m,ref,ls[c]);base=q0[ls[c]['qadrs']]
  if c in FRONT:
   q,e=solve_waypoint(m,ref,mujoco,ls[c],p+np.array([0,.05,.105]),base);goals[c]={'guard':q};errors.append(e)
  else:
   qq,ee=solve_waypoint(m,ref,mujoco,ls[c],p+np.array([0,shift,0]),base);errors.append(ee)
   goals[c]={'shift':qq,'cycle':[]}
   for phase in np.linspace(0,1,40,endpoint=False):
    if phase<duty:
     f=phase/duty;h=smoothstep(f)-.5; z=lift*math.sin(math.pi*f)
    else:
     f=(phase-duty)/(1-duty);h=.5-f;z=0.
    q,e=solve_waypoint(m,ref,mujoco,ls[c],p+np.array([0,shift+step*h,z]),base);goals[c]['cycle'].append(q);errors.append(e)
   goals[c]['cycle']=np.asarray(goals[c]['cycle'])
 mx=float(max(errors))
 if mx>.01:return {'error':'unreachable_ik','ik_max_error':mx,'params':dict(shift=shift,step=step,lift=lift)}
 offsets={'bl':0.,'mr':.25,'br':.50,'ml':.75} if mode=='wave' else {'bl':0.,'mr':.5,'br':0.,'ml':.5}
 N=int(round((4.+seconds)/m.opt.timestep));saturated=0;max_tau=0;rows=[];frames=[];first_move=None;fall=None;mean_contact=[]
 for k in range(N):
  t=k*m.opt.timestep;cmd=qi.copy()
  if t<2.:shiftalpha=smoothstep(t/2);frontalpha=0; gaitalpha=0
  elif t<3.2:shiftalpha=1;frontalpha=smoothstep((t-2.)/1.2);gaitalpha=0
  elif t<4.:shiftalpha=1;frontalpha=1;gaitalpha=0
  else:shiftalpha=1;frontalpha=1;gaitalpha=smoothstep(min((t-4.)/.7,1.))
  for c in FRONT:cmd[idx[c]]=qi[idx[c]]+frontalpha*(goals[c]['guard']-qi[idx[c]])
  for c in SUPPORT:
   qq=goals[c]['shift']
   if gaitalpha>0:
    phase=(((t-4.)/period)+offsets[c])%1
    tab=goals[c]['cycle']; frac=phase*len(tab); lo=int(frac)%len(tab);hi=(lo+1)%len(tab)
    qcy=(1-(frac-lo))*tab[lo]+(frac-lo)*tab[hi]
    qq=(1-gaitalpha)*qq+gaitalpha*qcy
   cmd[idx[c]]=qi[idx[c]]+shiftalpha*(qq-qi[idx[c]])
  unc=7*(cmd-d.qpos[qa])-.32*d.qvel[da]
  saturated+=bool(np.any(unc<m.actuator_ctrlrange[:,0]) or np.any(unc>m.actuator_ctrlrange[:,1]));d.ctrl[:]=np.clip(unc,m.actuator_ctrlrange[:,0],m.actuator_ctrlrange[:,1]);max_tau=max(max_tau,float(np.max(abs(d.ctrl))))
  mujoco.mj_step(m,d)
  if k%10==0 or k==N-1:
   cp=contact_points(m,d);ss=[c for c in SUPPORT if cp[c]];ff=[c for c in FRONT if cp[c]];rot=d.xmat[bid].reshape(3,3);tilt=math.degrees(math.acos(np.clip((rot@up0)[2],-1,1)));com=d.subtree_com[bid]
   z={c:float(np.min(geom_mesh_world_vertices(m,d,ls[c]['geom'])[:,2])) for c in SUPPORT+FRONT}
   mar=margin(com[:2],[p for c in SUPPORT for p in cp[c]]) if len(ss)>=3 else None
   rows.append(dict(t=round(t,3),contacts=ss,front_contacts=ff,min_z={c:round(x,4) for c,x in z.items()},tilt=round(tilt,3),advance_m=round(float(d.xpos[bid][1]-initxyz[1]),5),height_m=round(float(d.xpos[bid][2]),5),margin=None if mar is None else round(mar,4),tau=round(float(np.max(abs(d.ctrl))),4)))
   if t>=4:mean_contact.append(len(ss))
   if d.xpos[bid][2]<.065 or tilt>45 or not np.all(np.isfinite(d.qpos)):
    fall='tip_or_drop';break
  if record and k%50==0: frames.append({'t':round(t,3),'qpos':np.round(d.qpos,7).tolist()})
 walk=[r for r in rows if r['t']>=5.];steps={c:max([r['min_z'][c] for r in walk],default=-1) for c in SUPPORT}; minz={c:min([r['min_z'][c] for r in walk],default=-1) for c in FRONT}
 result={'complete':fall is None and k==N-1,'failure':fall,'mass':mass,'mu':mu,'mode':mode,'shift':shift,'step':step,'lift':lift,'period':period,'duty':duty,'duration':seconds,'IK_error_max':mx,
 'legs_show_positive_mesh_clearance':steps,'front_min_mesh_z':minz,'front_contact_steps':sum(bool(r['front_contacts']) for r in walk),'min_support_contact':min([len(r['contacts']) for r in walk],default=-1),'fraction_below_3_supports':sum(len(r['contacts'])<3 for r in walk)/max(1,len(walk)),
 'max_tilt':max([r['tilt'] for r in walk],default=-1),'max_tau':max_tau,'sat_fraction':saturated/N,'max_advance':max([r['advance_m'] for r in walk],default=0),'net_advance':walk[-1]['advance_m']-walk[0]['advance_m'] if walk else 0,'mean_support_contacts':float(np.mean(mean_contact)) if mean_contact else 0,'trace':rows,'frames':frames}
 return result
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--step',type=float,default=.025);p.add_argument('--lift',type=float,default=.018);p.add_argument('--shift',type=float,default=.035);p.add_argument('--period',type=float,default=3.2);p.add_argument('--seconds',type=float,default=9.6);p.add_argument('--mode',choices=['wave','trot'],default='wave');p.add_argument('--mass',default='nominal');p.add_argument('--mu',type=float,default=1.1);p.add_argument('--record',action='store_true');p.add_argument('--out')
 args=p.parse_args();d=vars(args);out=d.pop('out');r=run(**d);print(json.dumps({k:v for k,v in r.items() if k not in ('trace','frames')},ensure_ascii=False,indent=2));
 if out:Path(out).write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
