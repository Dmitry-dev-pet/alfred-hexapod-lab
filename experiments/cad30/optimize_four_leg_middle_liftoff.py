"""One physically simulated middle-foot liftoff after optimizing support body pose.
No external root manipulation in simulation, root moves from joint forces only.
"""
import numpy as np,mujoco,math,json
from pathlib import Path
from front_pair_trick import SUPPORT,FRONT,contact_points,margin
from hexlab.stand_geometry_v04 import load_legs,tip_world,geom_mesh_world_vertices
from hexlab.step_one_leg_v04 import solve_waypoint,smoothstep
from optimize_four_leg_pose_dynamics import RAW,ROOT

def run(dx=.06,dy=-.10,dz=.045,pitch=8.,lift=.035,mu=1.1,record=False):
 f=json.loads(Path(RAW).read_text());m=mujoco.MjModel.from_xml_path(ROOT);d=mujoco.MjData(m);d.qpos[:]=f['qpos'];d.qvel[:]=0;mujoco.mj_forward(m,d)
 m.geom_friction[m.geom('floor').id,0]=mu
 for c in SUPPORT+FRONT:m.geom_friction[m.geom('tibia'+c+'_collision_0').id,0]=mu
 L={x['suffix']:x for x in load_legs(m,d,mujoco)};jids=np.array(m.actuator_trnid[:,0],dtype=int);qa=m.jnt_qposadr[jids];da=m.jnt_dofadr[jids];ix={c:np.array([int(np.where(qa==z)[0][0]) for z in L[c]['qadrs']]) for c in SUPPORT+FRONT};qstart=d.qpos[qa].copy();qref=d.qpos.copy();origin=d.xpos[m.body('base_link_2').id].copy();upr=d.xmat[m.body('base_link_2').id].reshape(3,3).T@np.array([0,0,1]);feet={c:tip_world(m,d,L[c]).copy() for c in SUPPORT};root=m.body('base_link_2').id
 def targets(x,y,z,deg,swing=False):
  out=qstart.copy();err={};ref=mujoco.MjData(m);basequat=qref[3:7].copy();qrot=np.empty(4);mujoco.mju_axisAngle2Quat(qrot,np.array([1.,0,0]),math.radians(deg));qnew=np.empty(4);mujoco.mju_mulQuat(qnew,qrot,basequat)
  for c in SUPPORT:
   ref.qpos[:]=qref;ref.qpos[:3]+=np.array([x,y,z]);ref.qpos[3:7]=qnew;ref.qvel[:]=0;mujoco.mj_forward(m,ref)
   target=feet[c].copy();
   if c=='ml' and swing:target[2]+=lift
   q,e=solve_waypoint(m,ref,mujoco,L[c],target,qref[L[c]['qadrs']]);out[ix[c]]=q;err[c]=e
  return out,err
 first,e1=targets(0,-.08,.02,0)
 second,e2=targets(dx,dy,dz,pitch,False)
 airborne,e3=targets(dx,dy,dz,pitch,True)
 steps=int(6./m.opt.timestep);rows=[];frames=[];sat=0;peak=0;fail=None
 for k in range(steps):
  t=k*m.opt.timestep
  if t<2.7:qt=qstart+smoothstep(min(1,t/2.7))*(first-qstart)
  elif t<4.4:qt=first+smoothstep((t-2.7)/1.7)*(second-first)
  else:qt=second+smoothstep(min(1,(t-4.4)/.9))*(airborne-second)
  unc=7*(qt-d.qpos[qa])-.32*d.qvel[da];sat+=int(np.any((unc>m.actuator_ctrlrange[:,1])|(unc<m.actuator_ctrlrange[:,0])))
  d.ctrl[:]=np.clip(unc,m.actuator_ctrlrange[:,0],m.actuator_ctrlrange[:,1]);peak=max(peak,float(np.max(abs(d.ctrl))));mujoco.mj_step(m,d)
  if k%10==0 or k==steps-1:
   cp=contact_points(m,d);com=d.subtree_com[root];rot=d.xmat[root].reshape(3,3);tilt=math.degrees(math.acos(float(np.clip((rot@upr)[2],-1,1))))
   other=[p for c in SUPPORT if c!='ml' for p in cp[c]];mg=margin(com[:2],other) if len(other)>=3 else None
   meshmin={c:float(np.min(geom_mesh_world_vertices(m,d,L[c]['geom'])[:,2])) for c in FRONT+('ml',)}
   rows.append({'t':round(t,3),'COM_xyz':np.round(com,4).tolist(),'contacts':{c:bool(cp[c]) for c in SUPPORT+FRONT},'COM_margin_3_m':mg,'mesh_min_z_m':meshmin,'tilt_deg':tilt,'peak_tau_nm':float(np.max(abs(d.ctrl)))})
   if d.xpos[root][2]<.07 or tilt>40:fail='fall_or_tilt';break
  if record and k%25==0:frames.append({'t':round(t,3),'qpos':np.round(d.qpos,7).tolist()})
 hold=[r for r in rows if 5.3<=r['t']<=5.99]
 outcome=bool(hold and all(not r['contacts']['ml'] and r['mesh_min_z_m']['ml']>.005 and all(r['contacts'][c] for c in ('bl','br','mr')) and not any(r['contacts'][c] for c in FRONT) and r['COM_margin_3_m'] is not None and r['COM_margin_3_m']>.003 and r['tilt_deg']<15 for r in hold) and sat==0 and fail is None)
 return {'params':{'dx':dx,'dy':dy,'dz':dz,'pitch_deg':pitch,'lift_m':lift,'mu':mu},'IK_max_m':max([*e1.values(),*e2.values(),*e3.values()]),'completed':len(rows)>0 and not fail,'failure':fail,'true_middle_lift_safe':outcome,'hold_max_middle_mesh_min_z':max((x['mesh_min_z_m']['ml'] for x in hold),default=None),'hold_last_middle_mesh_min_z':hold[-1]['mesh_min_z_m']['ml'] if hold else None,'hold_middle_contact_free_fraction':np.mean([not x['contacts']['ml'] for x in hold]) if hold else None,'hold_last_com_margin_m':hold[-1]['COM_margin_3_m'] if hold else None,'hold_last_COM':hold[-1]['COM_xyz'] if hold else None,'hold_last_tilt_deg':hold[-1]['tilt_deg'] if hold else None,'min_other_support_contact_count':min((sum(x['contacts'][c] for c in ('bl','br','mr')) for x in hold),default=None),'front_contact_fraction_hold':np.mean([any(x['contacts'][c] for c in FRONT) for x in hold]) if hold else None,'torque_peak_nm':peak,'saturation_fraction':sat/max(1,k+1),'trace':rows if record else [],'qpos_frames':frames if record else []}
if __name__=='__main__':
 import json
 for c in [(.06,-.10,.045,8),(.07,-.10,.06,8),(.07,-.12,.045,8),(.05,-.12,.045,0)]:
  r=run(*c);print(json.dumps(r),flush=True)
