"""Diagnostic: can free-base servos realize kinematically optimized body poses?
Uses fixed original contact targets, no root welding, no applied external pose constraints.
"""
import json,math,numpy as np,mujoco
from pathlib import Path
import sys
HERE=Path(__file__).resolve().parent
ROOTDIR=HERE if (HERE/'source/mjcf').is_dir() else HERE.parents[1]
if str(ROOTDIR) not in sys.path: sys.path.insert(0,str(ROOTDIR))
MODEL_PATH=ROOTDIR/'source/mjcf/scene_local30_nominal.xml'
GUARD_PATH=ROOTDIR/'results/optimized_four_leg_guard_reference_2026-11-04.json'
from front_pair_trick import SUPPORT,FRONT,contact_points,margin
from hexlab.stand_geometry_v04 import load_legs,tip_world,geom_mesh_world_vertices
from hexlab.step_one_leg_v04 import solve_waypoint,smoothstep
ROOT=str(MODEL_PATH)
RAW=str(GUARD_PATH)

def run(dx,dy,dz,pitch_deg,mu=1.1,seconds=4.0):
 f=json.loads(Path(RAW).read_text())
 model=mujoco.MjModel.from_xml_path(ROOT); data=mujoco.MjData(model);data.qpos[:]=f['qpos'];data.qvel[:]=0;mujoco.mj_forward(model,data)
 model.geom_friction[model.geom('floor').id,0]=mu
 for c in FRONT+SUPPORT: model.geom_friction[model.geom('tibia'+c+'_collision_0').id,0]=mu
 root=model.body('base_link_2').id;legs={l['suffix']:l for l in load_legs(model,data,mujoco)}
 orig_qpos=data.qpos.copy(); start_COM=data.subtree_com[root].copy(); start_pos=data.xpos[root].copy(); old_rot=data.xmat[root].reshape(3,3);up0=old_rot.T@np.array([0.,0.,1.])
 jids=np.array(model.actuator_trnid[:,0],dtype=int); qa=model.jnt_qposadr[jids];da=model.jnt_dofadr[jids]; qstart=data.qpos[qa].copy();qtgt=qstart.copy();ix={c:[int(np.where(qa==q)[0][0]) for q in legs[c]['qadrs']] for c in FRONT+SUPPORT};ref=mujoco.MjData(model)
 basequat=orig_qpos[3:7].copy();qrot=np.empty(4);mujoco.mju_axisAngle2Quat(qrot,np.array([1.,0.,0.]),math.radians(pitch_deg));qnew=np.empty(4);mujoco.mju_mulQuat(qnew,qrot,basequat)
 errs={}
 for c in SUPPORT:
  l=legs[c];ref.qpos[:]=orig_qpos;ref.qpos[:3]+=np.array([dx,dy,dz]);ref.qpos[3:7]=qnew;ref.qvel[:]=0;mujoco.mj_forward(model,ref)
  goal=tip_world(model,data,l).copy();q,err=solve_waypoint(model,ref,mujoco,l,goal,orig_qpos[l['qadrs']]);qtgt[ix[c]]=q;errs[c]=round(err,5)
 N=int(round(seconds/model.opt.timestep));dt=model.opt.timestep;traj=[];saturation=0;peak=0;fatal=None
 for k in range(N):
  t=k*dt;alpha=smoothstep(min(1,t/2.5));cmd=qstart+alpha*(qtgt-qstart)
  unc=7*(cmd-data.qpos[qa])-.32*data.qvel[da];saturation+=int(np.any((unc>model.actuator_ctrlrange[:,1])|(unc<model.actuator_ctrlrange[:,0])))
  data.ctrl[:]=np.clip(unc,model.actuator_ctrlrange[:,0],model.actuator_ctrlrange[:,1]);peak=max(peak,float(np.max(abs(data.ctrl))))
  mujoco.mj_step(model,data)
  if k%10==0 or k==N-1:
   cp=contact_points(model,data);com=data.subtree_com[root].copy();rot=data.xmat[root].reshape(3,3);tilt=math.degrees(math.acos(float(np.clip((rot@up0)[2],-1,1))))
   margins={c:margin(com[:2],[p for leg in SUPPORT if leg!=c for p in cp[leg]]) if sum(len(cp[leg]) for leg in SUPPORT if leg!=c)>=3 else None for c in SUPPORT}
   front_clear={c:float(np.min(geom_mesh_world_vertices(model,data,legs[c]['geom'])[:,2])) for c in FRONT}
   traj.append({'t':round((k+1)*dt,3),'com':np.round(com,5).tolist(),'pos':np.round(data.xpos[root],5).tolist(),'tilt_deg':round(tilt,3),'contacts':{c:bool(cp[c]) for c in SUPPORT+FRONT},'margins_m':margins,'front_clearance_m':front_clear,'torque_nm':float(np.max(np.abs(data.ctrl)))})
   if data.xpos[root][2]<.075 or tilt>30 or not np.all(np.isfinite(data.qpos)):
    fatal='fall_or_nonfinite';break
 last=traj[-1]; hold=[x for x in traj if x['t']>=seconds-.5];
 valid=(fatal is None and all(sum(x['contacts'][c] for c in SUPPORT)==4 and all(not x['contacts'][c] for c in FRONT) and x['tilt_deg']<8 for x in hold) and saturation==0)
 return {'requested_body_delta_m':[dx,dy,dz],'requested_pitch_deg':pitch_deg,'kinematic_IK_errors_m':errs,'initial_COM_m':np.round(start_COM,5).tolist(),'final_COM_m':last['com'],'COM_delta_m':np.round(np.array(last['com'])-start_COM,5).tolist(),'initial_body_pos_m':np.round(start_pos,5).tolist(),'final_body_pos_m':last['pos'],'final_leave_one_foot_margins_m':last['margins_m'],'front_min_clearance_last_m':last['front_clearance_m'],'last_contacts':last['contacts'],'final_tilt_deg':last['tilt_deg'],'peak_tau_nm':round(peak,4),'saturation_step_fraction':round(saturation/max(1,k+1),5),'valid_four_contact_unsaturated':valid,'fail':fatal,'steps_run':k+1,'trace':traj}

if __name__=='__main__':
 import sys
 configs=[(0,0,0,0),(0,-.06,.04,-12),(0,-.08,.02,-12),(0,-.10,.02,-6)]
 for c in configs:
  r=run(*c);print(json.dumps({k:v for k,v in r.items() if k!='trace'}),flush=True)
  open(ROOTDIR/'results'/f'body_pose_dynamics_{str(c[1]).replace("-", "n").replace(".","p")}_{str(c[3]).replace("-","n")}.json','w').write(json.dumps(r,indent=2)+'\n')
