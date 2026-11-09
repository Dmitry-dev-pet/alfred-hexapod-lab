#!/usr/bin/env python3
"""Motor-driven approach then original v8 forward roll on corrected terrain."""
import contextlib,io,json,math,sys,hashlib
from pathlib import Path
import mujoco as mj,numpy as np
WORK=next(p for p in Path(__file__).resolve().parents if (p/'microduck-work/source-manifest.json').is_file());sys.path.insert(0,str(WORK/'microduck-contact'));from collision_model import build,terrain
HERE=Path(__file__).resolve().parent;ROLL=WORK/'microduck-roll/roulade.onnx';STAND=terrain.POLICY;CDT=.02;DT=.0005;EDGE=.25
ROLL_SHA='3d60da08fc13f29c1b57f41977aa898132c0d60042100149d8e775affcbca32b'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def run(ip,mode,duration=10.,approach_deadline=4.5,speed=.24,height_override=None,roll_duration=2.):
 h=(0. if mode=='flat' else .05) if height_override is None else height_override;terrain.EDGE=EDGE
 with contextlib.redirect_stdout(io.StringIO()):
  m,d,c,_=build(ip,h);mj.mj_resetDataKeyframe(m,d,mj.mj_name2id(m,mj.mjtObj.mjOBJ_KEY,'STAND'));sq=m.jnt_qposadr[m.actuator_trnid[:,0]];c.reset(d.qpos);c.last_ts=d.time;mj.mj_forward(m,d)
  p=ip.PolicyInference(m,d,walking_onnx_path=str(STAND),roulade_onnx_path=str(ROLL),roulade_duration=roll_duration,bam_ctrl=c,use_projected_gravity=True,new_cmd_obs=True);terrain.bench.install_encoder_view(p,c,m,d);bq,_=terrain.bench.backlash_indices(m);c.q_target[:]=d.qpos[sq]+d.qpos[bq];p.set_vel_cmd(0,0,0)
 assert not p.use_delay;js=m.actuator_trnid[:,0];lo=m.jnt_range[js,0]+.035;hi=m.jnt_range[js,1]-.035
 floor=mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,'floor');step=mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,'step_top');feet={s:mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,s+'_foot_collision') for s in ['left','right']};root=1;head=mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,'jaw_soft');head_weld=m.body_weldid[head]
 frames=[];trigger=None;handback=None;phase='settle';hit=False;first_hit=None;peak_body=peak_step=peak_env=peak_self=peak_force=peak_torque=0.;angle=frontier=0.;head_pivot=False;reset=False;actual_trigger=None;walk_started=False
 for k in range(round(duration/CDT)):
  t=k*CDT;mj.mj_forward(m,d);bounds={s:terrain.foot_bounds(m,d,g) for s,g in feet.items()};gap=EDGE-max(b['max'][0] for b in bounds.values())
  with contextlib.redirect_stdout(io.StringIO()):
   if k==50:phase='approach';p.set_vel_cmd(0 if mode=='flat' else speed,0,0);walk_started=True
   should=mode=='flat' or (mode=='contact' and hit) or (mode not in ['flat','contact'] and gap<=float(mode)/100)
   if phase=='approach' and should:
    trigger=t;actual_trigger={'root_position_m':d.qpos[:3].tolist(),'world_velocity_mps':d.qvel[:3].tolist(),'toe_gap_m':gap,'had_face_contact':hit};p.trigger_behavior('roulade');phase='roll'
   if phase=='approach' and t>=approach_deadline:phase='hold';p.set_vel_cmd(0,0,0)
   before=p.behavior_mode;p.update_behavior(CDT)
   if before and not p.behavior_mode:phase='recovery';handback=t
  action=p.infer();target=np.clip(p.default_pose+action*p.action_scale,lo,hi);c.q_target[:]=target;p.last_action=((target-p.default_pose)/p.action_scale).astype(np.float32)
  for _ in range(round(CDT/DT)):
   before=float(d.time);c.update();mj.mj_step(m,d);mj.mj_forward(m,d);reset|=abs(float(d.time)-before-DT)>1e-8;assert np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all();supported=False;head_contact=False;peak_torque=max(peak_torque,float(abs(d.actuator_force).max()))
   for i in range(d.ncon):
    ct=d.contact[i];pair=(ct.geom1,ct.geom2);w=np.zeros(6);mj.mj_contactForce(m,d,i,w)
    if floor in pair or step>=0 and step in pair:
     peak_env=max(peak_env,-float(ct.dist));peak_force=max(peak_force,float(np.linalg.norm(w[:3])));supported|=w[0]>.05;other=pair[1] if pair[0] in [floor,step] else pair[0];head_contact|=w[0]>.05 and m.body_weldid[m.geom_bodyid[other]]==head_weld
     if step>=0 and step in pair:
      peak_step=max(peak_step,-float(ct.dist))
      if m.body_weldid[m.geom_bodyid[other]]==root:peak_body=max(peak_body,-float(ct.dist))
      if abs(ct.frame[0])>.85 and w[0]>.05 and float(d.time)>1:
       hit=True
       if first_hit is None:first_hit=float(d.time)
    else:peak_self=max(peak_self,-float(ct.dist))
   if trigger is not None:
    vel=np.empty(6);mj.mj_objectVelocity(m,d,mj.mjtObj.mjOBJ_XBODY,root,vel,1);angle+=float(vel[1])*DT;frontier=max(frontier,angle);q=d.xquat[root];lat=abs(2*(q[2]*q[3]+q[0]*q[1]));head_pivot|=bool(head_contact and .35<angle<3.0 and lat<.8)
  loads={s:0. for s in feet};top={s:0. for s in feet};contacts=[]
  for i in range(d.ncon):
   ct=d.contact[i];pair=(ct.geom1,ct.geom2);w=np.zeros(6);mj.mj_contactForce(m,d,i,w);contacts.append({'geoms':[int(g) for g in pair],'force_n':float(w[0]),'distance_m':float(ct.dist),'pos':ct.pos.tolist(),'normal':ct.frame[:3].tolist()})
   if abs(ct.frame[2])>.85:
    for side,g in feet.items():
     if g in pair and (floor in pair or step>=0 and step in pair):loads[side]+=max(0.,float(w[0]))
     if g in pair and step>=0 and step in pair and ct.pos[2]>=h-.001:top[side]+=max(0.,float(w[0]))
  frames.append({'t':round(float(d.time),6),'phase':phase,**terrain.bench.sample(m,d,0,0),'motor_target_rad':target.tolist(),'toe_gap_m':gap,'foot_bounds':{s:terrain.foot_bounds(m,d,g) for s,g in feet.items()},'foot_loads_n':loads,'step_loads_n':top,'contacts':contacts,'raw_forward_deg':math.degrees(angle),'max_forward_deg':math.degrees(frontier)})
 last=frames[-50:];warnings={str(i):int(w.number) for i,w in enumerate(d.warning) if w.number};quiet=all(f['tilt_deg']<20 and .09<f['trunk_z_m']<.18+h and f['linear_speed_mps']<.06 and f['angular_speed_rps']<.7 and all(v>.05 for v in f['foot_loads_n'].values()) for f in last);top_stance=bool(h and quiet and all(all(v>.05 for v in f['step_loads_n'].values()) for f in last));goal=bool(top_stance and all(all(b['min'][0]>EDGE+.001 for b in f['foot_bounds'].values()) for f in last));roll=bool(frontier>=math.radians(330) and head_pivot);quality=not warnings and not reset and peak_body<.001 and peak_self<.003
 s={'mode':mode,'height_m':h,'edge_x_m':EDGE,'commanded_approach_mps':0 if mode=='flat' else speed,'approach_deadline_s':approach_deadline,'roll_duration_command_s':roll_duration,'trigger_s':trigger,'actual_trigger':actual_trigger,'first_face_contact_s':first_hit,'actual_handback_s':handback,'full_forward_roll':roll,'head_pivot':head_pivot,'max_forward_rotation_deg':math.degrees(frontier),'final_forward_rotation_deg':math.degrees(angle),'standing_final_1s':quiet,'supported_top_stance':top_stance,'step_goal_reached':goal,'status':'PASS' if quality and (roll and quiet if not h else goal) else 'FAIL','max_body_step_penetration_m':peak_body,'max_step_penetration_m':peak_step,'max_environment_penetration_m':peak_env,'max_self_contact_penetration_m':peak_self,'peak_environment_force_n':peak_force,'peak_torque_nm':peak_torque,'warnings':warnings,'physics_reset_detected':reset,'quality_gate':bool(quality),'final_position_m':frames[-1]['qpos'][:3],'duration_s':duration,'frames':len(frames)}
 return {'schema':'microduck-runup-roll-physical/v1','stand_policy_sha256':sha(STAND),'roll_policy_sha256':sha(ROLL),'collision_model_sha256':sha(WORK/'microduck-contact/collision_model.py'),'dt':DT,'control_dt':CDT,'contact_profile':'complete_env_05ms','terrain':{'height_m':h,'edge_x_m':EDGE,'half_size_xy_m':[.6,.6],'center_xy_m':[EDGE+.6,0]},'summary':s,'frames':frames}
def main():
 mf=json.loads((WORK/'microduck-work/source-manifest.json').read_text());mf['_path']=str(WORK/'microduck-work/source-manifest.json');gate=terrain.bench.source_gate(terrain.RL,terrain.BAM,mf);assert sha(ROLL)==ROLL_SHA;ip=terrain.bench.upstream_inference(terrain.RL);ip.ort.InferenceSession=terrain.bench.cpu_session;entries=[]
 cases=[('flat','flat',10.,4.5,.24,None,2.),('6','6',10.,4.5,.24,None,2.),('3','3',10.,4.5,.24,None,2.),('1','1',10.,4.5,.24,None,2.),('contact','contact',10.,4.5,.24,None,2.),('1-long','1',14.,7.,.24,None,2.),('contact-long','contact',14.,7.,.24,None,2.),('contact-fast','contact',14.,7.,.30,None,2.),('10-early','10',14.,7.,.30,None,2.),('flat-moving','1',14.,7.,.24,0.,2.),('1-roll3','1',16.,7.,.24,None,3.),('1-roll4','1',16.,7.,.24,None,4.)]
 for cid,mode,duration,deadline,speed,override,roll_duration in cases:
  name='runup-'+cid+'.json';path=HERE/'traces'/name
  if path.exists():
   tr=json.loads(path.read_text());assert tr['summary']['mode']==mode and tr['summary']['duration_s']==duration
   tr['summary']['approach_deadline_s']=deadline;tr['summary']['roll_duration_command_s']=roll_duration
  else:tr=run(ip,mode,duration,deadline,speed,override,roll_duration)
  path.write_text(json.dumps(tr,allow_nan=False,separators=(',',':'),sort_keys=True)+'\n');entry={**tr['summary'],'trace_file':name,'trace_sha256':sha(path)};entries.append(entry);print(json.dumps(entry),flush=True)
 (HERE/'traces/summary.json').write_text(json.dumps({'schema':'microduck-runup-roll-summary/v1','source_gate':gate,'episodes':entries},indent=2)+'\n')
if __name__=='__main__':main()
