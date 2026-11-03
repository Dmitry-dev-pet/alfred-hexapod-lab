"""MuJoCo local, approximate v0.3 18-DOF FNK0052 gait research.

Physical simulation with no kinematic body translation. Not CAD-v0.4.
Try tripod vs ripple and quantify real contact, drift, speed, falls.
"""
import argparse, json, math, sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import mujoco
from hexlab.mujoco_model import LEGS, LINKS, write_model

S_WING=.34  # Tripod: 34% swing; wave uses a shorter 14% swing
T_PERIOD=2.0
STEP=0.040
LIFT=.025
NOMINAL_Z=.165
FOOT_CENTRE_Z=.013

def smooth(s):
 s=max(0.,min(1.,s));return s*s*(3-2*s)

def ik(xyz, leg):
 name,mount,heading=leg
 dx,dy=xyz[0]-mount[0],xyz[1]-mount[1]
 theta=math.atan2(dy,dx)
 yaw=(theta-math.radians(heading)+math.pi)%(2*math.pi)-math.pi
 r=math.hypot(dx,dy)-LINKS[0]
 drop=NOMINAL_Z-xyz[2]
 c=(r*r+drop*drop-LINKS[1]**2-LINKS[2]**2)/(2*LINKS[1]*LINKS[2])
 reachable=abs(c)<=1.0
 knee=math.acos(max(-1.,min(1.,c)))
 hip=math.atan2(drop,r)-math.atan2(LINKS[2]*math.sin(knee),LINKS[1]+LINKS[2]*math.cos(knee))
 return [yaw,hip,knee],reachable

def run(gait='tripod', seconds=12., step=.04, period=2., lift=.025, mass=1.80):
 path=ROOT/'models/freenove_fnk0052_approx.xml'
 if not path.exists(): write_model(path)
 model=mujoco.MjModel.from_xml_path(str(path))
 # add the extra mass at torso, preserving x/y distribution only approximately
 extra=mass-sum(model.body_mass)
 torso=model.body('torso').id
 old_torso_mass=float(model.body_mass[torso])
 model.body_mass[torso]+=extra
 model.body_inertia[torso]*=float(model.body_mass[torso])/old_torso_mass
 data=mujoco.MjData(model)
 key=model.key('stand').id;mujoco.mj_resetDataKeyframe(model,data,key);mujoco.mj_forward(model,data)
 roots=np.array([[float(mount[i]) for i in range(3)] for _,mount,_ in LEGS])
 local0=[]
 for i,(name,_,_) in enumerate(LEGS):
  sid=model.site(f'{name}_foot_site').id
  p=data.site_xpos[sid]-data.qpos[:3]
  local0.append(p)
 local0=np.array(local0)
 n=int(seconds/model.opt.timestep)
 report=[]
 sampled_states=[]
 root0=np.array(data.qpos[:3]).copy()
 swing_fraction = S_WING if gait=="tripod" else .14
 phases=[]
 for i in range(6):
  if gait=='tripod': phases.append((i%2)*.5)
  else: phases.append(i/6)
 all_unreachable=0; max_qerr=0; joint_out=0; saturated_samples=0
 for k in range(n):
  t=k*model.opt.timestep
  if t < 1.: 
   data.ctrl[:]=model.key_ctrl[key]
  else:
   tm=t-1.
   # ramp from standing to periodic gait over first cycle
   amp=smooth(tm/period)
   for i,leg in enumerate(LEGS):
    phase=(tm/period + phases[i])%1
    if phase < swing_fraction:
     u=smooth(phase/swing_fraction)
     foot_x=step*(u-.5)
     foot_z=FOOT_CENTRE_Z+lift*math.sin(math.pi*phase/swing_fraction)
    else:
     u=(phase-swing_fraction)/(1-swing_fraction)
     foot_x=step*(.5-u)
     foot_z=FOOT_CENTRE_Z
    p=local0[i].copy();p[0]+=amp*foot_x;p[2]=FOOT_CENTRE_Z+amp*(foot_z-FOOT_CENTRE_Z)
    angles,possible=ik(p,leg)
    if not possible:all_unreachable+=1
    targets=np.array(angles)
    limits=np.array([[-.65,.65],[-.35,1.25],[0,2.]])
    if np.any(targets<limits[:,0]) or np.any(targets>limits[:,1]):joint_out+=1
    data.ctrl[i*3:i*3+3]=np.clip(targets,limits[:,0],limits[:,1])
    max_qerr=max(max_qerr,float(np.max(np.abs(data.qpos[7+i*3:10+i*3]-data.ctrl[i*3:i*3+3]))))
  mujoco.mj_step(model,data)
  if np.any(abs(data.actuator_force) >= .7 - 1e-5): saturated_samples+=1
  if k%10==0:
   found=set();forces=[]
   floor=model.geom('floor').id
   for ci in range(data.ncon):
    con=data.contact[ci]
    ga,gb=int(con.geom1),int(con.geom2)
    if floor not in (ga,gb): continue
    other=gb if ga==floor else ga
    body=int(model.geom_bodyid[other]);bname=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,body) or ''
    for idx,(name,_,_) in enumerate(LEGS):
     if bname.startswith(name): found.add(idx)
   root=np.array(data.qpos[:3]); rot=data.xmat[torso].reshape(3,3)
   tilt=math.degrees(math.acos(float(np.clip(rot[2,2],-1,1))))
   report.append({'t':round(t,3),'x':round(float(root[0]-root0[0]),5),'y':round(float(root[1]-root0[1]),5),'z':round(float(root[2]),5),'tilt':round(tilt,2),'contact_legs':len(found),'legs':sorted(found),'max_ctrl':round(float(np.max(np.abs(data.actuator_force))),3)})
   if k%100==0: sampled_states.append((round(t,3),list(map(float,data.qpos))))
   if not np.all(np.isfinite(data.qpos)) or root[2]<.055: break
 end=report[-1]
 stats={'mode':gait,'seconds_executed':round(end['t'],3),'body_dx_m':end['x'],'body_dy_m':end['y'],'body_z_final_m':end['z'],'max_tilt_deg':max(s['tilt'] for s in report),'min_height_m':min(s['z'] for s in report),'mean_leg_contacts_after_3s':round(float(np.mean([s['contact_legs'] for s in report if s['t']>3.])),3),'min_leg_contacts_after_3s':min([s['contact_legs'] for s in report if s['t']>3.] or [6]),'max_abs_actuator_torque_nm':max(s['max_ctrl'] for s in report),'actuator_saturation_fraction':round(saturated_samples/n,4),'num_unreachable_ik':all_unreachable,'num_targets_out_of_joint_range':joint_out,'max_joint_tracking_error_rad':round(max_qerr,3),'is_v03_approximation':True,'total_mass_estimated_kg':mass,'step_length_command_m':step,'swing_lift_command_m':lift,'period_s':period,'swing_duty_fraction':swing_fraction}
 return stats,report,sampled_states

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--gait',choices=['tripod','ripple'],default='tripod');p.add_argument('--seconds',type=float,default=12);p.add_argument('--step',type=float,default=.04);p.add_argument('--period',type=float,default=2);p.add_argument('--lift',type=float,default=.025);p.add_argument('--mass',type=float,default=1.80);p.add_argument('--out',type=Path,default=ROOT/'results/local_approx_experiment.json')
 a=p.parse_args();s,r,states=run(a.gait,a.seconds,a.step,a.period,a.lift,a.mass)
 a.out.parent.mkdir(parents=True,exist_ok=True)
 a.out.write_text(json.dumps({'metrics':s,'samples':r,'snapshots':states},indent=2))
 print(json.dumps(s,indent=2))