"""MuJoCo CAD30 nominal: re-balance unsafe pose B, then micro-WBC tests.

Requires private model and saved dynamic replay; no third-party STL copied.
Physics checks evaluate actual mj_step 2ms; this is NOT walking/MPC.
"""
from __future__ import annotations
import argparse,json,math
from pathlib import Path
import numpy as np,mujoco as mj
from scipy.optimize import linprog,minimize
from scipy.spatial import ConvexHull
from cad30_pose_b_jacobian_check import prepare
from wbc_contact_correction import micro_correct_stance

STANCE=('ml','mr','bl','br')
ALL=('ml','mr','bl','br','fl','fr')

class Experiment:
 def __init__(self,model_path,replay_path,mu=1.1):
  self.m,self.d,self.anchors,*_=prepare(model_path,replay_path)
  self.m.geom_friction[self.m.geom('floor').id,0]=mu
  for k in ALL:self.m.geom_friction[self.m.geom(f'tibia{k}_collision_0').id,0]=mu
  self.mu=mu; self.initial=self.d.qpos.copy(); self.base_id=int(self.m.body('base_link_2').id)
  m=self.m
  self.ids=[int(m.body(f'{joint}{leg}').jntadr[0]) for leg in ALL for joint in ('coxa','femur','tibia')]
  self.qads=np.array([m.jnt_qposadr[j] for j in self.ids],int)
  self.dofs=np.array([m.jnt_dofadr[j] for j in self.ids],int)
  self.names=[m.joint(j).name for j in self.ids]
  self.act_ids=m.actuator_trnid[:,0].astype(int)
  self.aq=m.jnt_qposadr[self.act_ids].astype(int)
  self.av=m.jnt_dofadr[self.act_ids].astype(int)
  self.cap=m.actuator_ctrlrange[:,1].copy()
  self.floor=int(m.geom('floor').id)
  self.gids={int(m.geom(f'tibia{k}_collision_0').id):k for k in ALL}
  self.prev_comm=self.d.qpos[self.aq].copy()
  self.max_unclipped=0.;self.saturated=0;self.rows=[];self.events=[]
  self.sum_foot_xy=None;self.max_foot_drift_mm=0.;self.lp_checks=[]
  self.every_step={'minimum_loaded_contacts':6,'minimum_support_margin_mm':1e6,'minimum_stance_Fz_N':1e6, 'max_abs_joint_after_safe_rad':0.,'max_abs_tau_Nm':0.};self.bad_step_count=0;self.safe_phase=False

 def observation(self):
  m,d=self.m,self.d
  p={k:[] for k in ALL};f={k:np.zeros(3) for k in ALL}
  for i in range(d.ncon):
   c=d.contact[i]
   geom=int(c.geom2) if int(c.geom1)==self.floor else int(c.geom1) if int(c.geom2)==self.floor else -1
   if geom not in self.gids: continue
   k=self.gids[geom]
   cv=np.zeros(6); mj.mj_contactForce(m,d,i,cv)
   world=c.frame.reshape(3,3).T@cv[:3]
   p[k].append((np.asarray(c.pos).copy(),max(0.,float(world[2]))))
   f[k]+=world
  support=tuple(k for k in STANCE if f[k][2]>.08)
  com=np.sum(d.xipos*m.body_mass[:,None],axis=0)/m.body_mass.sum()
  pos={k:np.average([z for z,w in p[k]],axis=0,weights=[max(w,1e-8) for z,w in p[k]]) if p[k] else None for k in ALL}
  if len(support)>=3:
   hull=ConvexHull(np.array([pos[k][:2] for k in support])).equations
   margin=float(np.min(-(hull[:,:2]@com[:2]+hull[:,2])))
  else:margin=float('-inf')
  return {'support':support,'com':com,'margin':margin,'force':f,'position':pos}

 def matrices(self,obs):
  m,d=self.m,self.d
  jb=np.zeros((12,6));jq=np.zeros((12,18))
  for i,k in enumerate(STANCE):
   pt=obs['position'][k]
   if pt is None:raise ValueError('Missing contact')
   jp=np.zeros((3,m.nv));jr=np.zeros((3,m.nv))
   mj.mj_jac(m,d,jp,jr,pt,int(m.body('tibia'+k).id))
   jb[3*i:3*i+3]=jp[:,:6];jq[3*i:3*i+3]=jp[:,self.dofs]
  return jb,jq

 def quasi_static_lp(self,obs,motor_frac_cap=.9):
  if tuple(obs['support'])!=STANCE:return {'feasible':False,'reason':'not four loaded contacts'}
  m,d=self.m,self.d
  pts=np.array([obs['position'][k] for k in STANCE]); com=obs['com']
  aeq=np.zeros((6,12));beq=np.r_[np.array([0,0,m.body_mass.sum()*(-m.opt.gravity[2])]),np.zeros(3)]
  for k,pt in enumerate(pts):
   aeq[:3,k*3:k*3+3]=np.eye(3)
   v=pt-com; vx,vy,vz=v
   aeq[3:,k*3:k*3+3]=[[0,-vz,vy],[vz,0,-vx],[-vy,vx,0]]
  temp=mj.MjData(m);temp.qpos[:]=d.qpos;temp.qvel[:]=0;temp.qacc[:]=0;mj.mj_forward(m,temp)
  h_all=np.zeros(m.nv);mj.mj_rne(m,temp,0,h_all)
  h=h_all[self.av]
  G=np.zeros((m.nu,12))
  for i,k in enumerate(STANCE):
   jp=np.zeros((3,m.nv));jr=np.zeros((3,m.nv))
   mj.mj_jac(m,temp,jp,jr,pts[i],int(m.body('tibia'+k).id))
   G[:,i*3:i*3+3]=jp[:,self.av].T
  au=[];bu=[];mu=self.mu
  for i in range(4):
   for ang in np.linspace(0,2*math.pi,32,endpoint=False):
    a=np.zeros(12);a[i*3:i*3+3]=[math.cos(ang),math.sin(ang),-mu*math.cos(math.pi/32)]
    au.append(a);bu.append(0.)
  cap=self.cap*motor_frac_cap
  au.extend(list(-G));bu.extend(cap-h)
  au.extend(list(G));bu.extend(cap+h)
  sol=linprog(np.zeros(12),A_ub=au,b_ub=bu,A_eq=aeq,b_eq=beq,bounds=[(None,None),(None,None),(.2,None)]*4,method='highs')
  if not sol.success:return {'feasible':False,'reason':sol.message,'motor_fraction_cap':motor_frac_cap}
  tau=h-G@sol.x
  F=sol.x.reshape((4,3))
  return {'feasible':True,'motor_fraction_cap':motor_frac_cap,'force_minimum_N':.2,'lp_max_tau_ratio':round(float(np.max(abs(tau)/self.cap)),6),
          'predicted_Fz_N':[round(float(x),4) for x in F[:,2]],'max_eq_error':float(np.max(abs(aeq@sol.x-beq)))}

 def choose_safe_target(self,target=1.66):
  obs=self.observation(); jb,jq=self.matrices(obs); J=np.c_[jb,jq]
  q=self.d.qpos[self.qads].copy();scales=np.r_[np.full(3,.005),np.full(3,.008),np.full(18,.03)];H=np.diag(1/scales**2)
  bounds=[(-.012,.012)]*3+[(-.012,.012)]*3+[(max(-.12,-target-x),min(.12,target-x)) for x in q]
  z=np.zeros(24)
  for i,n in enumerate(self.names):
   if n=='motor31':z[6+i]=min(0,target-q[i])
   if n=='motor27':z[6+i]=max(0,-target-q[i])
  sol=minimize(lambda x:.5*x@H@x,z,jac=lambda x:H@x,method='SLSQP',bounds=bounds,
    constraints={'type':'eq','fun':lambda x:J@x,'jac':lambda x:J},options={'ftol':1e-10,'maxiter':400})
  if not sol.success or np.max(abs(J@sol.x))>5e-6:raise RuntimeError(f'Safe pose optimizer failed: {sol.message}')
  target_q=self.d.qpos[self.aq].copy()
  for j,qa in enumerate(self.qads):
   ai=int(np.flatnonzero(self.aq==qa)[0]);target_q[ai]+=sol.x[6+j]
  self.events.append({'event':'safe_pose_optimizer','linear_desired_body_delta':np.round(sol.x[:6],8).tolist(),
                      'largest_joint_delta_rad':float(max(abs(sol.x[6:]))),'joint_limit_target_rad':target})
  return target_q

 def step_control(self,goal,phase,log_rate=50):
  m,d=self.m,self.d
  u=7.*(goal-d.qpos[self.aq])-.32*d.qvel[self.av]
  self.max_unclipped=max(self.max_unclipped,float(np.max(abs(u))))
  self.saturated+=int(np.any(abs(u)>self.cap+1e-7))
  d.ctrl[:]=np.clip(u,-self.cap,self.cap)
  mj.mj_step(m,d)
  obs=self.observation()
  self.every_step['minimum_loaded_contacts']=min(self.every_step['minimum_loaded_contacts'],len(obs['support']))
  self.every_step['minimum_support_margin_mm']=min(self.every_step['minimum_support_margin_mm'],1000*obs['margin'])
  self.every_step['minimum_stance_Fz_N']=min(self.every_step['minimum_stance_Fz_N'],*(float(obs['force'][k][2]) for k in STANCE))
  self.every_step['max_abs_tau_Nm']=max(self.every_step['max_abs_tau_Nm'],float(np.max(abs(d.actuator_force[:18]))))
  if self.safe_phase:
   self.every_step['max_abs_joint_after_safe_rad']=max(self.every_step['max_abs_joint_after_safe_rad'],float(max(abs(d.qpos[self.qads]))))
  if len(obs['support'])!=4 or not np.isfinite(obs['margin']) or obs['margin']<0:
   self.bad_step_count+=1
   if self.bad_step_count<=5:self.events.append({'event':'bad_contact_or_margin','time_s':float(d.time),'phase':phase,'contacts':len(obs['support']),'margin_m':float(obs['margin'])})
  if int(round(d.time/m.opt.timestep))%log_rate==0:
   points=np.array([obs['position'][k][:2] if obs['position'][k] is not None else [np.nan,np.nan] for k in STANCE])
   if self.sum_foot_xy is not None and np.all(np.isfinite(points)):
    slip=float(np.max(np.linalg.norm(points-self.sum_foot_xy,axis=1)))*1000
    self.max_foot_drift_mm=max(self.max_foot_drift_mm,slip)
   self.rows.append({'t':round(float(d.time),3),'phase':phase,'contact_legs':len(obs['support']),
     'margin_mm':round(obs['margin']*1000,4),'com_y_mm':round(obs['com'][1]*1000,4),
     'max_joint_abs_rad':round(float(max(abs(d.qpos[self.qads]))),6),
     'min_stance_Fz_N':round(float(min(obs['force'][k][2] for k in STANCE)),6),
     'peak_actuator_tau_Nm':round(float(max(abs(d.actuator_force[:18]))),6),
     'max_contact_xy_drift_mm':round(float(self.max_foot_drift_mm),5)})
  return obs

 def ramp(self,initial,final,duration,phase):
  n=int(round(duration/self.m.opt.timestep))
  for i in range(n):
   a=(i+1)/n;s=a*a*(3-2*a);self.step_control(initial+s*(final-initial),phase)

 def hold(self,goal,duration,phase):
  for _ in range(int(round(duration/self.m.opt.timestep))):self.step_control(goal,phase)

 def run(self,target=1.66,micros=3):
  self.d.qpos[:]=self.initial;self.d.qvel[:]=0;self.d.time=0;mj.mj_forward(self.m,self.d)
  u0=self.d.qpos[self.aq].copy()
  self.hold(u0,.8,'baseline_settle')
  tgt=self.choose_safe_target(target)
  self.ramp(u0,tgt,1.5,'four_leg_rebalance')
  self.hold(tgt,.8,'rebalance_hold')
  safe=self.observation();stage_safe={k:float(x) for k,x in {'time':self.d.time,'margin':safe['margin'],'com_y':safe['com'][1],'maxq':max(abs(self.d.qpos[self.qads]))}.items()}
  self.sum_foot_xy=np.array([safe['position'][k][:2] for k in STANCE])
  self.safe_phase=True
  start_com_y=safe['com'][1]
  if micros==0: self.hold(tgt,1.95,'control_hold_baseline')
  for i in range(micros):
   obs=self.observation()
   if len(obs['support'])!=4 or obs['margin']<.012 or min(obs['force'][k][2] for k in STANCE)<.5 or max(abs(self.d.qpos[self.qads]))>1.7:
    self.events.append({'event':'micro_veto_measured_state','cycle':i+1,'margin_m':float(obs['margin'])});break
   lp=self.quasi_static_lp(obs)
   lp['cycle']=i+1;lp['time_s']=float(self.d.time);self.lp_checks.append(lp)
   if not lp['feasible']:
    self.events.append({'event':'micro_veto_lp','cycle':i+1,'reason':lp['reason']});break
   jb,jq=self.matrices(obs)
   q=self.d.qpos[self.qads].copy()
   result=micro_correct_stance(joint_positions_rad=q,lower_limits_rad=np.full(18,-1.7),upper_limits_rad=np.full(18,1.7),
     stance_jacobian_base=jb,stance_jacobian_joints=jq,desired_base_delta=np.array([0.,.00035,0.,0.,0.,0.]),
     max_joint_delta_rad=.02)
   if not result.approved_kinematic:
    self.events.append({'event':'micro_veto_kinematic','cycle':i+1,'reason':result.reason});break
   pre=tgt.copy();tgt=pre.copy()
   for k,qa in enumerate(self.qads):
    ai=int(np.flatnonzero(self.aq==qa)[0]);tgt[ai]+=result.joint_delta_rad[k]
   self.events.append({'event':'micro_command','cycle':i+1,'time_s':float(self.d.time),
      'max_joint_delta_rad':float(max(abs(np.array(result.joint_delta_rad)))),
      'predicted_foot_residual_m':result.max_contact_linear_residual_m})
   self.ramp(pre,tgt,.30,'micro_ramp_'+str(i+1))
   self.hold(tgt,.35,'micro_hold_'+str(i+1))
  end=self.observation()
  subset=[r for r in self.rows if r['phase'].startswith('micro')]
  r={
    'schema':'cad30_b_to_safepose_micro_dynamic_v1','mujoco':mj.__version__,
    'nominal_parameters':{'mass_kg':float(self.m.body_mass.sum()),'mu':self.mu,'k_p':7.,'k_d':.32,'torque_cap_leg_nm':float(self.cap[0]),'physics_dt_s':float(self.m.opt.timestep),'research_joint_abs_rad':1.7},
    'input_state':'pose B semi_tucked from archived dynamic t=9.3s (initially exceeds research bound ±1.7 rad)',
    'chosen_target_abs_rad':target,'rebalance_end':stage_safe,
    'min_contact_legs_all_rows':min(x['contact_legs'] for x in self.rows),
    'min_com_margin_mm_all_rows':min(x['margin_mm'] for x in self.rows),
    'min_Fz_all_rows_N':min(x['min_stance_Fz_N'] for x in self.rows),
    'peak_measured_actuator_nm':max(x['peak_actuator_tau_Nm'] for x in self.rows),
    'peak_unclipped_control_nm':self.max_unclipped,'actuator_saturated_steps':self.saturated,
    'max_joint_abs_at_end_rad':float(max(abs(self.d.qpos[self.qads]))),
    'micro_requested':micros,'micro_executed':sum(e['event']=='micro_command' for e in self.events),
    'micro_com_y_progress_mm':float((end['com'][1]-start_com_y)*1000),
    'micro_min_contacts':min(x['contact_legs'] for x in subset) if subset else None,
    'micro_min_com_margin_mm':min(x['margin_mm'] for x in subset) if subset else None,
    'micro_min_Fz_N':min(x['min_stance_Fz_N'] for x in subset) if subset else None,
    'micro_max_contact_xy_drift_mm':self.max_foot_drift_mm,
    'micro_max_joint_abs_rad':max(x['max_joint_abs_rad'] for x in subset) if subset else None,
    'lp_checks':self.lp_checks,'events':self.events,'every_physics_step':self.every_step,'bad_step_count':self.bad_step_count,
    'sampled_trace_50step':self.rows,
    'limits':['One mass/friction case; real servo/mass not calibrated','Only torso adjustments with all four feet grounded, NOT a gait','LP is gravity-only and uses one weighted contact point per tibia, not proof of dynamic safety','Actual stance-foot landmark slip may differ from contact centroids','Initialization from pose B violates conservative research IK bound until corrected','Commands are calculated from current contact and qpos, not a full-horizon MPC']
   }
  return r

def main():
 p=argparse.ArgumentParser()
 p.add_argument('--model',type=Path,required=True);p.add_argument('--replay-zip',type=Path,required=True)
 p.add_argument('--target',type=float,default=1.66);p.add_argument('--micro-count',type=int,default=3);p.add_argument('--mu',type=float,default=1.1)
 p.add_argument('--out',type=Path)
 a=p.parse_args();r=Experiment(a.model,a.replay_zip,mu=a.mu).run(a.target,a.micro_count)
 def safe_json(o):
  if isinstance(o,dict):return {k:safe_json(v) for k,v in o.items()}
  if isinstance(o,(list,tuple)):return [safe_json(x) for x in o]
  if isinstance(o,float) and not np.isfinite(o):return None
  return o
 r=safe_json(r)
 if a.out:
  a.out.parent.mkdir(parents=True,exist_ok=True)
  a.out.write_text(json.dumps(r,ensure_ascii=False,indent=2,allow_nan=False))
 print(json.dumps({k:v for k,v in r.items() if k not in ('sampled_trace_50step',)},ensure_ascii=False,indent=2,allow_nan=False))
 if r['micro_executed']!=a.micro_count or r['min_contact_legs_all_rows']<4 or r['actuator_saturated_steps']>0:raise SystemExit(2)

if __name__=='__main__':main()