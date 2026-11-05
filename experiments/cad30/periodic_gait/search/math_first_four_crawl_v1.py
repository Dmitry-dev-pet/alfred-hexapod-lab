"""Math-only candidate four-foot crawl for Freenove FNK0052 CAD30.

Observed starting state is six loaded feet in Pupper physical baseline PR #40.
Uses actual CAD30 kinematics and mesh, sampled IK + point-foot gravity-wrench LP.
No mj_step, no contact dynamics, no actuation claims, no robot commands.

Run with restored private CAD source and two mesh-free physical trace JSON files:
  python math_first_four_crawl_v1.py --cad-root ... --six-state ... --front-reference ... --out math_four_crawl_result.json
"""
from __future__ import annotations
import argparse, json, math, sys
from pathlib import Path
from collections import Counter
import numpy as np
import mujoco as mj
from scipy.optimize import least_squares,linprog
from scipy.spatial import ConvexHull

WALK=('ml','mr','bl','br')
ALL=(*WALK,'fl','fr')
JOINT_CAP_RAD=1.7
TORQUE_CAP_NM=.7
FRICTION=1.1
MIN_CONTACT_N=.2
MARGIN_MM=5.0
BODY_PREP_BACK_M=.03
STEP_FORWARD_M=.02
SWING_HEIGHT_M=.012

class ModelStudy:
 def __init__(self,model_path,initial_qpos):
  # import CAD-based tip definition from original user's locally restored source
  from hexlab.stand_geometry_v04 import load_legs,tip_world,geom_mesh_world_vertices
  self.model=mj.MjModel.from_xml_path(str(model_path));self.data=mj.MjData(self.model)
  self.data.qpos[:]=initial_qpos;mj.mj_forward(self.model,self.data)
  self.legs={l['suffix']:l for l in load_legs(self.model,self.data,mj)}
  self.tip_world=tip_world;self.geom_mesh_world_vertices=geom_mesh_world_vertices
  self.original_q=np.array(initial_qpos,float)
  self.original_feet=self.toes()
 def toes(self):return {k:self.tip_world(self.model,self.data,l).copy() for k,l in self.legs.items()}
 def set_pose(self,q):
  self.data.qpos[:]=q;self.data.qvel[:]=0.;self.data.qacc[:]=0.;mj.mj_forward(self.model,self.data)
 def com(self):
  m=self.model;d=self.data;return (np.asarray(d.xipos)*np.asarray(m.body_mass)[:,None]).sum(axis=0)/m.body_mass.sum()
 def ik(self,root_xy,targets,seed,anchored=ALL):
  """4-/6-foot IK with root rotation/height frozen, joint bounds ±1.7rad."""
  m=self.model;d=self.data;d.qpos[:]=seed;d.qpos[:2]=root_xy;mj.mj_forward(m,d)
  errors={}
  for k in anchored:
   leg=self.legs[k];j=leg['qadrs'];qidx=leg['dadrs'];target=targets[k];start=np.clip(d.qpos[j].copy(),-1.7+1e-6,1.7-1e-6)
   def error(x):
    d.qpos[j]=x;mj.mj_forward(m,d)
    return np.r_[self.tip_world(m,d,leg)-target,.000025*(x-start)]
   def jac(x):
    d.qpos[j]=x;mj.mj_forward(m,d)
    point=self.tip_world(m,d,leg);pos=np.zeros((3,m.nv));rot=np.zeros((3,m.nv))
    mj.mj_jac(m,d,pos,rot,point,leg['body'])
    return np.vstack((pos[:,qidx],.000025*np.eye(3)))
   fit=least_squares(error,start,jac=jac,method='trf',bounds=(-np.ones(3)*JOINT_CAP_RAD,np.ones(3)*JOINT_CAP_RAD),max_nfev=80,ftol=1e-9,xtol=1e-9,gtol=1e-9)
   d.qpos[j]=fit.x;mj.mj_forward(m,d)
   errors[k]=float(np.linalg.norm(self.tip_world(m,d,leg)-target)*1000)
  return d.qpos.copy(),errors
 def margin(self,feet,grounded):
  pts=np.array([feet[k][:2] for k in grounded]);eq=ConvexHull(pts).equations
  return float(np.min(-(eq[:,:2]@self.com()[:2]+eq[:,2]))*1000)
 def wrench(self,grounded,mu=FRICTION,tau_cap=TORQUE_CAP_NM):
  """Minimize peak *model* motor torque with 3D point contacts and conservative friction diamond."""
  m=self.model;d=self.data;nv=m.nv;n=len(grounded)
  J=np.zeros((nv,3*n))
  for i,k in enumerate(grounded):
   leg=self.legs[k];point=self.tip_world(m,d,leg)
   p=np.zeros((3,nv));r=np.zeros((3,nv));mj.mj_jac(m,d,p,r,point,leg['body']);J[:,i*3:i*3+3]=p.T
  addr=m.jnt_dofadr[m.actuator_trnid[:,0]];gear=np.asarray(m.actuator_gear[:,0]);nu=len(addr)
  S=np.zeros((nv,nu));S[addr,np.arange(nu)]=gear
  Aeq=np.column_stack([J,S,np.zeros((nv,1))]);beq=d.qfrc_bias.copy()
  bounds=[]
  for _ in grounded:bounds.extend([(None,None),(None,None),(MIN_CONTACT_N,None)])
  bounds.extend([(None,None)]*nu);bounds.append((0,tau_cap))
  aub=[]
  for i in range(n):
   for sx in (-1.,1.):
    for sy in (-1.,1.):
     row=np.zeros(len(bounds));row[3*i]=sx;row[3*i+1]=sy;row[3*i+2]=-mu;aub.append(row)
  for j in range(nu):
   for sign in (-1.,1.):
    row=np.zeros(len(bounds));row[3*n+j]=sign;row[-1]=-1.;aub.append(row)
  obj=np.zeros(len(bounds));obj[-1]=1.
  fit=linprog(obj,A_ub=np.asarray(aub),b_ub=np.zeros(len(aub)),A_eq=Aeq,b_eq=beq,bounds=bounds,method='highs')
  if not fit.success:return {'pass':False,'reason':fit.message}
  forces=fit.x[:3*n].reshape(n,3)
  return {'pass':True,'min_peak_tau_Nm':float(fit.x[-1]),'min_Fz_N':float(forces[:,2].min()),'eq_max_residual_N':float(np.abs(Aeq@fit.x-beq).max())}
 def mesh_lowest_mm(self,leg):
  return float(np.min(self.geom_mesh_world_vertices(self.model,self.data,self.legs[leg]['geom'])[:,2])*1000)


def candidate_targets(feet):
 targets={k:v.copy() for k,v in feet.items()}
 for k in WALK:targets[k][2]=0.0
 for k in ('ml','mr'):targets[k][0]+=.050*(1 if k=='ml' else -1)
 for k in ('bl','br'):
  targets[k][0]+=.020*(-1 if k=='bl' else 1)
  targets[k][1]+=.080
 return targets


def run(cad_root,six_state,front_reference,mu=FRICTION,tau_cap=TORQUE_CAP_NM):
 base=json.loads(Path(six_state).read_text());front=json.loads(Path(front_reference).read_text())
 m=ModelStudy(Path(cad_root)/'source/mjcf/scene_local30_nominal.xml',base['final_qpos'])
 original_q=np.array(base['final_qpos']);root=original_q[:2].copy();feet=m.original_feet
 for k in WALK:feet[k][2]=0.0
 goals=candidate_targets(feet)
 q=original_q.copy()
 nodes=[];errors=[]
 def record(phase,grounded,targets=None,swing=None):
  m.set_pose(q)
  gr=tuple(grounded)
  toe_error=0.0
  if targets is not None:toe_error=max(np.linalg.norm(m.tip_world(m.model,m.data,m.legs[k])-targets[k])*1000 for k in targets)
  wrench=m.wrench(gr,mu=mu,tau_cap=tau_cap)
  margin=m.margin(targets if targets is not None else feet,gr)
  d={'phase':phase,'grounded':list(gr),'COM_xyz_m':m.com().tolist(),'root_xy_m':m.data.qpos[:2].tolist(),'qpos':q.tolist(),
   'margin_mm':margin,'max_toe_error_mm':float(toe_error),'max_joint_abs_rad':float(np.max(np.abs(q[7:25]))),
   'min_peak_tau_Nm':wrench.get('min_peak_tau_Nm'),'static_lp_pass':wrench['pass']}
  if swing is not None:d['swing']=swing;d['swing_tibia_lowest_mm']=m.mesh_lowest_mm(swing)
  nodes.append(d)
  if margin<MARGIN_MM-.00001 or toe_error>.7 or d['max_joint_abs_rad']>JOINT_CAP_RAD or not wrench['pass']:
   errors.append({'phase':phase,'index':len(nodes)-1,'margin_mm':margin,'toe_error_mm':toe_error,'qmax':d['max_joint_abs_rad'],'LP':wrench['pass']})
  return d
 def ik(root_target,targets,grounded,phase,swing=None):
  nonlocal q
  q,errs=m.ik(root_target,targets,q,anchored=tuple(targets.keys()))
  return record(phase,grounded,targets,swing=swing)
 # Measured six-contact physical baseline, with the measured toe+0.3/0.8mm offsets represented via target at z=0.
 ik(root,feet,ALL,'six_stance_observed')
 prep_order=('bl','br','ml','mr')
 for leg in prep_order:
  begin=feet[leg].copy();end=goals[leg].copy()
  grounded=tuple(k for k in ALL if k!=leg)
  for i,t in enumerate(np.linspace(0,1,17)):
   temp={k:v.copy() for k,v in feet.items()}
   temp[leg]=(1-t)*begin+t*end;temp[leg][2]+=math.sin(math.pi*t)*SWING_HEIGHT_M
   ik(root,temp,grounded,'six_support_reposition_'+leg,swing=leg if 0<t<1 else None)
  feet[leg]=end
  ik(root,feet,ALL,'six_support_touchdown_'+leg)
 # raise front pair while four walking legs continue to hold; front q trajectory interpolated to physically observed lifted-pair q.
 initial_front={k:q[m.legs[k]['qadrs']].copy() for k in ('fl','fr')}
 final_front={k:np.array(front['final_qpos'])[m.legs[k]['qadrs']].copy() for k in ('fl','fr')}
 for t in np.linspace(0,1,17):
  for k in ('fl','fr'):
   q[m.legs[k]['qadrs']]=(1-t)*initial_front[k]+t*final_front[k]
  # keep stance goal q from preceding phase; intentional pure angle interpolation for suspended claws
  record('front_pair_raise',WALK,{k:feet[k] for k in WALK},swing=None)
 # move root backward 30mm on four stance legs
 start_xy=root.copy();base_xy=root.copy()+[0.,-BODY_PREP_BACK_M]
 for t in np.linspace(0,1,17):
  ik((1-t)*start_xy+t*base_xy,{k:feet[k] for k in WALK},WALK,'four_stance_rebalance')
 # four-step cycle in world frame; root targets are offsets from original recorded six-contact base
 root_stances=[('ml',np.array([root[0]+.010,root[1]-.065])),('mr',np.array([root[0]+.010,root[1]-.065])),
               ('bl',np.array([root[0]+.015,root[1]-.045])),('br',np.array([root[0]-.020,root[1]-.02352]))]
 previous_xy=base_xy.copy()
 for leg,next_xy in root_stances:
  for t in np.linspace(0,1,11):
   ik((1-t)*previous_xy+t*next_xy,{k:feet[k] for k in WALK},WALK,'before_'+leg+'_body_shift')
  old=feet[leg].copy();new=old.copy();new[1]+=STEP_FORWARD_M
  for t in np.linspace(0,1,13):
   temp={k:feet[k].copy() for k in WALK};temp[leg]=(1-t)*old+t*new;temp[leg][2]+=math.sin(math.pi*t)*SWING_HEIGHT_M
   grounded=tuple(k for k in WALK if k!=leg)
   ik(next_xy,temp,grounded,'swing_'+leg,swing=leg if 0<t<1 else None)
  feet[leg]=new
  ik(next_xy,{k:feet[k] for k in WALK},WALK,'touchdown_'+leg)
  previous_xy=next_xy
 # terminal body shift into same relative posture, translated +20mm world Y
 close_xy=base_xy+[0,STEP_FORWARD_M]
 for t in np.linspace(0,1,11):
  ik((1-t)*previous_xy+t*close_xy,{k:feet[k] for k in WALK},WALK,'closure')
 # Recompute terminal step from mathematical start with same root pose and foot frame translation
 result={'schema':'cad30-math-first-four-crawl-v1','cad_provenance':'private Freenove CAD30 (30 original meshes), not embedded',
  'source_six':'PR #40 actual six-loaded Pupper stance after 35mm physical prebalance',
  'input_state_source_files':[Path(six_state).name,Path(front_reference).name],
  'method':'sampled 3D CAD toe IK + COM support polygon + conservative 3D point-contact static LP (minimize peak actuator torque)',
  'mu':mu,'assumed_joint_cap_rad':JOINT_CAP_RAD,'assumed_motor_tau_cap_Nm':tau_cap,
  'candidate_footprint_mm':{'middle_inward_each':50,'rear_forward_each':80,'rear_outward_each':20},
  'root_rebalance_mm':-30,'cycle_order':['ml','mr','bl','br'],'each_foot_world_forward_mm':20,
  'cycle_terminal_root_plus_world_Y_mm':20,'cycle_repeatable_geometry_only':True,
  'number_of_sampled_nodes':len(nodes),
  'min_sampled_support_margin_mm':min(d['margin_mm'] for d in nodes),
  'max_sampled_toe_error_mm':max(d['max_toe_error_mm'] for d in nodes),
  'max_sampled_leg_joint_abs_rad':max(d['max_joint_abs_rad'] for d in nodes),
  'max_sampled_required_static_tau_Nm':max(d['min_peak_tau_Nm'] or 0 for d in nodes),
  'min_swing_mesh_ground_clearance_mm':min(d['swing_tibia_lowest_mm'] for d in nodes if 'swing_tibia_lowest_mm' in d),
  'max_body_net_Y_from_six_prep_mm':float((close_xy[1]-start_xy[1])*1000),
  'recorded_original_six_root_Y_m':float(start_xy[1]),
  'final_root_Y_m':float(close_xy[1]),
  'errors':errors, 'status':'SAMPLED_GEOMETRIC_STATIC_CANDIDATE' if not errors else 'FAILED_MATH_FEASIBILITY',
  'caveats':['No dynamic mj_step. All root/toe targets are virtual joint IK poses; no real contact load measurement.',
    'Preparatory feet must be physically moved, not teleported; continuous sampled states do not guarantee transitions.',
    'Point contacts are toe geometry assumptions; actual model contact points and self-collisions require dynamic check.',
    'Research friction/torque/joint limits not hardware calibrated; small servo/contact uncertainty may invalidate.',
    'No net positive displacement from original six-grounded start within one math cycle; 2+ further cycles needed.']}
 return result,nodes

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--cad-root',required=True);p.add_argument('--six-state',required=True);p.add_argument('--front-reference',required=True);p.add_argument('--out',required=True);p.add_argument('--trace-out',default='')
 a=p.parse_args();out,nodes=run(a.cad_root,a.six_state,a.front_reference)
 Path(a.out).write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n')
 if a.trace_out:Path(a.trace_out).write_text(json.dumps({'summary':out,'nodes':nodes},ensure_ascii=False,indent=2)+'\n')
 print(json.dumps({k:v for k,v in out.items() if k!='caveats'},ensure_ascii=False,indent=2))