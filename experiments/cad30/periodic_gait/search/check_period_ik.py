import json,sys,time,os
from pathlib import Path
import numpy as np
import mujoco as mj
ROOT=Path(__file__).parent
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT.parent));
CAD_ROOT=Path(os.environ.get('CAD30_ROOT',str(ROOT/'cad'))).resolve()
PHYSICAL_ROOT=Path(os.environ.get('CAD30_BASELINE_DIR',str(ROOT/'physical'))).resolve()
sys.path.insert(0,str(CAD_ROOT))
from math_first_four_crawl_v1 import ModelStudy,WALK,ALL,JOINT_CAP_RAD
from hexlab.stand_geometry_v04 import load_legs,tip_world
from search_global_geometry import support_ineq

class Check:
 def __init__(self):
  self.st=json.load(open(PHYSICAL_ROOT/'front_pair_shift35_p0.json'))
  self.m=ModelStudy(CAD_ROOT/'source/mjcf/scene_local30_nominal.xml',self.st['final_qpos'])
  self.original_q=np.array(self.st['final_qpos'])
  self.original_root=self.original_q[:2].copy()
  self.original_com=self.m.com()[:2].copy()
  self.original_feet=self.m.toes()
 def calculate(self,candidate,samples=2,record=False):
  mm=self.m
  mm.set_pose(self.original_q)
  root0=self.original_root
  f={k:self.original_feet[k].copy() for k in WALK}
  target={k:np.array(candidate['start_foot_xy_m'][k]+[0.]) for k in WALK}
  order=candidate['order']; delta=.020
  dCom=np.asarray(candidate['COM0_xy_m'])-self.original_com
  root_start=root0+dCom
  q,err=mm.ik(root_start,target,self.original_q,anchored=WALK)
  mm.set_pose(q)
  worst={'max_err_mm':max(err.values()),'max_abs_joint':float(np.max(np.abs(q[7:25]))),'min_margin_mm':float('inf'),'max_tau':0.,'max_com_target_error_mm':0.,'failed':[]}
  def inspect(phase,legs,assertCOM=None):
   nonlocal worst
   actual=mm.com()[:2]
   if assertCOM is not None:
    worst['max_com_target_error_mm']=max(worst['max_com_target_error_mm'],float(np.linalg.norm(actual-np.asarray(assertCOM)))*1000)
   co=mm.toes();toe_err=max(np.linalg.norm(co[k]-target[k])*1000 for k in WALK)
   worst['max_err_mm']=max(worst['max_err_mm'],float(toe_err))
   worst['max_abs_joint']=max(worst['max_abs_joint'],float(np.max(np.abs(q[7:25]))))
   margin=mm.margin(co,legs)
   worst['min_margin_mm']=min(worst['min_margin_mm'],margin)
   if margin<9.0:worst['failed'].append((phase,'margin',round(margin,2)))
   w=mm.wrench(legs)
   if not w['pass']:worst['failed'].append((phase,'LP',w.get('reason','unknown')[:90]))
   else:worst['max_tau']=max(worst['max_tau'],w['min_peak_tau_Nm'])
   if toe_err>1:worst['failed'].append((phase,'toe_error',round(toe_err,2)))
   if worst['max_abs_joint']>1.7:worst['failed'].append((phase,'joint limit',round(worst['max_abs_joint'],3)))
   if record:worst.setdefault('trace',[]).append({'phase':phase,'root':q[:2].tolist(),'com':mm.com().tolist(),'toes':{k:v.tolist() for k,v in co.items()},'margin_mm':margin,'LP':w,'qpos':q.tolist()})
  q0=q.copy();inspect('start',WALK,candidate['COM0_xy_m'])
  last_root=root_start.copy()
  start_target={k:v.copy() for k,v in target.items()}
  for idx,leg in enumerate(order):
   next_com=np.asarray(candidate['COM_waypoints_xy_m'][idx]);rootgoal=root0+(next_com-self.original_com)
   # move root in all-stanced support before swing
   for t in np.linspace(0,1,samples+1)[1:]:
    root=(1-t)*last_root+t*rootgoal
    q,err=mm.ik(root,target,q,anchored=WALK);mm.set_pose(q);inspect('pre_'+leg,WALK)
   last_root=rootgoal
   start=target[leg].copy();end=start.copy();end[1]+=delta
   for t in np.linspace(0,1,samples+2)[1:]:
    target[leg]=(1-t)*start+t*end
    target[leg][2]=np.sin(np.pi*t)*.012
    q,err=mm.ik(rootgoal,target,q,anchored=WALK);mm.set_pose(q)
    inspect('swing_'+leg,tuple(k for k in WALK if k!=leg),next_com)
   target[leg]=end
   inspect('touchdown_'+leg,WALK)
  # final translation and closing q
  endcom=np.asarray(candidate['COM0_xy_m'])+[0,delta]
  targetgoal=root_start+[0,delta]
  for t in np.linspace(0,1,samples+1)[1:]:
   root=(1-t)*last_root+t*targetgoal
   q,err=mm.ik(root,target,q,anchored=WALK);mm.set_pose(q);inspect('closure',WALK,endcom)
  worst['terminal_q_linf_rad']=float(np.max(np.abs(q[7:25]-q0[7:25])))
  worst['terminal_root_xy_error_mm']=float(np.linalg.norm(q[:2]-(q0[:2]+[0,delta]))*1000)
  worst['terminal_foot_translation_error_mm']=max(float(np.linalg.norm(target[k]-(start_target[k]+[0,delta,0]))*1000) for k in WALK)
  if worst['terminal_q_linf_rad']>0.03:worst['failed'].append(('closure','joint mismatch',worst['terminal_q_linf_rad']))
  worst['pass']=(not worst['failed'] and worst['max_com_target_error_mm']<=5 and worst['max_err_mm']<=1 and worst['max_tau']<0.7)
  worst.update({'order':order,'adjustments_mm':candidate['adjustments_mm'],'rank_score_m':candidate['rank_score_m'],'root_start_m':root_start.tolist(),'start_com_m':candidate['COM0_xy_m'],'period_delta_mm':20.0,'notes':'Original CAD 3D kinematics and gravity wrench LP at sampled points; no physical transitions, no motor commands.'})
  return worst

if __name__=='__main__':
 r=json.load(open(ROOT/'search_2d_results.json'))
 ch=Check();t=time.time();results=[]
 for idx,c in enumerate(r['best_candidates'][:40]):
  try:rr=ch.calculate(c,samples=1)
  except Exception as exc:rr={'pass':False,'failed':[{'exception':repr(exc)}],'order':c['order'],'adjustments_mm':c['adjustments_mm']}
  results.append(rr)
  if (idx+1)%5==0:
   print('CHECK',idx+1,'elapsed',round(time.time()-t,2),'passes',sum(x.get('pass',False) for x in results),'sample_last',rr.get('failed',[])[:2],flush=True)
 out={'schema':'cad30-sampled-global-period-IKLP-screen-v1','model':'Freenove CAD30 original 30 meshes, MuJoCo 3.14, sampled IK+gravity LP', 'tested_ik':len(results),'passing_ik_static':sum(x.get('pass',False) for x in results),'results':results,'time_s':round(time.time()-t,2)}
 (ROOT/'search_ik_results.json').write_text(json.dumps(out,indent=2,ensure_ascii=False))
 print(json.dumps({'tested':out['tested_ik'],'passed':out['passing_ik_static'],'elapsed':out['time_s'],'best':sorted(results,key=lambda x:len(x.get('failed',[])))[:2]},indent=2)[:15000])