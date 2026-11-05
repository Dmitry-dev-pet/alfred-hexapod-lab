"""G1 bounded numerical screening. NOT a dynamics or continuous gait proof.
Only tests sampled 3D IK + gravity wrench + planned (not measured) contacts.
"""
from pathlib import Path
import os
import numpy as np
import sys,json,time,collections,signal
ROOT=Path(__file__).parent
sys.path.insert(0,str(ROOT))
from check_period_ik import Check,WALK

class ScreeningStop(Exception):pass

class ThreeD:
 def __init__(self):
  self.c=Check();self.mm=self.c.m
 def test(self,candidate,correct=True,detail=False):
  mm=self.mm; cc=self.c; mm.set_pose(cc.original_q)
  target={k:np.asarray(candidate['start_foot_xy_m'][k]+[0.],float) for k in WALK}
  desiredCom0=np.asarray(candidate['COM0_xy_m']); root0=cc.original_root+(desiredCom0-cc.original_com)
  q=cc.original_q.copy()
  out={'pass':False,'order':candidate['order'],'adjustments_mm':candidate['adjustments_mm'], 'rank':candidate['rank_score_m'], 'reason':'', 'stage':'', 'nodes_checked':0,'max_toe_err_mm':0.,'max_joint':0.,'min_margin_mm':1e9,'max_tau':0.,'max_COM_error_mm':0.,'corrections':0}
  def put(root,com_target,grounded,phase):
   nonlocal q
   root=np.array(root,float)
   for k in range(3 if correct else 1):
    q,err=mm.ik(root,target,q,anchored=WALK);mm.set_pose(q)
    com=mm.com()[:2];e=np.asarray(com_target)-com
    if np.linalg.norm(e)<.001:break
    root+=np.clip(e,-.012,.012)
    out['corrections']+=1
   toed={k:np.linalg.norm(mm.toes()[k]-target[k])*1000 for k in WALK}
   out['max_toe_err_mm']=max(out['max_toe_err_mm'],max(toed.values()))
   out['max_joint']=max(out['max_joint'],float(np.max(np.abs(q[7:25]))))
   out['max_COM_error_mm']=max(out['max_COM_error_mm'],float(np.linalg.norm(com-np.asarray(com_target))*1000))
   margin=mm.margin(mm.toes(),grounded)
   out['min_margin_mm']=min(out['min_margin_mm'],margin)
   out['nodes_checked']+=1
   out['stage']=phase
   if max(toed.values())>float(os.environ.get('CAD30_IK_TOL_MM','0.8')):raise ScreeningStop('TOE_IK_OVER_TOLERANCE')
   if out['max_joint']>1.7+1e-6:raise ScreeningStop('JOINT_ABS_OVER_1P7RAD')
   if margin<9.5:raise ScreeningStop('STATIC_COM_MARGIN_BELOW_9P5MM')
   if float(np.linalg.norm(com-np.asarray(com_target))*1000)>8:raise ScreeningStop('ACTUAL_COM_DIFFERS_OVER_8MM')
   w=mm.wrench(grounded)
   if not w['pass']:raise ScreeningStop('STATIC_GRAVITY_FORCE_LP_INFEASIBLE')
   out['max_tau']=max(out['max_tau'],float(w['min_peak_tau_Nm']))
   if out['max_tau']>.7:raise ScreeningStop('STATIC_MOTOR_TAU_OVER_0P7NM')
   if detail:out.setdefault('trace',[]).append({'phase':phase,'com':mm.com().tolist(),'qpos':q.tolist(),'toe':{k:v.tolist() for k,v in mm.toes().items()},'margin':margin,'tau':w['min_peak_tau_Nm']})
   return root
  root=root0.copy();nominalCom=desiredCom0.copy()
  try:
   root=put(root,nominalCom,WALK,'initial_four_legs');q0=q.copy()
   for i,leg in enumerate(candidate['order']):
    newCom=np.array(candidate['COM_waypoints_xy_m'][i]); oldRoot=root.copy();oldCom=nominalCom.copy()
    for t in (.5,1.):
     ctarget=(1-t)*oldCom+t*newCom
     root=put((1-t)*oldRoot+t*root,ctarget,WALK,'prepare_'+leg)
    nominalCom=newCom.copy()
    oldToe=target[leg].copy();dest=oldToe.copy();dest[1]+=.020
    for t in (.25,.50,.75,1.):
     target[leg]=(1-t)*oldToe+t*dest
     target[leg][2]=.012*np.sin(np.pi*t)
     root=put(root,newCom,tuple(k for k in WALK if k!=leg),'swing_'+leg)
    target[leg]=dest
    root=put(root,newCom,WALK,'touchdown_'+leg)
   end=np.array(candidate['COM0_xy_m'])+[0,.020]
   for t in (.5,1.):
    ctarget=(1-t)*nominalCom+t*end
    root=put(root,ctarget,WALK,'closure')
   out['joint_closure_rad']=float(np.max(np.abs(q[7:25]-q0[7:25])))
   out['root_closure_mm']=float(np.linalg.norm(q[:2]-(q0[:2]+[0,.020]))*1000)
   if out['joint_closure_rad']>.03:raise ScreeningStop('INTERNAL_Q_PERIOD_MISMATCH')
   if out['root_closure_mm']>5:raise ScreeningStop('ROOT_TRANSLATION_PERIOD_MISMATCH')
   out['pass']=True;out['reason']='SAMPLED_IK_STATIC_LP_ONLY'
  except ScreeningStop as e:out['reason']=str(e)
  return out


def main():
 signal.signal(signal.SIGALRM, lambda signum, frame: (_ for _ in ()).throw(TimeoutError("candidate >2s")))
 rows=json.load(open(ROOT/'search_2d_results.json'))['all_candidates'];avail={tuple(x['adjustments_mm'].values()) for x in json.load(open(ROOT/'search_stance_results.json'))['results'] if x['start_pass']}
 by=collections.defaultdict(list)
 for cand in rows:
  key=tuple(cand['adjustments_mm'].values())
  if key in avail:by[key].append(cand)
 selected=[c for cands in by.values() for c in cands[:2]]
 solver=ThreeD();start=time.time();out=[]
 for i,c in enumerate(selected):
  try:
   signal.alarm(2)
   d=solver.test(c,correct=True,detail=False)
  except Exception as e:d={'pass':False,'reason':'exception_or_timeout:'+str(e)[:60],'stage':'unknown','order':c['order'],'adjustments_mm':c['adjustments_mm']}
  finally:signal.alarm(0)
  out.append(d)
  if (i+1)%24==0:
   print('SCREEN',i+1,'/',len(selected),'PASS',sum(x['pass'] for x in out),'elapsed',round(time.time()-start,1),flush=True)
   (ROOT/'periodic_3d_partial.json').write_text(json.dumps({'checked':i+1,'results':out},indent=2))
 from collections import Counter
 result={'schema':'CAD30-periodic-bounded-IK-LP-v2','scope':'48 four-leg reachable starting footprints x 2 best global order plans, sampled max 27 states, 3 corrective COM iterations, model joint <=1.7, static LP torque <=0.7, COM margin>=9.5; not dynamically verified','count':len(out),'passing':sum(x['pass'] for x in out),'reasons':Counter(x['reason'] for x in out),'states':out,'elapsed_s':time.time()-start}
 (ROOT/'periodic_3d_sweep.json').write_text(json.dumps(result,indent=2,ensure_ascii=False))
 print('FINAL',json.dumps({'checked':len(out),'pass':result['passing'],'reasons':result['reasons'],'elapsed':result['elapsed_s']},indent=2))
if __name__=='__main__':main()