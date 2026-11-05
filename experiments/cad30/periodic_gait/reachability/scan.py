"""Bounded 3-stance COM reachability samples on actual FNK0052 CAD30.
Not an exhaustive reachability certificate, not proof of cycle or dynamic control.
"""
from __future__ import annotations
import os
import json,sys,hashlib,time,os
from pathlib import Path
import numpy as np
import mujoco as mj
ROOT=Path(os.environ.get('CAD30_RESEARCH_ROOT','/mnt/data/periodic_work'))
sys.path[:0]=[str(ROOT/'math'),str(ROOT/'cad')]
from math_first_four_crawl_v1 import ModelStudy, WALK
STATE=ROOT/'physical/pupper_four_bl20_simple.json'
XML=ROOT/'cad/source/mjcf/scene_local30_nominal.xml'
OUT=Path(__file__).resolve().parent
Q0=np.asarray(json.loads(STATE.read_text())['final_qpos'])

def one(lift,dx_mm,dy_mm,dz_mm,ori=None,margin_min=10.,toe_tol_mm=.8, include_lp=True):
 m=ModelStudy(XML,Q0)
 m.set_pose(Q0)
 feet=m.toes(); remain=[i for i in WALK if i!=lift]
 if ori is not None: raise NotImplementedError
 seed=Q0.copy(); seed[2]+=dz_mm/1000
 # IMPORTANT ik sets XY only, and inherits Z from q seed
 q,errs=m.ik(Q0[:2]+np.array([dx_mm,dy_mm])/1000,feet,seed,anchored=remain)
 m.set_pose(q)
 com=m.com()[:2]; mar=m.margin(m.toes(),remain)
 e=max(errs.values())
 jmax=float(np.max(np.abs(q[7:25])))
 ok_ik=e<=toe_tol_mm and jmax<=1.7+1e-7
 ok_geom=mar>=margin_min
 w=m.wrench(remain) if include_lp and ok_ik and ok_geom else {'pass':False}
 ok=ok_ik and ok_geom and w['pass'] and w['min_peak_tau_Nm']<=.7
 return {'lift':lift,'root_delta_mm':[dx_mm,dy_mm,dz_mm],'com_xy_mm':np.round(com*1000,3).tolist(),
         'margin_mm':round(mar,3),'toe_error_mm':round(e,4),'qmax_rad':round(jmax,5),'ik_pass':bool(ok_ik),
         'geometric_pass':bool(ok_geom),'static_lp_pass':bool(w['pass']),'tau_Nm':round(w['min_peak_tau_Nm'],5) if w['pass'] else None,'screen_pass':bool(ok)}

def run(dz=0,lifts=('ml','mr','bl','br'),step=15):
 xs=np.arange(-90,91,step);ys=np.arange(-135,46,step)
 results=[];start=time.time()
 for lift in lifts:
  # reuse model avoids recompiling 30 STL every grid point
  m=ModelStudy(XML,Q0);m.set_pose(Q0);feet=m.toes(); remain=[k for k in WALK if k!=lift]
  for dx in xs:
   for dy in ys:
    seed=Q0.copy(); seed[2]+=dz/1000
    q,errs=m.ik(Q0[:2]+np.array([dx,dy])/1000,feet,seed,anchored=remain)
    m.set_pose(q)
    com=m.com()[:2]; mar=m.margin(m.toes(),remain)
    e=max(errs.values());jmax=float(np.max(np.abs(q[7:25])))
    ok_ik=e<=.8 and jmax<=1.7+1e-7
    ok_geom=mar>=10
    w=m.wrench(remain) if ok_ik and ok_geom else {'pass':False}
    ok=ok_ik and ok_geom and w['pass'] and w['min_peak_tau_Nm']<=.7
    results.append({'lift':lift,'root_delta_mm':[float(dx),float(dy),dz],'com_xy_mm':np.round(com*1000,3).tolist(),
         'margin_mm':round(float(mar),3),'toe_error_mm':round(float(e),4),'qmax_rad':round(jmax,5),'ik_pass':bool(ok_ik),
         'geometric_pass':bool(ok_geom),'static_lp_pass':bool(w['pass']),'tau_Nm':round(w['min_peak_tau_Nm'],5) if w['pass'] else None,'screen_pass':bool(ok)})
  sub=[r for r in results if r['lift']==lift]
  print(lift, 'n',len(sub),'IK',sum(z['ik_pass'] for z in sub),'both IK & geometric',sum(z['ik_pass'] and z['geometric_pass'] for z in sub),'lp',sum(z['screen_pass'] for z in sub),'elapsed',round(time.time()-start,2),flush=True)
 data={'schema':'fnk0052-3-stance-COM-bounded-sample-v1','state_sha256':hashlib.sha256(STATE.read_bytes()).hexdigest(),'xml_sha256':hashlib.sha256(XML.read_bytes()).hexdigest(),
      'source_state':STATE.name,'source_note':'real recorded MuJoCo qpos after one BL swing, not a complete cycle',
      'constraints':{'model':'CAD30 original nominal','joint_abs_max_rad':1.7,'torque_max_Nm':0.7,'contact_margin_min_mm':10,'toe_error_max_mm':0.8,'mu':1.1,
                     'orientation':'fixed measured quaternion','root_z_delta_mm':dz,'foot_contact':'3 named toe-vertices FIXED in world, 4th leg pose unmodified; not real swing','grids':'root XY finite grid; no dynamics, no interval guarantees'},
      'sample_count':len(results),'results':results}
 path=OUT/f'scan_z_{dz:+d}mm.json';path.write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n')
 print('WROTE',path,flush=True)
 return data
if __name__=='__main__':run(dz=int(sys.argv[1]) if len(sys.argv)>1 else 0, lifts=tuple(sys.argv[2:]) if len(sys.argv)>2 else ('ml','mr','bl','br'))
