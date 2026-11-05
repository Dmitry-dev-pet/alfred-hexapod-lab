import os
"""Bounded height+XY search for 3-support nominal CAD30 configurations.
No full swing, no continuous path, no exhaustive certificate.
"""
from pathlib import Path
import sys,json,time,hashlib,numpy as np
ROOT=Path(os.environ.get('CAD30_RESEARCH_ROOT','/mnt/data/periodic_work'));sys.path[:0]=[str(ROOT/'math'),str(ROOT/'cad')]
from math_first_four_crawl_v1 import ModelStudy,WALK
Q0=np.array(json.loads((ROOT/'physical/pupper_four_bl20_simple.json').read_text())['final_qpos'])
XML=ROOT/'cad/source/mjcf/scene_local30_nominal.xml'
OUT=Path(__file__).resolve().parent
m=ModelStudy(XML,Q0);m.set_pose(Q0);feet=m.toes();start=time.time();records=[]
for dz in [-30,-20,-10,0,10,20,30]:
 for leg in ('ml','mr'):
  anchored=[k for k in WALK if k!=leg]
  xs=[30,45,60,75,90,105] if leg=='ml' else [-105,-90,-75,-60,-45,-30]
  for dx in xs:
   for dy in [-60,-75,-90,-105,-120,-135]:
    seed=Q0.copy();seed[2]+=dz/1000
    q,errs=m.ik(Q0[:2]+[dx/1000,dy/1000],feet,seed,anchored=anchored)
    m.set_pose(q)
    e=max(errs.values());jmax=float(np.max(abs(q[7:25])));com=np.asarray(m.com()[:2]);mar=m.margin(m.toes(),anchored)
    ik=(e<=.8 and jmax<=1.7+1e-7)
    geo=mar>=10
    w=m.wrench(anchored) if ik and geo else {'pass':False}
    ok=bool(ik and geo and w['pass'] and w['min_peak_tau_Nm']<=.7)
    records.append({'leg':leg,'root_delta_mm':[dx,dy,dz],'com_xy_mm':np.round(com*1000,3).tolist(),'margin_mm':round(mar,3),'toe_error_mm':round(float(e),3),'qmax_rad':round(jmax,4),'ik':bool(ik),'geometry':bool(geo),'lp':bool(w['pass']),'peak_tau_nm':round(w.get('min_peak_tau_Nm',float('nan')),4) if w['pass'] else None,'pass':ok})
 print('dz',dz,'done',len(records),'npassML',sum(x['pass'] for x in records if x['leg']=='ml'), 'npassMR',sum(x['pass'] for x in records if x['leg']=='mr'),'s',round(time.time()-start,1),flush=True)
path=OUT/'height_sensitivity_3stance.json';path.write_text(json.dumps({'schema':'cad30-3-stances-height-xy-grid-v1','status':'BOUNDED_SAMPLE_NOT_EXHAUSTIVE','constraints':'fixed original body orientation, 3 toes anchored, no collision or continuity guarantee','records':records},indent=2)+'\n')
print('RESULT',path,'time',round(time.time()-start,1),flush=True)
