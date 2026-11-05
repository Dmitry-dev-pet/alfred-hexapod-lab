import os
from pathlib import Path
import json,sys,numpy as np
R=Path(os.environ.get('CAD30_RESEARCH_ROOT','/mnt/data/periodic_work'));sys.path[:0]=[str(R/'math'),str(R/'cad')]
from math_first_four_crawl_v1 import ModelStudy,WALK
q0=np.array(json.load(open(R/'physical/pupper_four_bl20_simple.json'))['final_qpos']);xml=R/'cad/source/mjcf/scene_local30_nominal.xml';m=ModelStudy(xml,q0);m.set_pose(q0);feet=m.toes();xs=json.load(open(str(Path(__file__).resolve().parent/'height_sensitivity_3stance.json')))['records'];out=[]
for a in xs:
 if not a['pass']:continue
 dx,dy,dz=a['root_delta_mm']; seed=q0.copy();seed[2]+=dz/1000
 q,errs=m.ik(q0[:2]+np.array([dx,dy])/1000,feet,seed,anchored=WALK);m.set_pose(q)
 e=max(errs.values());jmax=float(np.max(np.abs(q[7:25])));mar=m.margin(m.toes(),WALK)
 w=m.wrench(WALK) if e<.8 and mar>=10 else {'pass':False}
 feasible=bool(e<=.8 and jmax<=1.7 and mar>=10 and w['pass'] and w['min_peak_tau_Nm']<=.7)
 out.append({**a,'four_anchor_toe_error_mm':round(e,3),'four_anchor_joint_rad':round(jmax,4),'four_anchor_margin_mm':round(mar,3),'four_anchor_pass':feasible})
print('n',len(out),'four pass',sum(x['four_anchor_pass'] for x in out))
for leg in ('ml','mr'):
 sub=[x for x in out if x['leg']==leg];print(leg,'n',len(sub),'min 4-leg toe residual',round(min(x['four_anchor_toe_error_mm'] for x in sub),3));print('best',min(sub,key=lambda x:x['four_anchor_toe_error_mm']))
p=Path(str(Path(__file__).resolve().parent/'compare_three_vs_four.json'));p.write_text(json.dumps({'schema':'3-stance-vs-4-stance-xy-height-matched-samples-v1','results':out},indent=2))
