import os
"""Analytical rigid-link necessary reach for maintaining fourth original toe.
A necessary, not sufficient, full-contact condition.
"""
import sys,json,numpy as np, mujoco as mj
from pathlib import Path
from scipy.spatial.transform import Rotation
ROOT=Path(os.environ.get('CAD30_RESEARCH_ROOT','/mnt/data/periodic_work'));sys.path[:0]=[str(ROOT/'math'),str(ROOT/'cad')]
from math_first_four_crawl_v1 import ModelStudy,WALK
q0=np.asarray(json.load(open(ROOT/'physical/pupper_four_bl20_simple.json'))['final_qpos']);m=ModelStudy(ROOT/'cad/source/mjcf/scene_local30_nominal.xml',q0);m.set_pose(q0)
feet=m.toes();base=Rotation.from_quat(q0[[4,5,6,3]]);orig={}
for k in WALK:
 l=m.legs[k]; p=m.data.xanchor; j=l['jids'];
 orig[k]={'max_link_mm':(np.linalg.norm(p[j[0]]-p[j[1]])+np.linalg.norm(p[j[1]]-p[j[2]])+np.linalg.norm(p[j[2]]-feet[k]))*1000}
rows=json.load(open(str(Path(__file__).resolve().parent/'tilt_four_overlap.json')))['results'];out=[]
for a in rows:
 dx,dy,dz=a['delta_mm']; quat=(Rotation.from_euler('xy',[a['roll_deg'],a['pitch_deg']],degrees=True)*base).as_quat()
 q=q0.copy();q[:3]+=[dx/1000,dy/1000,dz/1000];q[3:7]=quat[[3,0,1,2]];m.set_pose(q)
 leg=a['leg'];hip=np.asarray(m.data.xanchor[m.legs[leg]['jids'][0]]); need=float(np.linalg.norm(hip-feet[leg])*1000)
 limit=orig[leg]['max_link_mm'];out.append({**a,'lifted_leg_hip_to_original_toe_mm':round(need,3),'link_length_upper_bound_mm':round(limit,3),'rigid_link_bound_violated':bool(need>limit+1e-7)})
for leg in ('ml','mr'):
 rr=[x for x in out if x['leg']==leg];print(leg,'total',len(rr),'strictly impossible to have 4 tips planted at exact root pose',sum(x['rigid_link_bound_violated'] for x in rr),'best',min(rr,key=lambda x:x['lifted_leg_hip_to_original_toe_mm']))
p=Path(str(Path(__file__).resolve().parent/'geometric_fourth_tip_bounds.json'));p.write_text(json.dumps({'schema':'cad30-rigid-link-necessary-bound-v1','explanation':'4th swing toe cannot stay planted at same world contact when its distance from hip exceeds sum of rigid link lengths. No statement about path feasibility or global impossible gait.','links':orig,'tested':len(out),'records':out},indent=2)+'\n');print('wrote',p)
