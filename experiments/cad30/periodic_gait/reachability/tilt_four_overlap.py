import os
import json,sys,time,numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
ROOT=Path(os.environ.get('CAD30_RESEARCH_ROOT','/mnt/data/periodic_work'));sys.path[:0]=[str(ROOT/'math'),str(ROOT/'cad')]
from math_first_four_crawl_v1 import ModelStudy,WALK
Q0=np.asarray(json.load(open(ROOT/'physical/pupper_four_bl20_simple.json'))['final_qpos']); XML=ROOT/'cad/source/mjcf/scene_local30_nominal.xml'
model=ModelStudy(XML,Q0);model.set_pose(Q0);feet=model.toes();base=Rotation.from_quat(Q0[[4,5,6,3]])
rows=json.load(open(str(Path(__file__).resolve().parent/'tilt_sensitivity.json')))['records'];surv=[x for x in rows if x['screen_pass']];out=[];t=time.time()
for a in surv:
 dx,dy,dz=a['delta_mm'];roll,pitch=a['roll_deg'],a['pitch_deg'];q=Q0.copy();q[2]+=dz/1000
 quat=(Rotation.from_euler('xy',[roll,pitch],degrees=True)*base).as_quat();q[3:7]=quat[[3,0,1,2]]
 qq,errors=model.ik(Q0[:2]+np.array([dx,dy])/1000,feet,q,anchored=WALK);model.set_pose(qq)
 er=max(errors.values());max_joint=np.max(abs(qq[7:25]));margin=model.margin(model.toes(),WALK)
 w=model.wrench(WALK) if er<.8 and margin>=10 else {'pass':False}
 ok=bool(er<=.8 and max_joint<=1.7 and margin>=10 and w['pass'] and w['min_peak_tau_Nm']<=.7)
 out.append({**a,'four_toe_error_mm':round(float(er),3),'four_jointmax_rad':round(float(max_joint),4),'four_pass':ok})
for leg in ('ml','mr'):
 subset=[x for x in out if x['leg']==leg];print(leg,'3-pass',len(subset),'4-pass',sum(x['four_pass'] for x in subset),'min4toeErr',min(x['four_toe_error_mm'] for x in subset)); print('best', min(subset,key=lambda x:x['four_toe_error_mm']))
p=Path(str(Path(__file__).resolve().parent/'tilt_four_overlap.json'));p.write_text(json.dumps({'schema':'CAD30-tilt3-vs-four-anchored-same-pose-v1','results':out},indent=2));print('wrote',p,'time',round(time.time()-t,2))
