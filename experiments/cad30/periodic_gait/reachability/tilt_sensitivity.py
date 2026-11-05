import os
"""Bounded pitch/roll sampling of 3-stance reachable CAD30 configurations.
NO claim of certified continuous reachability or mechanical impossibility.
"""
import json,sys,time,numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
ROOT=Path(os.environ.get('CAD30_RESEARCH_ROOT','/mnt/data/periodic_work'));sys.path[:0]=[str(ROOT/'math'),str(ROOT/'cad')]
from math_first_four_crawl_v1 import ModelStudy,WALK
XML=ROOT/'cad/source/mjcf/scene_local30_nominal.xml';Q0=np.array(json.loads((ROOT/'physical/pupper_four_bl20_simple.json').read_text())['final_qpos']);
base=Rotation.from_quat(Q0[[4,5,6,3]])
model=ModelStudy(XML,Q0);model.set_pose(Q0);feet=model.toes();t=time.time();out=[]
for leg in ('ml','mr'):
 legs=[k for k in WALK if k!=leg]
 for roll,pitch in [(-8,0),(0,-8),(0,0),(0,8),(8,0)]:
  quat=(Rotation.from_euler('xy',[roll,pitch],degrees=True)*base).as_quat()
  for dz in [0,15,30]:
   for dx in ([15,30,45,60,75] if leg=='ml' else [-75,-60,-45,-30,-15]):
    for dy in [-60,-75,-90,-105,-120]:
     seed=Q0.copy();seed[2]+=dz/1000;seed[3:7]=quat[[3,0,1,2]]
     q,err=model.ik(Q0[:2]+np.array([dx,dy])/1000,feet,seed,anchored=legs)
     model.set_pose(q);toe_err=max(err.values());jointmax=float(np.max(abs(q[7:25])));mar=model.margin(model.toes(),legs)
     ik=toe_err<=.8 and jointmax<=1.7+1e-7;geo=mar>=10
     wrench=model.wrench(legs) if ik and geo else {'pass':False}
     ok=bool(ik and geo and wrench['pass'] and wrench['min_peak_tau_Nm']<=.7)
     out.append({'leg':leg,'roll_deg':roll,'pitch_deg':pitch,'delta_mm':[dx,dy,dz],'com_xy_mm':np.round(model.com()[:2]*1000,2).tolist(),
      'margin_mm':round(float(mar),3),'toe_err_mm':round(float(toe_err),3),'jointmax_rad':round(jointmax,4),'static_tau_nm':round(float(wrench['min_peak_tau_Nm']),4) if wrench['pass'] else None,'screen_pass':ok})
  print(leg,roll,pitch,'pass',sum(z['screen_pass'] for z in out if z['leg']==leg),'elapsed',round(time.time()-t,1),flush=True)
p=Path(str(Path(__file__).resolve().parent/'tilt_sensitivity.json'));p.write_text(json.dumps({'schema':'cad30-3-stance-tilt-sweep-v1','sampling':'fixed captured contact toe points, roll/pitch perturbations world-axis ±8deg, root X/Y/Z bounded, grid only, no continuous/dynamic guarantee','records':out},indent=2)+'\n');print('saved',p,round(time.time()-t,1))
