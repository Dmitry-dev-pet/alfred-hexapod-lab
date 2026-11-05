import os
import sys,json,time,numpy as np
from pathlib import Path
ROOT=Path(os.environ.get('CAD30_RESEARCH_ROOT','/mnt/data/periodic_work'));sys.path[:0]=[str(ROOT/'math'),str(ROOT/'cad')]
from math_first_four_crawl_v1 import ModelStudy,WALK
import mujoco as mj
state=json.load(open(ROOT/'physical/pupper_four_bl20_simple.json'));q0=np.asarray(state['final_qpos']);xml=ROOT/'cad/source/mjcf/scene_local30_nominal.xml'
m=ModelStudy(xml,q0);m.set_pose(q0);feet=m.toes();res=[];t=time.time()
for dx in np.arange(-90,91,15):
 for dy in np.arange(-135,46,15):
  q,errors=m.ik(q0[:2]+[dx/1000,dy/1000],feet,q0,anchored=WALK)
  m.set_pose(q)
  com=np.asarray(m.com()[:2]); err=max(errors.values());qmax=float(np.max(np.abs(q[7:25]))); margin=m.margin(m.toes(),WALK)
  lp=m.wrench(WALK) if err<=.8 and margin>=10 else {'pass':False}
  entry={'dx_mm':float(dx),'dy_mm':float(dy),'com_xy_mm':np.round(com*1000,3).tolist(),'ik_error_mm':round(float(err),3),'max_joint_rad':round(qmax,5),'margin_mm':round(float(margin),3),'static_lp_pass':bool(lp['pass']),'static_tau_Nm':lp.get('min_peak_tau_Nm'), 'pass':bool(err<=.8 and qmax<=1.7+1e-6 and margin>=10 and lp['pass'] and lp['min_peak_tau_Nm']<=.7)}
  res.append(entry)
 print('dx',dx,'npass',sum(x['pass'] for x in res),'t',round(time.time()-t,1),flush=True)
out={'schema':'CAD30-four-stance-fixed-pose-grid-v1','limits':'fixed torso Z/orientation, nominal CAD, 4 toes fixed, root-xy grid ±90×[-135,+45] step15mm, no dynamics', 'results':res,'passing':sum(x['pass'] for x in res),'total':len(res)}
path=Path(str(Path(__file__).resolve().parent/'four_stance_grid_z0.json'));path.write_text(json.dumps(out,indent=2))
print('WROTE',path,'PASS',out['passing'],'TOTAL',out['total'], 'duration',round(time.time()-t,2),flush=True)
