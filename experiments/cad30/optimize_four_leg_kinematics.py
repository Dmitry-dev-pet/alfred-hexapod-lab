"""Diagnostic kinematic search for achievable body poses, NOT dynamic gait verification."""
import numpy as np,mujoco,json,math,itertools
from scipy.optimize import linprog
from pathlib import Path
import sys
HERE=Path(__file__).resolve().parent
ROOTDIR=HERE if (HERE/'source/mjcf').is_dir() else HERE.parents[1]
if str(ROOTDIR) not in sys.path: sys.path.insert(0,str(ROOTDIR))
MODEL_PATH=ROOTDIR/str(MODEL_PATH)
GUARD_PATH=ROOTDIR/'results/optimized_four_leg_guard_reference_2026-11-04.json'
from hexlab.stand_geometry_v04 import load_legs,tip_world
from hexlab.step_one_leg_v04 import solve_waypoint
from front_pair_trick import SUPPORT,contact_points,margin
record=json.loads(Path(GUARD_PATH).read_text())
m=mujoco.MjModel.from_xml_path(str(MODEL_PATH));d=mujoco.MjData(m);d.qpos[:]=record['qpos'];mujoco.mj_forward(m,d);legs={l['suffix']:l for l in load_legs(m,d,mujoco)};feet={c:tip_world(m,d,legs[c]).copy() for c in SUPPORT}
cp=contact_points(m,d);tri={c:[np.mean(cp[z],axis=0) for z in SUPPORT if z!=c] for c in SUPPORT};com0=d.subtree_com[m.body('base_link_2').id].copy();ref=mujoco.MjData(m)
basequat=d.qpos[3:7].copy()
res=[]
for dx,dy,dz,pitch in itertools.product([-.06,-.04,-.02,0,.02,.04,.06],[-.14,-.12,-.10,-.08,-.06,-.04,-.02,0],[-.02,0,.02,.04,.06],[math.radians(x) for x in [-12,-6,0,6,12]]):
 ref.qpos[:]=d.qpos;ref.qpos[:3]+=np.array([dx,dy,dz]);qrot=np.empty(4);mujoco.mju_axisAngle2Quat(qrot,np.array([1.,0,0]),pitch);qnew=np.empty(4);mujoco.mju_mulQuat(qnew,qrot,basequat);ref.qpos[3:7]=qnew
 errs=[];qs=[]
 for c in SUPPORT:
  l=legs[c];q0=d.qpos[l['qadrs']];q,err=solve_waypoint(m,ref,mujoco,l,feet[c],q0);ref.qpos[l['qadrs']]=q;errs.append(err);qs.append(q)
 maxerr=max(errs)
 if maxerr>.004: continue
 mujoco.mj_forward(m,ref);COM=ref.subtree_com[m.body('base_link_2').id].copy();margins={c:margin(COM[:2],tri[c]) for c in SUPPORT};worst=min(margins.values())
 res.append(dict(dx=dx,dy=dy,dz=dz,pitch_deg=math.degrees(pitch),com=COM[:2].tolist(),root=ref.xpos[m.body('base_link_2').id].tolist(),err=maxerr,max_q=max(np.max(np.abs(q)) for q in qs),margin_ml=margins['ml'],margin_mr=margins['mr'],margin_bl=margins['bl'],margin_br=margins['br'],score=min(margins['ml'],margins['mr'])))
res=sorted(res,key=lambda x:x['score'],reverse=True);print('feasible',len(res),'from',7*8*5*5,'baseline_COM',com0[:2]);
for x in res[:10]:print({k:round(v,4) if isinstance(v,float) else v for k,v in x.items()})
with open(ROOTDIR/'results/optimized_pose_workspace_grid_2026-11-04.json','w') as f:json.dump({'configuration':'4 support feet fixed in world; front legs airborne; body pose sampled; independent 3DOF IK; no dynamic forces; no hardware','baseline_COM':com0[:2].tolist(),'feasible_count':len(res),'trial_count':7*8*5*5,'top':res[:100],'best':res[0] if res else None},f,indent=2)
