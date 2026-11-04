"""Find candidate body pose with 3 supports (swinging left middle), only kinematics."""
import json,numpy as np,mujoco,itertools,math
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
f=json.loads(Path(GUARD_PATH).read_text());m=mujoco.MjModel.from_xml_path(str(MODEL_PATH));d=mujoco.MjData(m);d.qpos[:]=f['qpos'];mujoco.mj_forward(m,d);legs={x['suffix']:x for x in load_legs(m,d,mujoco)};ref=mujoco.MjData(m);foot={c:tip_world(m,d,legs[c]).copy() for c in SUPPORT};cp=contact_points(m,d);tri=[np.mean(cp[c],axis=0) for c in SUPPORT if c!='ml'];bquat=d.qpos[3:7].copy();root=m.body('base_link_2').id
rs=[]
for dx,dy,dz,pitch in itertools.product([0,.02,.03,.04,.05,.06,.07,.08],[-.16,-.14,-.12,-.10,-.08,-.06],[0,.015,.03,.045,.06,.075],[-15,-8,0,8,15]):
 ref.qpos[:]=d.qpos;ref.qpos[:3]+=np.array([dx,dy,dz]);qr=np.empty(4);mujoco.mju_axisAngle2Quat(qr,np.array([1.,0.,0.]),math.radians(pitch));qq=np.empty(4);mujoco.mju_mulQuat(qq,qr,bquat);ref.qpos[3:7]=qq
 er={}
 for c in SUPPORT:
  if c=='ml':continue
  l=legs[c];q,e=solve_waypoint(m,ref,mujoco,l,foot[c],d.qpos[l['qadrs']]);ref.qpos[l['qadrs']]=q;er[c]=e
 if max(er.values())>.005:continue
 mujoco.mj_forward(m,ref);com=ref.subtree_com[root].copy();marg=margin(com[:2],tri)
 rs.append({'dx':dx,'dy':dy,'dz':dz,'pitch':pitch,'com':com[:2].tolist(),'margin':marg,'IK_max':max(er.values()),'max_angle':max(float(max(abs(ref.qpos[legs[c]['qadrs']]))) for c in ('mr','bl','br'))})
rs.sort(key=lambda x:x['margin'],reverse=True)
print('total',len(rs),'positive',sum(x['margin']>0 for x in rs));print('best',rs[:12]);
open(ROOTDIR/'results/three_support_ik_grid_2026-11-04.json','w').write(json.dumps({'attempts':8*6*6*5,'feasible':len(rs),'positive_margin':sum(x['margin']>0 for x in rs),'top':rs[:100]},indent=2)+'\n')
