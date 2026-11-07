"""Independent MuJoCo forward-kinematics checking offline posture and leg tables.

All checks are geometry only; no dynamics/forces/servo claims.
"""
import json,math,os,unittest
from pathlib import Path
import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation as R
from hexlab.stand_geometry_v04 import load_legs,tip_world

ROOT=Path(__file__).resolve().parents[1]
MESH=Path(os.environ['CAD30_SCENE_PATH']) if 'CAD30_SCENE_PATH' in os.environ else None
WALK=Path(os.environ['V4_GAITS_KIT_PATH']) if 'V4_GAITS_KIT_PATH' in os.environ else ROOT/'docs/hexapod/data/live-v4-gaits-kit.json'
POSE=ROOT/'docs/hexapod/data/live-v5-motion-kit.json'

class PoseTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  if MESH is None or not MESH.is_file() or not WALK.is_file():raise unittest.SkipTest('Requires CAD30_SCENE_PATH and checked-out V4 gait kit')
  cls.k=json.loads(WALK.read_text());cls.x=json.loads(POSE.read_text());cls.m=mj.MjModel.from_xml_path(str(MESH));cls.d=mj.MjData(cls.m)
  cls.q0=np.r_[cls.k['root_position'],cls.k['root_quat_wxyz'],cls.k['q_stand']]
  cls.d.qpos[:]=cls.q0;mj.mj_forward(cls.m,cls.d)
  cls.legs=load_legs(cls.m,cls.d,mj)
  cls.tips={leg['suffix']:tip_world(cls.m,cls.d,leg).copy() for leg in cls.legs}
  cls.f=np.array(cls.k['forward_world']);cls.l=np.array(cls.k['left_world']);cls.up=np.array([0,0,1])
  cls.qstand=np.array(cls.k['q_stand'])
  cls.ext=cls.x
 def compute(self,q,translation=np.zeros(3),rotvec=np.zeros(3)):
  d=self.d;d.qpos[:]=self.q0
  d.qpos[:3]+=translation
  r0=R.from_quat([*self.k['root_quat_wxyz'][1:],self.k['root_quat_wxyz'][0]])
  newR=R.from_rotvec(rotvec)*r0
  xyzw=newR.as_quat();d.qpos[3:7]=[xyzw[3],xyzw[0],xyzw[1],xyzw[2]]
  d.qpos[7:]=q
  mj.mj_forward(self.m,d)
  return {leg['suffix']:tip_world(self.m,d,leg).copy() for leg in self.legs}
 def test_body_axes_keep_world_toes_fixed(self):
  max_error=0
  for axis,limit in self.x['pose_limits'].items():
   for sign,kind in [(-1,'neg'),(1,'pos')]:
    t=np.zeros(3);rv=np.zeros(3)
    if axis=='forward_m':t=self.f*sign*limit
    elif axis=='left_m':t=self.l*sign*limit
    elif axis=='height_m':t=self.up*sign*limit
    elif axis=='roll_rad':rv=self.f*sign*limit
    elif axis=='pitch_rad':rv=self.l*sign*limit
    elif axis=='yaw_rad':rv=self.up*sign*limit
    q=self.x['pose_axes'][axis][kind];feet=self.compute(q,t,rv)
    for name,p in feet.items():
     err=float(np.linalg.norm(p-self.tips[name]));max_error=max(max_error,err)
     self.assertLess(err,.0008,(axis,kind,name,err))
  print('V5 FK single axis foot-lock max error (mm):',round(max_error*1000,6))
 def test_selected_leg_3d_foot_motion(self):
  max_error=0
  for name,idx in self.x['leg_joint_indices'].items():
   for axis,limit in self.x['leg_limits'].items():
    for sign,side in [(-1,'neg'),(1,'pos')]:
     q=self.qstand.copy();q[idx]=self.x['leg_axes'][name][axis][side]
     feet=self.compute(q)
     unit={'forward_m':self.f,'left_m':self.l,'lift_m':self.up}[axis]
     desired=self.tips[name]+sign*limit*unit
     error=float(np.linalg.norm(feet[name]-desired));max_error=max(max_error,error)
     self.assertLess(error,.0008,(name,axis,side,error))
     self.assertLess(max(np.linalg.norm(feet[other]-self.tips[other]) for other in feet if other!=name),.00001)
  print('V5 FK isolated leg tip max error (mm):',round(max_error*1000,6))
 def test_body_limits_all_single_axes(self):
  for axis,pair in self.x['pose_axes'].items():
   for q in pair.values():
    self.assertLessEqual(np.abs(q[:18]).max(),1.7)
 def test_no_unverified_physics(self):
  self.assertFalse(self.x['physical_guarantee']);self.assertTrue(self.x['no_realtime_physics'])
  self.assertFalse(self.x['original_assets_included'])
if __name__=='__main__':unittest.main()
