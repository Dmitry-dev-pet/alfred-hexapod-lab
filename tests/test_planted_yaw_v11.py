"""Offline original CAD30 FK test: all body-yaw points hold six world toe anchors."""
import json,os,unittest,math
from pathlib import Path
from scipy.spatial.transform import Rotation as R
import numpy as np
import mujoco as mj
from hexlab.stand_geometry_v04 import load_legs,tip_world
ROOT=Path(__file__).resolve().parents[1]
SCENE=Path(os.environ.get('CAD30_SCENE_PATH',ROOT/'source/mjcf/scene_local30_nominal.xml'))
KIT=ROOT/'docs/hexapod/freenove-grounded-v11/planted-body-yaw-kit.json'
REF=ROOT/'docs/hexapod/data/live-v4-gaits-kit.json'

class PlantedYaw(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  if not SCENE.is_file():raise unittest.SkipTest('Original CAD30 owner source needed')
  cls.k=json.loads(KIT.read_text());cls.r=json.loads(REF.read_text())
  cls.model=mj.MjModel.from_xml_path(str(SCENE));cls.data=mj.MjData(cls.model)
  cls.root=np.array(cls.k['root_xyz_m'])
  cls.R0=R.from_quat([*cls.r['root_quat_wxyz'][1:],cls.r['root_quat_wxyz'][0]])
  cls.names=list(cls.k['cad_tip_local'])
  cls.legs=load_legs(cls.model,cls.data,mj)
 def toes(self,deg,q):
  d=self.data;d.qpos[:3]=self.root
  quat=(R.from_euler('z',deg,degrees=True)*self.R0).as_quat()
  d.qpos[3:7]=[quat[3],*quat[:3]]
  d.qpos[7:]=q;mj.mj_forward(self.model,d)
  return {l['suffix']:tip_world(self.model,d,l) for l in self.legs}
 def test_every_integer_yaw_degree_plants_six_toes(self):
  anchors={n:np.asarray(v) for n,v in self.k['ground_anchors_xyz_m'].items()}
  worst=0
  for deg in range(-15,16):
   for name,pos in self.toes(deg,self.k['rows'][str(deg)]).items():
    e=np.linalg.norm(pos-anchors[name])*1000;worst=max(worst,e)
    self.assertLess(e,.04,(deg,name,e))
  print('31 body-yaw poses, 186 planted toes, max err mm',round(worst,5))
 def test_half_degree_interpolation(self):
  anchors={n:np.asarray(v) for n,v in self.k['ground_anchors_xyz_m'].items()}
  worst=0
  for deg in [-14.5,-7.5,-.5,.5,7.5,14.5]:
   a=np.array(self.k['rows'][str(math.floor(deg))]);b=np.array(self.k['rows'][str(math.ceil(deg))]);q=(a+b)*.5
   for name,pos in self.toes(deg,q).items():
    e=np.linalg.norm(pos-anchors[name])*1000;worst=max(worst,e)
    self.assertLess(e,.1,(deg,name,e))
  print('intermediate body-yaw max err mm',round(worst,4))
 def test_data_and_hardware_policy(self):
  self.assertEqual(len(self.k['rows']),31)
  self.assertEqual(self.k['status'],'OFFLINE_IK_GEOMETRY_ONLY_NOT_PHYSICS')
  self.assertEqual(len(self.k['cad_tip_local']),6)
  self.assertFalse(any('hardware safe' in x.lower() for x in self.k['limitations']))
if __name__=='__main__':unittest.main()
