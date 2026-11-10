"""Independent CAD30 geometric tests for factory command preview, no dynamics."""
import json,math,os,sys,unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import mujoco as mj
from hexlab.stand_geometry_v04 import load_legs,tip_world
ROOT=Path(__file__).resolve().parents[1]
KIT=ROOT/'docs/hexapod/data/freenove-original-v7-kit.json'
REF=ROOT/'docs/hexapod/data/live-v4-gaits-kit.json'
SCENE=Path(os.environ['CAD30_SCENE_PATH']) if 'CAD30_SCENE_PATH' in os.environ else None
sys.path.insert(0,str(ROOT/'scripts'))
from build_factory_v7 import vendor_gait,CAD_TO_VENDOR_LEG,VENDOR_POINTS,VENDOR_LEGS,OUTPUT_SAMPLES

class TestOEM(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.f=json.loads(KIT.read_text());cls.r=json.loads(REF.read_text())
  if SCENE is None or not SCENE.is_file(): raise unittest.SkipTest('Original CAD30 scene required')
  cls.m=mj.MjModel.from_xml_path(str(SCENE));cls.d=mj.MjData(cls.m)
  cls.legs=load_legs(cls.m,cls.d,mj)
  cls.q20=np.asarray(cls.f['q_home']);cls.home_root=np.asarray(cls.f['cad_adapter']['neutral_root_xyz_m'])
  cls.qrot=np.asarray(cls.r['root_quat_wxyz'])
 def pose(self,q,shift=None,rot=None):
  d=self.d;d.qpos[:3]=self.home_root+(np.asarray(shift) if shift is not None else 0)
  R0=Rotation.from_quat([self.qrot[1],self.qrot[2],self.qrot[3],self.qrot[0]])
  if rot is not None:R0=Rotation.from_rotvec(rot)*R0
  x,y,z,w=R0.as_quat();d.qpos[3:7]=[w,x,y,z]
  d.qpos[7:]=q;mj.mj_forward(self.m,d)
  return {z['suffix']:tip_world(self.m,d,z).copy() for z in self.legs}
 def test_neutral_factory_xy_mapping_and_ground(self):
  actual=self.pose(self.q20)
  self.assertEqual(set(actual),set(CAD_TO_VENDOR_LEG))
  for name,xyz in actual.items():
   vendor=VENDOR_POINTS[VENDOR_LEGS.index(CAD_TO_VENDOR_LEG[name])]
   self.assertLess(np.linalg.norm(xyz-np.array([-vendor[0],vendor[1],0])/1000),.0005)
 def test_vendor_native_periods_and_lift(self):
  for mode,F in [('1',48),('2',108)]:
   frames,n=vendor_gait(mode,0,35,0);self.assertEqual(n,F)
   self.assertEqual(len(frames),n)
   self.assertTrue(all(len(z)==6 for z in frames))
   self.assertGreater(max(z[2]+25 for sample in frames for z in sample),39.9)
   self.assertAlmostEqual(min(z[2]+25 for sample in frames for z in sample),0,delta=.1)
   travel=max(z[1] for sample in frames for z in sample)-min(z[1] for sample in frames for z in sample)
   self.assertGreater(travel,35)
 def test_full_oem_cad_gait_fk_sampled(self):
  maxerr=0
  for mode,key in [('1','tripod'),('2','wave')]:
   for direction in ['forward','back','left','right','forward_left','forward_right','back_left','back_right','cw','ccw']:
    from build_factory_v7 import DIRECTION_COMMANDS
    pts,F=vendor_gait(mode,*DIRECTION_COMMANDS[direction]);rows=self.f['gaits'][key]['rows'][direction]
    for i in [0,2,7,11,15,23,30,39,47]:
     ix=i*F/OUTPUT_SAMPLES;low=int(ix);f=ix-low;upper=min(low+1,F-1)
     desired=[np.asarray(pts[low][n])*(1-f)+np.asarray(pts[upper][n])*f for n in range(6)]
     actual=self.pose(rows[i]);
     for name,coord in actual.items():
      p=desired[VENDOR_LEGS.index(CAD_TO_VENDOR_LEG[name])]
      target=np.array([-p[0],p[1],p[2]+25])/1000
      err=np.linalg.norm(coord-target);maxerr=max(err,maxerr)
      self.assertLess(err,.0008,(mode,direction,i,name,err))
  print('Independent OEM CAD30 FK error mm',round(maxerr*1000,5))
 def test_full_body_oem_extremes_plant_feet(self):
  target=self.pose(self.q20)
  limits=self.f['factory_commands'];angles=math.radians(limits['attitude_clamp_deg'])
  basis={
   'left_m':([-limits['position_xy_clamp_mm']/1000,0,0],[0,0,0]),
   'forward_m':([0,limits['position_xy_clamp_mm']/1000,0],[0,0,0]),
   'height_m':([0,0,limits['position_z_clamp_mm']/1000],[0,0,0]),
   'roll_rad':([0,0,0],[0,angles,0]),
   'pitch_rad':([0,0,0],[-angles,0,0]),
   'yaw_rad':([0,0,0],[0,0,angles])
  }
  maxerr=0
  for key,(s,r) in basis.items():
   for sign,side in [(1,'pos'),(-1,'neg')]:
    actual=self.pose(self.f['body_axes'][key][side],np.asarray(s)*sign,np.asarray(r)*sign)
    for name in target:
     error=np.linalg.norm(actual[name]-target[name]);maxerr=max(maxerr,error)
     self.assertLess(error,.0008,(key,side,name))
  print('Independent factory body toe plant error mm',round(maxerr*1000,5))
 def test_labeling_and_range_policy(self):
  self.assertTrue(self.f['cad_adapter']['unverified_real_motor_bounds'])
  self.assertEqual(self.f['factory_commands']['lift_mm'],40)
  self.assertEqual(self.f['factory_commands']['position_xy_clamp_mm'],40)
  self.assertEqual(self.f['factory_commands']['attitude_clamp_deg'],15)
  self.assertLess(self.f['max_ik_error_mm'],.8)
  self.assertGreater(self.f['max_abs_joint_rad'],1.7)
  self.assertNotIn('ripple',self.f['gaits'])
if __name__=='__main__':unittest.main()
