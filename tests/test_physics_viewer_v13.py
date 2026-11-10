"""Audit of replay vs 53 original full-body MuJoCo V12 physics receipts."""
import hashlib, json, math, unittest
from pathlib import Path
R=Path(__file__).resolve().parents[1]
P=R/'docs/hexapod/data'
SOURCE=R/'research/physics_v12/results'
SETS={'nominal_full':29,'upper_mass_full':10,'friction_035_full':7,'torque_035_full':7}
MODEL_SHA='fe9d7c292b199f19b11da220e74f826e05fb5433a43c5b46c1bc620eb34f6df3'
LEGS=('ml','bl','br','mr','fr','fl')
class ActualPhysicsReplay(unittest.TestCase):
 def test_all_53_original_runs_and_physical_states(self):
  if not SOURCE.is_dir():self.skipTest('Unpack private V12 raw receipts from Drive for deep audit')
  count=0
  for variant,num in SETS.items():
   f=P/('physics-v12-'+variant+'.json');data=json.loads(f.read_text())
   self.assertEqual(data['format'],'hexapod-actual-mujoco-v12-replay-v13')
   self.assertEqual(data['variant'],variant)
   self.assertEqual(data['hz'],50)
   self.assertEqual(len(data['runs']),num)
   if variant!='upper_mass_full':self.assertEqual(data['original_v12_scene_sha256'],MODEL_SHA)
   for name,trace in data['runs'].items():
    count+=1;archived=SOURCE/variant/(name+'.json');old=json.loads(archived.read_text())
    self.assertEqual(trace['provenance']['original_case_sha256'],hashlib.sha256(archived.read_bytes()).hexdigest())
    frames=trace['frames'];self.assertEqual(len(frames),len(old['samples'])+1)
    self.assertEqual(len(frames[0]),36);self.assertEqual(frames[0][0],0)
    self.assertAlmostEqual(frames[-1][0],old['duration_simulated_s'],delta=.021)
    for frame,ref in zip(frames[1:],old['samples']):
     self.assertEqual(len(frame),36)
     self.assertAlmostEqual(frame[0],ref['t_s'],delta=.0001)
     for key,i in [('root_x_m',1),('root_y_m',2),('root_z_m',3)]:self.assertAlmostEqual(frame[i],ref[key],delta=.0012) # V12 reports base_link_2 body xpos; replay records free-joint origin qpos
     self.assertAlmostEqual(math.hypot(*frame[4:8]),1,delta=.000001)
     for leg,v in zip(LEGS,frame[30:36]):self.assertAlmostEqual(v,ref['per_leg_Fz_N'][leg],delta=.011)
     mask=sum(1<<i for i,leg in enumerate(LEGS) if ref['per_leg_Fz_N'][leg]>.05)
     self.assertEqual(frame[28],mask)
     self.assertAlmostEqual(frame[29],ref['max_contact_point_speed_mm_s'],delta=.011)
    if variant=='nominal_full':
     self.assertAlmostEqual(trace['summary']['motor_saturation'],old['actuator_saturation_fraction'],delta=.00001)
  self.assertEqual(count,53)
 def test_logged_pose_is_actually_different_from_joint_targets(self):
  if not (R/'research/physics_v12/data/targets_v11.json').is_file():self.skipTest('Unpack exact V12 target profiles from Drive')
  import json
  rec=json.loads((P/'physics-v12-nominal_full.json').read_text())['runs']['tripod_forward']['frames']
  targets=json.loads((R/'research/physics_v12/data/targets_v11.json').read_text())['cases']['tripod_forward']['targets_q20']
  self.assertGreater(abs(rec[40][8]-targets[80][0]),.001) # actual lag vs commanded q, t=0.8s
 def test_no_physics_or_contact_lock_in_viewer(self):
  a=(R/'docs/hexapod/physics-replay-v13/app.js').read_text()
  self.assertIn('const root=vec(s.root),qroot=quat(s.quat)',a)
  self.assertIn('s.joints[node.adr]-ref.q_stand[node.adr]',a)
  self.assertIn("fetch(url,{cache:'no-store'})",a)
  for bad in ('mj_step(', 'WebSocket(', 'GroundLock(', 'new Worker(', 'new FactoryPilot(', 's.x +=','pose.yaw +='):
   self.assertNotIn(bad,a)
 def test_subresource_graph_and_complete_markup(self):
  from html.parser import HTMLParser
  h=(R/'docs/hexapod/physics-replay-v13/index.html').read_text()
  for item in ('id="variant"','id="experiment"','id="timeline"','id="toggle"','id="clock"','id="contact-line"','id="slip-line"','id="reset-camera"','window.addEventListener(\'error\''):
   self.assertIn(item,h)
  s=(R/'docs/hexapod/physics-replay-v13/app.js').read_text()
  self.assertIn("from './replay-controller.js'",s)
  self.assertIn('validatePack(pack)',s)
  for file in ('app.js','replay-controller.js','index.html','replay.css'):
   self.assertTrue((R/'docs/hexapod/physics-replay-v13'/file).is_file())
if __name__=='__main__':unittest.main()
