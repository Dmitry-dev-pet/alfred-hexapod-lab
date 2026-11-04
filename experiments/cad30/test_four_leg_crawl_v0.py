import json,unittest
from pathlib import Path
from four_leg_support_gate import diagnostic_liftoff_gate
from four_leg_crawl_v0 import run
from front_pair_trick import margin
import numpy as np,mujoco
ROOT=Path(__file__).resolve().parent
class DiagnosticGate(unittest.TestCase):
 def test_geometry_square(self):
  p=np.array([[-.1,-.1],[.1,-.1],[.1,.1],[-.1,.1]])
  self.assertAlmostEqual(margin((0,0),p),.1)
  self.assertLess(margin((0,.14),p),0)
 def test_geometry_of_frozen_four_contact_state(self):
  r=json.loads((ROOT/'results/four_leg_crawl_wave_nominal_trace.json').read_text())
  sample=min(r['frames'],key=lambda e:abs(e['t']-3.9))
  m=mujoco.MjModel.from_xml_path(str(ROOT/'source/mjcf/scene_local30_nominal.xml'))
  d=mujoco.MjData(m);d.qpos[:]=sample['qpos'];mujoco.mj_forward(m,d)
  v=diagnostic_liftoff_gate(m,d)
  self.assertTrue(v['bl']['passes_static_gate'])
  self.assertTrue(v['br']['passes_static_gate'])
  self.assertFalse(v['ml']['passes_static_gate'])
  self.assertFalse(v['mr']['passes_static_gate'])
 def test_negative_result_reproduces(self):
  r=run(step=.012,lift=.008,shift=.037,period=4.,seconds=8.,mu=1.1,mass='nominal',record=False)
  self.assertTrue(r['complete'])
  self.assertEqual(r['front_contact_steps'],0)
  self.assertEqual(r['legs_show_positive_mesh_clearance']['ml'],0.)
  self.assertEqual(r['legs_show_positive_mesh_clearance']['mr'],0.)
  self.assertLess(abs(r['net_advance']),.002)
if __name__=='__main__':unittest.main()
