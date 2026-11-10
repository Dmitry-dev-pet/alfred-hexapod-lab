"""Pure metadata + offline gait schedule checks; no live physics required."""
from pathlib import Path
import importlib.util, json, math, unittest
SCRIPT=Path(__file__).resolve().parents[1]/'scripts/build_live_v4_gaits.py'
spec=importlib.util.spec_from_file_location('build_live_v4_gaits',SCRIPT)
mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
KIT=Path(__file__).resolve().parents[1]/'docs/hexapod/data/live-v4-gaits-kit.json'
class TestGaits(unittest.TestCase):
  def test_schedule_supports(self):
    for gait,airborne in [('wave',1),('ripple',2),('tripod',3)]:
      for j in range(48):
        count=sum(mod.foot_curve(gait,i,j/48)[2] for i in range(6))
        self.assertEqual(count,airborne,(gait,j,count))
  def test_kit_no_physics_claim(self):
    k=json.loads(KIT.read_text())
    self.assertIn('KINEMATIC PREVIEW',k['notice'])
    self.assertEqual(k['stride_mm_levels'],[15,25,35])
    self.assertEqual(k['gaits'],['wave','ripple','tripod'])
    self.assertEqual(len(k['skeleton']),46)
    self.assertEqual(len(k['q_stand']),20)
    self.assertEqual(len(k['joint_tables']),3)
    for gait in k['gaits']:
      for length in k['stride_mm_levels']:
        t=k['joint_tables'][gait][str(length)]
        self.assertEqual(set(t),set(mod.MODES))
        for m in t:
          self.assertEqual(len(t[m]),48)
          self.assertTrue(all(len(q)==20 and all(math.isfinite(x) for x in q) for q in t[m]))
          self.assertLessEqual(k['max_abs_joint_rad'][gait][str(length)][m],1.7)
          self.assertLess(k['max_ik_error_mm'][gait][str(length)][m],1.)
if __name__=='__main__':unittest.main()
