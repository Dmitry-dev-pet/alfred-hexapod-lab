import unittest
from lite3_s2_robustness import VARIATIONS, StepAudit, modify_model
import lite3_onnx_joint_audit as base
import numpy as np
import mujoco
class S2Tests(unittest.TestCase):
 def test_expected_variations(self):
  self.assertEqual(len({v.name for v in VARIATIONS}),len(VARIATIONS))
  self.assertEqual(VARIATIONS[0].name,'original_baseline')
 def test_mass_scale_matches_610g_uniform(self):
  m=modify_model(base.get_model(),VARIATIONS[3])
  self.assertAlmostEqual(float(sum(m.body_mass)),0.610,places=6)
 def test_friction_changes_both_contacts(self):
  m=modify_model(base.get_model(),VARIATIONS[1])
  for i in range(m.ngeom):
   n=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,i)
   if n=='floor' or (n and n.endswith('_shank_collision')):
    self.assertAlmostEqual(float(m.geom_friction[i,0]),0.35,places=6)
 def test_torque_force_cap_applied(self):
  m=modify_model(base.get_model(),VARIATIONS[4])
  self.assertTrue(np.allclose(m.actuator_forcerange[:,1],0.2256))
 def test_clean_counter(self):
  d=StepAudit().as_dict()
  self.assertEqual(d['physics_steps'],0)
  self.assertEqual(d['fraction_steps_less_3_loaded'],0.0)
if __name__=='__main__':unittest.main()