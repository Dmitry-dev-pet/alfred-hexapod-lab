"""Snapshot acceptance checks for the saved math-only receipt; NOT a dynamics test."""
import json,unittest
from pathlib import Path
HERE=Path(__file__).resolve().parent
R=json.loads((HERE/'math_first_four_crawl_result.json').read_text())
T=json.loads((HERE/'math_first_four_crawl_trace.json').read_text())
class MathFirstGaitReceiptTests(unittest.TestCase):
 def test_receipt_is_explicitly_math_only(self):
  self.assertEqual(R['status'],'SAMPLED_GEOMETRIC_STATIC_CANDIDATE')
  self.assertIn('No dynamic mj_step.',R['caveats'][0])
 def test_four_steps_and_closure(self):
  steps=tuple(k.replace('swing_','') for k in [n['phase'] for n in T['nodes']] if k.startswith('swing_'))
  self.assertEqual(set(steps),set(R['cycle_order']))
  self.assertEqual(R['cycle_terminal_root_plus_world_Y_mm'],20)
  self.assertAlmostEqual(R['max_body_net_Y_from_six_prep_mm'],-10,places=8)
 def test_numerical_gates(self):
  self.assertEqual(len(T['nodes']),218)
  self.assertGreater(R['min_sampled_support_margin_mm'],5)
  self.assertLess(R['max_sampled_toe_error_mm'],0.7)
  self.assertLessEqual(R['max_sampled_leg_joint_abs_rad'],1.7)
  self.assertLess(R['max_sampled_required_static_tau_Nm'],0.7)
  self.assertGreater(R['min_swing_mesh_ground_clearance_mm'],2)
  self.assertEqual(R['errors'],[])
 def test_contact_schedule(self):
  for n in T['nodes']:
   self.assertTrue(n['static_lp_pass'])
   self.assertGreaterEqual(len(n['grounded']),3)
   self.assertLessEqual(len(n['grounded']),6)
   if n['phase'].startswith('swing_') and n.get('swing'):
    self.assertEqual(len(n['grounded']),3)
if __name__=='__main__':unittest.main()
