"""Math-only acceptance checks. No MuJoCo dynamics or hardware assertions."""
import json,unittest
from pathlib import Path
HERE=Path(__file__).resolve().parent
RESULT=(HERE/'results'/'math_first_four_crawl_result.json') if (HERE/'results'/'math_first_four_crawl_result.json').exists() else HERE/'math_first_four_crawl_result.json'
TRACE=HERE/'math_first_four_crawl_trace.json'
R=json.loads(RESULT.read_text())

class MathFirstGaitReceiptTests(unittest.TestCase):
 def test_receipt_is_explicitly_math_only(self):
  self.assertEqual(R['status'],'SAMPLED_GEOMETRIC_STATIC_CANDIDATE')
  self.assertIn('No dynamic mj_step.',R['caveats'][0])
 def test_four_steps_and_closure(self):
  self.assertEqual(R['cycle_order'],['ml','mr','bl','br'])
  self.assertEqual(R['cycle_terminal_root_plus_world_Y_mm'],20)
  self.assertAlmostEqual(R['max_body_net_Y_from_six_prep_mm'],-10,places=8)
 def test_numerical_gates(self):
  self.assertEqual(R['number_of_sampled_nodes'],218)
  self.assertGreater(R['min_sampled_support_margin_mm'],5)
  self.assertLess(R['max_sampled_toe_error_mm'],0.7)
  self.assertLessEqual(R['max_sampled_leg_joint_abs_rad'],1.7)
  self.assertLess(R['max_sampled_required_static_tau_Nm'],0.7)
  self.assertGreater(R['min_swing_mesh_ground_clearance_mm'],2)
  self.assertEqual(R['errors'],[])

class MathTraceVerificationTests(unittest.TestCase):
 @unittest.skipUnless(TRACE.exists(), 'Full 218-node mathematical trace not committed to GitHub; extract companion ZIP from Drive')
 def test_full_sampled_contact_schedule(self):
  nodes=json.loads(TRACE.read_text())['nodes']
  self.assertEqual(len(nodes),R['number_of_sampled_nodes'])
  steps={n['phase'].replace('swing_','') for n in nodes if n['phase'].startswith('swing_')}
  self.assertEqual(steps,set(R['cycle_order']))
  for n in nodes:
   self.assertTrue(n['static_lp_pass'])
   self.assertGreaterEqual(len(n['grounded']),3)
   self.assertLessEqual(len(n['grounded']),6)
   if n['phase'].startswith('swing_') and n.get('swing'):
    self.assertEqual(len(n['grounded']),3)
if __name__=='__main__':unittest.main()
