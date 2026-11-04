"""Actual CAD30 tests; require the owner's model and archived pose-B replay."""
import os
import unittest
from pathlib import Path
from cad30_dynamic_v2 import Experiment

MODEL=Path(os.environ.get('CAD30_MODEL_XML','source/mjcf/scene_local30_nominal.xml'))
REPLAY=Path(os.environ.get('CAD30_REPLAY_ZIP','FNK0052_claw_tuck_ABC_2026-11-04_v2_no_CAD.zip'))

@unittest.skipUnless(MODEL.is_file() and REPLAY.is_file(),'Restore private CAD30 and pose-B replay locally')
class DynamicMicroTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.test_run=Experiment(MODEL,REPLAY,mu=1.1).run(target=1.66,micros=3)
        cls.hold_run=Experiment(MODEL,REPLAY,mu=1.1).run(target=1.66,micros=0)
        cls.low_friction=Experiment(MODEL,REPLAY,mu=0.6).run(target=1.66,micros=3)

    def test_four_real_contacts_on_every_nominal_physics_step(self):
        r=self.test_run
        self.assertEqual(r['every_physics_step']['minimum_loaded_contacts'],4)
        self.assertEqual(r['bad_step_count'],0)
        self.assertGreater(r['every_physics_step']['minimum_support_margin_mm'],10)
        self.assertGreater(r['every_physics_step']['minimum_stance_Fz_N'],0.2)

    def test_final_pose_has_research_joint_reserve(self):
        r=self.test_run
        self.assertLess(r['rebalance_end']['maxq'],1.7)
        self.assertLess(r['every_physics_step']['max_abs_joint_after_safe_rad'],1.7)

    def test_real_motor_caps_and_friction_lp(self):
        r=self.test_run
        self.assertEqual(r['actuator_saturated_steps'],0)
        self.assertLess(r['every_physics_step']['max_abs_tau_Nm'],.7)
        self.assertEqual(r['micro_executed'],3)
        self.assertEqual(len(r['lp_checks']),3)
        self.assertTrue(all(s['feasible'] for s in r['lp_checks']))
        self.assertTrue(all(min(s['predicted_Fz_N'])>=.19999 for s in r['lp_checks']))

    def test_micro_moves_a_bit_more_than_matched_hold_but_not_walking(self):
        self.assertGreater(self.test_run['micro_com_y_progress_mm']-self.hold_run['micro_com_y_progress_mm'],.2)
        self.assertLess(self.test_run['micro_com_y_progress_mm'],5)
        self.assertAlmostEqual(self.test_run['micro_max_contact_xy_drift_mm'],self.hold_run['micro_max_contact_xy_drift_mm'],delta=.15)

    def test_low_friction_vetoes_instead_of_claiming_success(self):
        r=self.low_friction
        self.assertEqual(r['micro_executed'],0)
        self.assertGreater(r['bad_step_count'],0)
        self.assertTrue(any(s['feasible'] is False for s in r['lp_checks']))

if __name__=='__main__': unittest.main()