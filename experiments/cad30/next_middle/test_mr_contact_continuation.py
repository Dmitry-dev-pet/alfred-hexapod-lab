"""Deterministic mathematics and optional private-CAD regression for MR continuation.

The simulation deliberately has a strict FAIL; tests check the negative control
and the independently measured partial toe transfer without mislabeling gait.
"""
from __future__ import annotations
import os,sys,json, unittest
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from mr_contact_continuation import smoothstep,Trial

class NumericContract(unittest.TestCase):
    def test_smoothstep_endpoints(self):
        self.assertEqual(smoothstep(0.),0.)
        self.assertEqual(smoothstep(1.),1.)
        self.assertAlmostEqual(smoothstep(.5),.5)
    def test_smoothstep_monotonic(self):
        x=np.linspace(0,1,21)
        self.assertTrue(np.all(np.diff([smoothstep(v) for v in x])>=0))
    def test_saved_receipt_is_explicit_fail(self):
        x=json.loads((HERE/'results/continuous_six_ml_mr_20261105.json').read_text())
        self.assertEqual(x['prelude_status'],'PASS_TO_PRE_ML')
        self.assertEqual(x['ml_status'],'LOCAL_ML_LIFT_SWING_TOUCHDOWN_PASS')
        self.assertEqual(x['mr_status'],'FAIL')
        self.assertFalse(x['mr_accepted_strict'])
        self.assertGreater(x['mr_actual_toe_world_forward_mm'],20)
        self.assertGreater(x['mr_force_loss_frames'],0)
        self.assertTrue(x['not_validated_full_gait'])
    def test_checkpoint_dimensions(self):
        x=json.loads((HERE/'results/after_ml_physical_checkpoint_20261105.json').read_text())
        self.assertEqual(len(x['qpos']),27)
        self.assertEqual(len(x['qvel']),26)
        self.assertEqual(len(x['motor_goals']),20)

class ActualCad30Regression(unittest.TestCase):
    @unittest.skipUnless(os.environ.get('CAD30_ROOT'), 'Requires original private CAD30; SKIP rather than simulate proxy')
    def test_mr_partial_and_contact_chatter(self):
        repo=Path(os.environ.get("HEXAPOD_REPO_ROOT", HERE.parents[3]))
        tr=Trial(Path(os.environ['CAD30_ROOT']),repo,HERE/'results/after_ml_physical_checkpoint_20261105.json')
        r,_=tr.run(step_mm=33,clearance_mm=18,swing_node_s=.35,
            pre_shift_xy_mm=(0.,20.),ml_servo_bias_rad=(0.,-.075,.055),hold_s=.6)
        self.assertFalse(r['accepted'])
        self.assertGreater(r['physical_toe_forward_mm'],20)
        self.assertGreater(r['zero_contact_tibia_total_s'],.25)
        self.assertGreaterEqual(r['trio_bad_force_frames'],1)
        self.assertLessEqual(r['max_raw_tau_Nm'],.7)
        self.assertLessEqual(r['max_leg_joint_rad'],1.7)
if __name__=='__main__':unittest.main()
