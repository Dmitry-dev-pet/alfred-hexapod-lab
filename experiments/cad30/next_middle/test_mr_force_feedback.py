"""Matched MuJoCo contact-feedback regression tests. Real CAD required; skip explicitly if absent."""
from __future__ import annotations
import json,os,sys,unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parent))
from mr_contact_continuation import Trial

class FeedbackContract(unittest.TestCase):
    def test_default_mode_off_is_explicit(self):
        import inspect
        params=inspect.signature(Trial.run).parameters
        self.assertFalse(params['touchdown_feedback'].default)
        self.assertGreater(params['touchdown_max_goal_rad'].default,0.)

    def test_stance_ids(self):
        from mr_contact_continuation import WALK,LEGS
        self.assertEqual(set(WALK),{'ml','mr','bl','br'})
        self.assertEqual(len(LEGS),6)

class PhysicalFeedback(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cad=Path(os.getenv('CAD30_ROOT','/path/unset'))
        cls.repo=Path(os.getenv('HEXAPOD_REPO_ROOT','/path/unset'))
        cls.state=Path(os.getenv('MR_AFTER_ML_CHECKPOINT','/path/unset'))
        if not (cls.cad/'source/mjcf/scene_local30_nominal.xml').is_file() or not cls.state.is_file():
            raise unittest.SkipTest('Private CAD30 + physical checkpoint not available; no fake integration pass')
    def run_trial(self,feedback,mu=1.1):
        trial=Trial(self.cad,self.repo,self.state,mu=mu,leg='mr')
        result,_=trial.run(step_mm=33,clearance_mm=18,swing_node_s=.35,pre_shift_xy_mm=(0,20),
            ml_servo_bias_rad=(0.,-.075,.055),initial_hold_s=.2,hold_s=.6,
            touchdown_feedback=feedback)
        return result
    def test_baseline_still_fails_strictly(self):
        out=self.run_trial(False)
        self.assertFalse(out['accepted'])
        self.assertEqual(out['trio_bad_force_frames'],8)
    def test_closed_loop_pass_nominal(self):
        out=self.run_trial(True)
        self.assertTrue(out['accepted'],out['failures'])
        self.assertEqual(out['trio_bad_force_frames'],0)
        self.assertGreater(out['min_fz_trio_N'],.2)
        self.assertGreaterEqual(out['min_margin_trio_mm'],10.)
        self.assertGreater(out['physical_toe_forward_mm'],20.)
        self.assertLessEqual(out['max_raw_tau_Nm'],.7)
        self.assertGreaterEqual(out['last_hold_fraction_four_loaded'],.99)
        self.assertGreater(out['touchdown_feedback']['updates'],0)
    def test_reduced_mu08_passes_local(self):
        out=self.run_trial(True,mu=.8)
        self.assertTrue(out['accepted'])
        self.assertEqual(out['trio_bad_force_frames'],0)

if __name__=='__main__':unittest.main()
