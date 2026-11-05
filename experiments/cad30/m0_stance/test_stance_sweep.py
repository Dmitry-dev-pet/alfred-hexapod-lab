import unittest
import numpy as np
from stance_sweep import Variant, candidate_targets, summarize_math, summarize_physics


class StanceSweepTests(unittest.TestCase):
    def test_candidate_offsets_preserve_originals(self):
        raw = {leg: np.array([0., 0., 0.01]) for leg in ("ml", "mr", "bl", "br", "fl", "fr")}
        v = Variant("test", 30, 80, 20, 0)
        got = candidate_targets(raw, v)
        self.assertAlmostEqual(got["ml"][0], 0.03)
        self.assertAlmostEqual(got["mr"][0], -0.03)
        self.assertAlmostEqual(got["bl"][0], -0.02)
        self.assertAlmostEqual(got["br"][0], 0.02)
        self.assertAlmostEqual(got["bl"][1], .08)
        self.assertAlmostEqual(got["fr"][2], .01)
        self.assertAlmostEqual(raw["bl"][1], 0)

    def test_math_summary_does_not_call_sampled_poses_gait(self):
        result = {"status": "SAMPLED_GEOMETRIC_STATIC_CANDIDATE", "errors": [],
                  "min_sampled_support_margin_mm": 7.8, "max_sampled_leg_joint_abs_rad": 1.67,
                  "max_sampled_required_static_tau_Nm": .41, "max_body_net_Y_from_six_prep_mm": 20.0}
        nodes = [{"root_xy_m": [0, -.03]}, {"root_xy_m": [0, -.095]}]
        summary = summarize_math(result, nodes)
        self.assertEqual(summary["sampled_errors"], 0)
        self.assertAlmostEqual(summary["minimum_planned_root_Y_relative_to_saved_six_mm"], -65)
        self.assertFalse(summary["is_physical_gait"])

    def test_failure_is_preserved(self):
        phys = {"status": "FAIL", "physics_steps": 1, "nodes_attempted": 130,
                "nodes_total": 218, "min_loaded_leg_count": 3,
                "COM_Y_delta_from_saved_six_start_mm": -61,
                "root_Y_delta_from_saved_six_start_mm": -80,
                "failure": {"phase": "swing_ml", "reason": "margin", "COM_margin_mm": -2.04}}
        summary = summarize_physics(phys)
        self.assertEqual(summary["first_failure_phase"], "swing_ml")
        self.assertFalse(summary["has_full_verified_cycle"])


if __name__ == "__main__":
    unittest.main()