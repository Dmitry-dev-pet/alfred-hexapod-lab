"""Pure-Python regression tests for CAD30 contact-trace diagnostics."""
import unittest

from contact_chatter_audit import audit, index_runs


class ContactAuditTests(unittest.TestCase):
    def test_runs(self):
        self.assertEqual(index_runs([0, 1, 3, 7, 8]), [[0, 1], [3], [7, 8]])

    def test_isolated_fz_drop_causes_strict_fail(self):
        phases = ["initial_hold", "swing_mr", "swing_mr", "swing_mr", "final_hold", "final_hold"]
        frames = []
        for i, phase in enumerate(phases):
            fz = {leg: 2.0 for leg in ("ml", "mr", "bl", "br")}
            if i == 2:
                fz["ml"] = 0
            if i == 4:
                fz["mr"] = 0
            frames.append({"time_s": 10.0 + 0.002 * i, "phase": phase, "Fz_N": fz})
        res = audit(frames)
        self.assertFalse(res["swing"]["strict_pass"])
        self.assertEqual(res["swing"]["failed_frames"], 1)
        self.assertEqual(res["swing"]["max_consecutive_failed_frames"], 1)
        self.assertEqual(res["swing"]["missing_by_leg"]["ml"], 1)
        self.assertAlmostEqual(res["final_hold"]["continuous_good_tail_s"], 0.002)
        self.assertFalse(res["overall_strict_contact_pass"])

    def test_fully_loaded(self):
        frames = [{"time_s": 0.002 * i, "phase": phase,
                   "Fz_N": {leg: 1.0 for leg in ("ml", "mr", "bl", "br")}}
                  for i, phase in enumerate(("swing_mr", "swing_mr", "final_hold", "final_hold"))]
        res = audit(frames)
        self.assertTrue(res["overall_strict_contact_pass"])
        self.assertEqual(res["final_hold"]["continuous_good_tail_s"], 0.004)

    def test_nonuniform_trace_rejected(self):
        frames = [{"time_s": t, "phase": phase,
                   "Fz_N": {leg: 1.0 for leg in ("ml", "mr", "bl", "br")}}
                  for t, phase in ((0., "swing_mr"), (.002, "swing_mr"), (.1, "final_hold"))]
        with self.assertRaises(ValueError):
            audit(frames)


if __name__ == "__main__":
    unittest.main()