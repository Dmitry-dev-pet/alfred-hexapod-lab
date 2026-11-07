"""Offline tests for SeBaJu physics-result classification (no MuJoCo needed)."""
import importlib.util
from pathlib import Path
import unittest

SRC = Path(__file__).resolve().parents[1] / "research/sebaju/run_headless.py"
spec = importlib.util.spec_from_file_location("alfred_sebaju", SRC)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


def sample(t, airborne=False, tilt=2.0, grounded=None, wheel_speed=0.02):
    if grounded is None:
        grounded = [not airborne, not airborne]
    return {
        "t": round(t, 3),
        "in_flight": airborne,
        "wheel_gap_min_m": 0.035 if airborne else 0.0,
        "tilt_deg": tilt,
        "grounded": grounded,
        "wheel_speed": wheel_speed,
    }


class SeBaJuAnalysisTests(unittest.TestCase):
    def test_longest_contiguous_flight_not_sum_of_airborne_gaps(self):
        records = [{"in_flight": x} for x in (False, True, True, False, True, True, True, False)]
        self.assertEqual(mod.longest_true_interval(records, "in_flight"), (4, 7))

    def test_jump_recovery_requires_real_contact_and_stable_window(self):
        records = []
        for tick in range(1300):
            t = tick / 100
            airborne = 5.8 <= t < 6.0
            records.append(sample(t, airborne=airborne,
                                  tilt=18 if 6 <= t < 6.2 else 2))
        result = mod.analyze(records, 0.01, "jump", 12)
        self.assertTrue(result["jump_detected"])
        self.assertAlmostEqual(result["flight_duration_s"], 0.2)
        self.assertAlmostEqual(result["touchdown_s"], 6.0)
        self.assertTrue(result["landing_recovery_observed"])
        self.assertNotIn("self_righted", result)

    def test_no_takeoff_cannot_be_claimed_as_jump(self):
        records = [sample(t / 100) for t in range(1300)]
        result = mod.analyze(records, 0.01, "jump", 12)
        self.assertFalse(result["jump_detected"])
        self.assertIsNone(result["touchdown_s"])

    def test_rolling_torque_not_equal_to_successful_self_right(self):
        records = []
        for tick in range(1800):
            t = tick / 100
            tilt = 85 if t >= 12.2 else 2
            records.append(sample(t, tilt=tilt, grounded=[False, False] if t >= 12.2 else None))
        result = mod.analyze(records, 0.01, "tip", 12.0)
        self.assertTrue(result["physical_tip_observed"])
        self.assertFalse(result["self_righted"])

    def test_no_physical_tip_is_inconclusive_not_success(self):
        records = [sample(i / 100) for i in range(1800)]
        result = mod.analyze(records, 0.01, "tip", 12.0)
        self.assertFalse(result["physical_tip_observed"])
        self.assertIsNone(result["self_righted"])
        self.assertEqual(result["tip_assessment"], "inconclusive-no-tip")

    def test_tip_then_true_upright_hold(self):
        records = []
        for tick in range(1800):
            t = tick / 100
            tilt = 75 if 12.1 <= t < 12.6 else 3
            grounded = [False, False] if 12.1 <= t < 12.6 else [True, True]
            records.append(sample(t, tilt=tilt, grounded=grounded))
        result = mod.analyze(records, 0.01, "tip", 12.0)
        self.assertTrue(result["physical_tip_observed"])
        self.assertTrue(result["self_righted"])
        self.assertGreater(result["self_righted_at_s"], 13.0)

    def test_empty_observations_fail_closed(self):
        with self.assertRaises(ValueError):
            mod.analyze([], 0.002, "jump", 12.0)


if __name__ == "__main__":
    unittest.main()
