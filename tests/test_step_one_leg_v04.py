"""Pure-Python checks of one-leg swing schedule. Physics is tested in MuJoCo CI."""
import unittest

from hexlab.step_one_leg_v04 import (
    STEP_FORWARD_M, STEP_LIFT_M, LIFT_SECONDS, SWING_SECONDS,
    LAND_SECONDS, RECOVER_SECONDS, phases, smoothstep
)


class FirstStepScheduleTests(unittest.TestCase):
    def test_smoothstep_clamp_and_end_derivatives(self):
        self.assertEqual(smoothstep(-10),0)
        self.assertEqual(smoothstep(0),0)
        self.assertEqual(smoothstep(1),1)
        self.assertEqual(smoothstep(10),1)
        self.assertAlmostEqual(smoothstep(.5),.5)
        eps=1e-4
        self.assertLess((smoothstep(eps)-smoothstep(0))/eps, .001)
        self.assertLess((smoothstep(1)-smoothstep(1-eps))/eps, .001)

    def test_phase_sequence(self):
        self.assertEqual(phases(0)[0],"lift")
        self.assertEqual(phases(LIFT_SECONDS)[0],"swing")
        self.assertEqual(phases(LIFT_SECONDS+SWING_SECONDS)[0],"lower")
        self.assertEqual(phases(LIFT_SECONDS+SWING_SECONDS+LAND_SECONDS)[0],"recover")
        self.assertEqual(phases(1e4),("recover",1.))

    def test_forward_motion_and_lift_nonzero(self):
        self.assertGreater(STEP_FORWARD_M,.015)
        self.assertLess(STEP_FORWARD_M,.050)
        self.assertGreater(STEP_LIFT_M,.015)
        self.assertLess(STEP_LIFT_M,.050)
        self.assertGreater(RECOVER_SECONDS,.5)

    def test_negative_phase_time_rejected(self):
        with self.assertRaises(ValueError):
            phases(-.1)

if __name__ == "__main__":
    unittest.main()
