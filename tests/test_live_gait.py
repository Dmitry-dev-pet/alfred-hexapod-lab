"""Pure gait contract tests: do not claim MuJoCo physics passed."""
import math
import unittest

from hexlab.live_gait import LEGS, LiveGait, Twist


ANCHORS = {
    "ml": (.02, .20), "bl": (-.16, .16), "br": (-.16, -.16),
    "mr": (.02, -.20), "fr": (.16, -.16), "fl": (.16, .16),
}


class TestLiveGait(unittest.TestCase):
    def test_idle_does_not_move(self):
        ctl = LiveGait(ANCHORS)
        for _ in range(100):
            f = ctl.tick(.02)
            self.assertEqual(f.mode, "stand")
            self.assertEqual(f.scheduled_swing, ())
            self.assertTrue(all(p == (0., 0., 0.) for p in f.foot_offsets.values()))

    def test_six_leg_wave_one_scheduled_swing(self):
        ctl = LiveGait(ANCHORS)
        for n in range(140):
            if n % 5 == 0:
                ctl.set_command(Twist(vx=.010))
            f = ctl.tick(.02)
            self.assertEqual(len(f.foot_offsets), 6)
            self.assertLessEqual(len(f.scheduled_swing), 1)
            self.assertTrue(all(math.isfinite(c) for p in f.foot_offsets.values() for c in p))

    def test_tripod_three_scheduled_swings(self):
        ctl = LiveGait(ANCHORS, gait="tripod")
        ctl.set_command(Twist(vx=.010))
        f = ctl.tick(.02)
        self.assertEqual(len(f.scheduled_swing), 3)

    def test_turn_moves_opposite_sides_differently(self):
        ctl = LiveGait(ANCHORS)
        for i in range(140):
            if i % 5 == 0:
                ctl.set_command(Twist(wz=.15))
            f = ctl.tick(.02)
        self.assertGreater(abs(f.foot_offsets["fl"][0] - f.foot_offsets["br"][0]), .001)

    def test_reverse_mid_swing_no_target_jump(self):
        ctl = LiveGait(ANCHORS)
        prior = None
        seen_change = False
        for i in range(210):
            if i % 4 == 0:
                ctl.set_command(Twist(vx=.012 if i < 75 else -.012,
                                      vy=0 if i < 110 else .008))
            f = ctl.tick(.02)
            if i == 75:
                seen_change = True
            if prior is not None:
                for leg in LEGS:
                    for x, y in zip(f.foot_offsets[leg], prior[leg]):
                        self.assertLessEqual(abs(x - y), ctl.max_foot_speed * .02 + 1e-10)
            prior = f.foot_offsets
        self.assertTrue(seen_change)

    def test_command_timeout_and_safe_stop(self):
        ctl = LiveGait(ANCHORS)
        ctl.set_command(Twist(vx=.012))
        modes = []
        for _ in range(300):
            modes.append(ctl.tick(.02).mode)
        self.assertIn("walking", modes)
        self.assertIn("stand", modes)
        self.assertEqual(modes[-1], "stand")
        self.assertEqual(ctl.velocity, Twist())

    def test_invalid_controls(self):
        with self.assertRaises(ValueError):
            Twist(vx=float("nan"))
        with self.assertRaises(ValueError):
            LiveGait({"ml": (0., 0.)})
        ctl = LiveGait(ANCHORS)
        for dt in (0, -.1, .101, float("inf")):
            with self.assertRaises(ValueError):
                ctl.tick(dt)

    def test_clip_and_recover_after_timeout(self):
        ctl = LiveGait(ANCHORS)
        ctl.set_command(Twist(vx=5, vy=-7, wz=9))
        for i in range(65):
            if i % 5 == 0:
                ctl.set_command(Twist(vx=5, vy=-7, wz=9))
            f = ctl.tick(.02)
        self.assertLessEqual(abs(f.body_velocity.vx), .04 + 1e-10)
        self.assertLessEqual(abs(f.body_velocity.wz), .35 + 1e-10)
        for _ in range(240):
            f = ctl.tick(.02)
        self.assertEqual(f.mode, "stand")
        ctl.set_command(Twist(vy=.01))
        self.assertEqual(ctl.tick(.02).mode, "walking")


if __name__ == "__main__":
    unittest.main()
