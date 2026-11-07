"""Fast unit tests of nonzero reward design without Gym or SB3 installs."""
import unittest
import numpy as np
from reward import REWARD_PROFILES,calculate_reward


class RewardDesignTests(unittest.TestCase):
    def given(self, **kw):
        s={'vel_xy':np.array([.2,0]),'yaw_rate':0.,'tilt_rad':0.,
           'height_m':.155,'delta_action_l2':0.,'torque_norm':0.,
           'vertical_rate':0.,'fell':False}
        s.update(kw);return s

    def test_reward_is_not_all_zero(self):
        for name,weights in REWARD_PROFILES.items():
            self.assertGreater(weights['linear_xy'],0.)
            self.assertGreater(weights['yaw'],0.)
            self.assertLess(weights['fall'],0.)

    def test_matched_velocity_outscores_no_motion(self):
        for profile in REWARD_PROFILES:
            good,_,_,_=calculate_reward(self.given(),(.2,0,0),profile)
            bad,_,_,_=calculate_reward(self.given(vel_xy=np.zeros(2)),(.2,0,0),profile)
            self.assertGreater(good,bad)

    def test_yaw_rate_tracking_has_separate_objective(self):
        good,_,_,_=calculate_reward(self.given(yaw_rate=.45),(0,0,.45),'tracking')
        bad,_,_,_=calculate_reward(self.given(yaw_rate=0),(0,0,.45),'tracking')
        self.assertGreater(good,bad)

    def test_fall_penalty_is_real(self):
        good,_,_,_=calculate_reward(self.given(),(.2,0,0),'tracking')
        bad,_,_,_=calculate_reward(self.given(fell=True),(.2,0,0),'tracking')
        self.assertAlmostEqual(good-bad,10.,places=7)

    def test_penalizes_actuator_and_action_jerk(self):
        good,_,_,_=calculate_reward(self.given(),(.2,0,0),'stability')
        bad,_,_,_=calculate_reward(self.given(torque_norm=1.,delta_action_l2=1.),(.2,0,0),'stability')
        self.assertGreater(good,bad)

    def test_nonfinite_fails_closed(self):
        with self.assertRaises(FloatingPointError):
            calculate_reward(self.given(yaw_rate=np.nan),(.2,0,0),'tracking')

if __name__=='__main__':unittest.main()
