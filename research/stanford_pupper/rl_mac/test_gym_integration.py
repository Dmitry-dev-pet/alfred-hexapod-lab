"""Requires actual Gymnasium; run after Mac dependency installation."""
import unittest
try:
    from gymnasium.utils.env_checker import check_env
    from gym_env import PupperVelocityGym
    GYM_AVAILABLE=True
except ModuleNotFoundError:
    GYM_AVAILABLE=False

@unittest.skipUnless(GYM_AVAILABLE,'gymnasium not installed')
class GymIntegrationTests(unittest.TestCase):
    def test_env_checker(self):
        env=PupperVelocityGym(seed=20261107,max_seconds=2)
        try:check_env(env)
        finally:env.close()
    def test_episodes_have_nonzero_rewards(self):
        env=PupperVelocityGym(seed=20261107,max_seconds=2)
        try:
            obs,info=env.reset(seed=20261107)
            import numpy as np
            for _ in range(4):
                obs,reward,done,truncated,info=env.step(np.zeros(12,dtype=np.float32))
                self.assertTrue(np.isfinite(reward))
                self.assertEqual(obs.shape,(960,))
        finally:env.close()

if __name__=='__main__':unittest.main()
