"""Smoke the actual 20s evaluation path with a zero-action placeholder policy."""
import unittest
import numpy as np
from evaluate_sb3 import commanded,evaluate

class DummyPolicy:
    def predict(self,obs,deterministic=True):
        assert obs.shape==(960,)
        return np.zeros(12,dtype=np.float32),None

class EvaluationTests(unittest.TestCase):
    def test_route_uses_all_axes_and_stop(self):
        self.assertEqual(commanded(0),(0,0,0))
        self.assertEqual(commanded(2),(.20,0,0))
        self.assertEqual(commanded(5),(0,.15,0))
        self.assertEqual(commanded(8),(0,0,.35))
        self.assertEqual(commanded(11),(-.15,0,0))
        self.assertEqual(commanded(19),(0,0,0))
    def test_zero_policy_replay_is_measured_and_deterministic(self):
        a=evaluate(DummyPolicy(),seconds=2)
        b=evaluate(DummyPolicy(),seconds=2)
        self.assertEqual(a['summary'],b['summary'])
        self.assertEqual(a['frames'],b['frames'])
        self.assertEqual(a['summary']['physics_steps'],500)
        self.assertEqual(a['summary']['recorded_frames'],50)

if __name__=='__main__':unittest.main()
