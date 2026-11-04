"""Regress a genuine three-support swing with two front legs raised.

The robot is NOT established to walk on four legs. This tests only a single
short, rear-leg swing and a known-negative middle-leg control.
"""
import unittest
from probe_four_leg_rear_step import run

class FourLegStepProbe(unittest.TestCase):
    def test_rear_bl_swing_three_supports(self):
        r = run('bl', step_m=.01, lift_m=.01, shift=.02, mu=1.1)
        self.assertTrue(r['complete'])
        self.assertTrue(r['all_front_clear'])
        self.assertEqual(r['under_3_contacts'], 0)
        self.assertGreater(r['step_clearance_max'], .005)
        self.assertGreater(r['min_margin'], .004)
        self.assertEqual(r['saturation_frac'], 0.)
        # This is toe movement, NOT a claim of forward body locomotion.
        a=min(r['trace'], key=lambda x:abs(x['t']-2.9))['stepping_foot_y']
        b=min(r['trace'], key=lambda x:abs(x['t']-6.3))['stepping_foot_y']
        self.assertGreater(b-a, .008)

    def test_middle_ml_does_not_pass_same_control(self):
        r = run('ml', step_m=.01, lift_m=.01, shift=.02, mu=1.1)
        self.assertTrue(r['complete'])
        self.assertTrue(r['all_front_clear'])
        self.assertTrue(r['under_3_contacts']>0 or r['step_clearance_max']<=.001)

    def test_low_friction_negative(self):
        r = run('bl', step_m=.01, lift_m=.01, shift=.02, mu=.3)
        self.assertTrue(r['complete'])
        self.assertGreater(r['saturation_frac'], 0.)
        self.assertFalse(r['all_front_clear'])

if __name__=='__main__': unittest.main()
