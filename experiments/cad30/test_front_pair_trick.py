"""Deterministic verification of the first two-front-leg balance trick.

Run after restoring pinned Freenove CAD30 private source/ assets and MuJoCo.
"""
import unittest
import numpy as np
from front_pair_trick import margin,experiment

class ConvexSupportTests(unittest.TestCase):
    def test_inside_and_outside(self):
        feet=np.array([[-.1,-.1],[.1,-.1],[.1,.1],[-.1,.1]])
        self.assertAlmostEqual(margin((0,0),feet),.1)
        self.assertLess(margin((0,.13),feet),0)
    def test_insufficient_support(self):
        self.assertIsNone(margin((0,0),[(0,0),(1,0)]))

class FirstTrickValidation(unittest.TestCase):
    def test_nominal_4_leg_support_is_real_contact(self):
        r=experiment(shift_m=.04,lift_m=.035,scenario='nominal',friction=1.1)
        self.assertTrue(r['complete'])
        self.assertTrue(r['front_pair_clear_four_leg_balance_hold'])
        self.assertGreater(r['hold_front_min_mesh_height_m'],.004)
        self.assertGreater(r['hold_min_margin_m'],0)
        self.assertLess(r['saturation_step_fraction'],.001)
    def test_without_shift_lift_fails(self):
        r=experiment(shift_m=0,lift_m=.035,scenario='nominal',friction=1.1)
        self.assertFalse(r['front_pair_clear_four_leg_balance_hold'])
    def test_slippery_result_remains_negative(self):
        r=experiment(shift_m=.04,lift_m=.035,scenario='nominal',friction=.3)
        self.assertFalse(r['front_pair_clear_four_leg_balance_hold'])

if __name__=='__main__': unittest.main()
