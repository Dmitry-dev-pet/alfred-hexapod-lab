"""Portable regression checks for CAD30 third-rear-step receipt; not a hardware test."""
import json
import math
import sys
import unittest
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from continuous_third_bl import required_com_shift_for_lift


class MathAndReceiptTests(unittest.TestCase):
    def setUp(self):
        self.r = json.loads((HERE/'results'/'continuous_third_bl_summary.json').read_text())

    def test_inside_triangle_has_no_requested_shift(self):
        xy = {'ml':np.array([-0.1,0.1]),'mr':np.array([0.1,0.1]),
              'bl':np.array([-0.1,-0.1]),'br':np.array([0.1,-0.1])}
        r=required_com_shift_for_lift(xy,np.array([0.04,0.04]),'bl',margin_mm=5.)
        self.assertTrue(r['geometry_pass'])
        self.assertAlmostEqual(r['required_COM_displacement_mm'],0.,places=6)

    def test_unknown_contact_is_not_geometry_pass(self):
        r=required_com_shift_for_lift({'ml':[0,0]},np.array([0.,0.]),'br')
        self.assertFalse(r['geometry_pass'])

    def test_three_real_steps_and_extra_negative_ml(self):
        r=self.r
        self.assertTrue(r['first_ml']['accepted'])
        self.assertTrue(r['second_mr']['accepted'])
        self.assertTrue(r['ml_prep_accepted'])
        self.assertTrue(r['third_bl_accepted'])
        self.assertGreater(r['first_ml']['actual_ml_world_forward_mm'],20)
        self.assertGreater(r['second_mr']['physical_toe_forward_mm'],20)
        self.assertLess(r['additional_ml_backward_prep']['physical_toe_forward_mm'],-70)
        self.assertGreater(r['third_bl']['physical_toe_forward_mm'],20)

    def test_bl_contact_gates_and_provisional_model_limits(self):
        d=self.r['third_bl']
        self.assertEqual(d['trio_bad_force_frames'],0)
        self.assertGreater(d['min_margin_trio_mm'],10)
        self.assertGreater(d['min_fz_trio_N'],0.2)
        self.assertLessEqual(d['max_raw_tau_Nm'],0.7)
        self.assertLessEqual(d['max_leg_joint_rad'],1.7)
        self.assertGreaterEqual(d['last_hold_fraction_four_loaded'],0.99)
        self.assertTrue(self.r['all_phases_same_MjData'])

    def test_still_not_full_forward_gait_and_br_is_unproven(self):
        r=self.r
        self.assertLess(r['net_COM_Y_mm_from_saved_six'],0)
        self.assertFalse(r['repeatable_positive_net_gait'])
        self.assertFalse(r['all_four_legs_swing_forward'])
        self.assertLess(r['br_after_bl_geometry']['initial_signed_margin_mm'],0)
        self.assertGreater(r['br_after_bl_geometry']['required_COM_displacement_mm'],50)

if __name__=='__main__': unittest.main()
