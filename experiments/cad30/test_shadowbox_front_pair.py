"""Verification of the high front-leg air-jab experiment in real MuJoCo dynamics."""
import unittest
import numpy as np
from shadowbox_front_pair import experiment,interpolate_keys

class TrajectoryInterpolationTests(unittest.TestCase):
    def test_smoothstep_at_phase_ends(self):
        stages=[(0.,np.array([0.,0.])),(1.,np.array([1.,2.])),(2.,np.array([0.,1.]))]
        np.testing.assert_allclose(interpolate_keys(-1.,stages),[0.,0.])
        np.testing.assert_allclose(interpolate_keys(.5,stages),[.5,1.])
        np.testing.assert_allclose(interpolate_keys(1.,stages),[1.,2.])
        np.testing.assert_allclose(interpolate_keys(5.,stages),[0.,1.])
    def test_invalid_dimensions_fail(self):
        with self.assertRaises(ValueError):experiment(lift_m=.5)
        with self.assertRaises(ValueError):experiment(mu=.08)

class HighFrontPosePhysicsTests(unittest.TestCase):
    def test_two_high_feet_and_alternating_jabs(self):
        r=experiment(lift_m=.14,guard_forward_m=.05,jab_forward_m=.09,shift_m=.04,mu=1.1)
        self.assertTrue(r['complete'])
        self.assertTrue(r['high_guard_and_air_jabs_valid'])
        self.assertFalse(r['torque_saturation_observed'])
        self.assertGreater(r['min_front_mesh_clearance_during_guard_and_jabs_m'],.09)
        self.assertGreater(r['min_support_COM_margin_during_guard_and_jabs_m'],.015)
        self.assertEqual(r['end_six_contact_restore_fraction'],1.)
        for leg in ('fr','fl'):
            self.assertGreater(r['measured_forward_jab_excursion_m'][leg],.03)
    def test_cannot_balance_same_way_without_weight_shift(self):
        r=experiment(lift_m=.14,guard_forward_m=.05,jab_forward_m=.09,shift_m=0.,mu=1.1)
        self.assertTrue(r['complete'])
        self.assertFalse(r['high_guard_and_air_jabs_contact_valid'])
    def test_slippery_surface_rejected(self):
        r=experiment(lift_m=.14,guard_forward_m=.05,jab_forward_m=.09,shift_m=.04,mu=.3)
        self.assertFalse(r['high_guard_and_air_jabs_valid'])
        self.assertTrue(r['torque_saturation_observed'])
    def test_heavy_weak_traction_torque_limit_is_not_pass(self):
        r=experiment(lift_m=.14,guard_forward_m=.05,jab_forward_m=.09,shift_m=.04,mu=.6,mass='upper')
        self.assertTrue(r['high_guard_and_air_jabs_contact_valid'])
        self.assertTrue(r['torque_saturation_observed'])
        self.assertFalse(r['high_guard_and_air_jabs_valid'])

if __name__=='__main__': unittest.main()
