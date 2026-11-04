"""Deterministic checks for the support-reposition experiment; no fake gait claims."""
import unittest
from rear_foothold_reposition_v1 import run

class TestReposition(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.nominal=run(dx=.04,dy=.20,mass='nominal',mu=1.1,rear_leg='br')

    def test_physics_safe_rear_swap_nominal(self):
        r=self.nominal
        self.assertIsNone(r['fail'])
        self.assertTrue(r['safe_rear_reposition'])
        self.assertEqual(r['min_support_legs_rear_phase'],3)
        self.assertEqual(r['min_rear_swing_support_legs'],3)
        self.assertGreater(r['min_margin_rear_phase_m'],.005)
        self.assertGreater(r['margin_last_m'],.005)
        self.assertGreater(r['rear_max_clearance_m'],.008)
        self.assertEqual(r['saturation_fraction'],0)
        self.assertEqual(r['front_clear_frac'],1)
        self.assertGreater(r['min_rear_mesh_vertex_distance_to_middle_m'],.015)

    def test_reposition_does_not_unload_middle_enough(self):
        self.assertGreater(self.nominal['post_middle_load_N'],1.0)
        self.assertEqual(self.nominal['gate_release_count'],0)
        self.assertEqual(self.nominal['max_sensor_gate_dwell_s'],0)

    def test_shorter_reposition_does_not_build_support_polygon(self):
        r=run(dx=.04,dy=.12,mass='nominal',mu=1.1,rear_leg='br')
        self.assertFalse(r['safe_rear_reposition'])
        self.assertLess(r['margin_last_m'],0)

    def test_low_friction_rejected(self):
        r=run(dx=.04,dy=.20,mass='nominal',mu=.3,rear_leg='br')
        self.assertFalse(r['safe_rear_reposition'])
        self.assertGreater(r['saturation_fraction'],.1)

    def test_mirror_geometry_still_needs_check(self):
        r=run(dx=-.04,dy=.20,mass='nominal',mu=1.1,rear_leg='bl')
        self.assertGreater(r['margin_last_m'],.005)
        self.assertFalse(r['safe_rear_reposition'])
        self.assertLess(r['min_rear_swing_support_legs'],3)

if __name__=='__main__':unittest.main()
