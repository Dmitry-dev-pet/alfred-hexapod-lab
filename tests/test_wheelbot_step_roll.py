"""Offline regression checks for source-pinned STEP audit and original roll input."""
import importlib.util
import math
from pathlib import Path
import sys
import unittest

SCRIPTS = Path(__file__).resolve().parents[1] / "research/mini_wheelbot_v2"
sys.path.insert(0,str(SCRIPTS))
from simulate_step_source_roll import upstream_roll_input, OMEGA_CAP_ASSUMED
from audit_step_axes import parse_cylinders


class SourceRollTests(unittest.TestCase):
    def test_open_loop_sequence_is_piecewise_source_parity(self):
        self.assertEqual(upstream_roll_input(.2),0)
        self.assertEqual(upstream_roll_input(1.),0)
        self.assertAlmostEqual(upstream_roll_input(2.8),-.165,places=6)
        self.assertAlmostEqual(upstream_roll_input(4.6),-.33,places=6)
        self.assertAlmostEqual(upstream_roll_input(4.601),.55,places=6)
        self.assertAlmostEqual(upstream_roll_input(6.1),.55,places=6)
        self.assertEqual(upstream_roll_input(6.101),0)
        self.assertAlmostEqual(upstream_roll_input(4.601,sign=-1),-.55,places=6)

    def test_speed_model_is_explicit_hypothesis(self):
        self.assertAlmostEqual(OMEGA_CAP_ASSUMED, 883.415854189,places=1)

    def test_cylinder_step_parser(self):
        step = """
        #1=CARTESIAN_POINT('',(10.5,-32.5,0.));
        #2=DIRECTION('',(1.,0.,0.));
        #3=DIRECTION('',(0.,1.,0.));
        #4=AXIS2_PLACEMENT_3D('',#1,#2,#3);
        #5=CYLINDRICAL_SURFACE('',#4,15.0);
        """
        surfaces = parse_cylinders(step)
        self.assertEqual(len(surfaces),1)
        self.assertEqual(surfaces[0]["radius_mm"],15.0)
        self.assertEqual(surfaces[0]["axis_point_mm"],(10.5,-32.5,0.))
        self.assertEqual(surfaces[0]["axis_direction"],(1.,0.,0.))


if __name__=="__main__":
    unittest.main()
