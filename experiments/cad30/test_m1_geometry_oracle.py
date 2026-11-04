"""M1/G1 regression checks: geometry alone is NOT physical viability."""
import itertools
import unittest

from m1_geometry_oracle import (
    COM, LEGS, TOES, support_margin, nearest_com_for_lift,
    best_margin_with_fixed_foot_reach, straight_stride_horizon_feasible,
)


class TestM1PoseB(unittest.TestCase):
    def test_original_four_toe_polygon_contains_com(self):
        self.assertGreater(support_margin([TOES[x][:2] for x in LEGS], COM), .025)

    def test_rear_lift_is_geometrically_possible_without_com_shift(self):
        for leg in ("bl", "br"):
            self.assertLess(nearest_com_for_lift(leg)[0], .000001)

    def test_middle_lift_from_original_toe_placement_needs_large_shift(self):
        for leg in ("ml", "mr"):
            shift, target = nearest_com_for_lift(leg)
            self.assertGreater(shift, .090)
            kept = [TOES[x][:2] for x in LEGS if x != leg]
            self.assertGreaterEqual(support_margin(kept, target), .004999)

    def test_frozen_body_middle_lift_upper_bound_has_negative_margin(self):
        # Outer approximation to reach disks: a negative max margin still
        # rules out a positive-margin support in the more restrictive disk model.
        for leg in ("ml", "mr"):
            self.assertLess(best_margin_with_fixed_foot_reach(leg), 0.)

    def test_all_24_fixed_straight_stride_20mm_orders_rejected(self):
        for seq in itertools.permutations(LEGS):
            self.assertFalse(straight_stride_horizon_feasible(seq, stride=.020))


if __name__ == "__main__":
    unittest.main()
