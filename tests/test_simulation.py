import unittest
from hexlab.simulation import FOOT_XY, support_margin, convex_hull, Walker, simulate


class TestStaticModel(unittest.TestCase):
    def test_center_in_full_hexagon(self):
        self.assertGreater(support_margin((0,0), FOOT_XY), 0.1)

    def test_center_in_alternating_tripods(self):
        for indices in ((0,2,4),(1,3,5)):
            self.assertGreater(support_margin((0,0), [FOOT_XY[i] for i in indices]), 0.05)

    def test_two_points_cannot_support(self):
        self.assertEqual(support_margin((0,0), FOOT_XY[:2]), float('-inf'))

    def test_3_points_outside(self):
        self.assertLess(support_margin((0.8,0), [FOOT_XY[i] for i in (0,2,4)]), 0)

    def test_decentralized_no_more_than_two_lifted(self):
        walker = Walker('decentralized')
        for _ in range(600):
            walker.advance(0.02)
            self.assertLessEqual(sum(l.swinging for l in walker.legs), 2)
            self.assertGreater(walker.margin(), -1e-9)

    def test_compare_injury(self):
        fixed = simulate('tripod',fail_leg_at=6)
        local = simulate('decentralized',fail_leg_at=6)
        self.assertGreater(fixed['fraction_geometrically_unstable'], 0.05)
        self.assertEqual(local['fraction_geometrically_unstable'], 0.0)


if __name__ == '__main__':
    unittest.main()