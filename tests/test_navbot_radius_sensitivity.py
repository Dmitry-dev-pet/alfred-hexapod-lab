import unittest
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'research' / 'navbot_en01'))
import radius_sensitivity as radius

class RadiusSensitivityTests(unittest.TestCase):
    def test_baseline_equivalence(self):
        src = radius.BASE_MODEL.read_text()
        self.assertEqual(radius.variant_xml(src, 24.0), src)

    def test_only_wheel_size_and_root_height_change(self):
        src = radius.BASE_MODEL.read_text()
        modified = radius.variant_xml(src, 18.5)
        self.assertEqual(modified.count('size="0.018500 0.009"'), 2)
        self.assertEqual(modified.count('pos="0 0 0.084500"'), 1)
        self.assertEqual(modified.count('<motor '), 4)
        self.assertIn('ctrlrange="-0.030 0.030"', modified)

    def test_reject_unknown_input(self):
        with self.assertRaises(ValueError):
            radius.variant_xml('<mujoco/>', 18.5)
        with self.assertRaises(ValueError):
            radius.variant_xml(radius.BASE_MODEL.read_text(), -1.0)

if __name__ == '__main__':
    unittest.main()
