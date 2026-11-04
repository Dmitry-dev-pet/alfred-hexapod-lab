"""Pure structure/semantics checks for the public recorded-trajectory export."""
import json
import unittest
from pathlib import Path

DATA = Path(__file__).resolve().parent.parent / 'docs' / 'hexapod' / 'data' / 'tripod-60s.json'


class TestViewerExport(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.doc = json.loads(DATA.read_text(encoding='utf-8'))

    def test_one_continuous_sixty_second_record(self):
        self.assertEqual(self.doc['format'], 'fnk0052-physical-points-v1')
        self.assertEqual(self.doc['sample_count'], 1501)
        self.assertAlmostEqual(self.doc['duration_s'], 60, places=2)
        self.assertAlmostEqual(self.doc['sample_interval_s'], .04, places=3)
        self.assertEqual(len(self.doc['frames']), 1501)

    def test_stride_and_contact_masks(self):
        for f in self.doc['frames']:
            self.assertEqual(len(f), 80)
            self.assertIsInstance(f[-1], int)
            self.assertGreaterEqual(f[-1], 0)
            self.assertLessEqual(f[-1], 63)
            self.assertGreaterEqual(f[-1].bit_count(), 3)

    def test_cad_is_not_published(self):
        viewer = DATA.parent.parent
        for path in viewer.rglob('*'):
            self.assertNotIn(path.suffix.lower(), ('.stl', '.xml', '.obj', '.mjcf'))
        self.assertIn('procedural', self.doc['visualization'])

    def test_movement_is_physical_not_fake_root_scroll(self):
        first, last = self.doc['frames'][0], self.doc['frames'][-1]
        forward = self.doc['forward_xy']
        traveled = (last[0]-first[0])*forward[0]+(last[1]-first[1])*forward[1]
        self.assertGreater(traveled, 1.05)
        self.assertLess(traveled, 1.20)
        self.assertAlmostEqual(traveled, self.doc['summary']['advance_m'], delta=.005)


if __name__ == '__main__':
    unittest.main()
