import gzip
import json
import math
import tempfile
import unittest
from pathlib import Path

from locked_command_dt import CONTROL_DT, read_frames, triangle_margin, sha256


class LockedReplayContractTests(unittest.TestCase):
    def make_trace(self, path, times):
        with gzip.open(path, 'wt') as out:
            json.dump({'frames': [{'time_s': t, 'motor_goal': [0] * 20}
                                  for t in times]}, out)

    def test_uniform_trace(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'trace.json.gz'
            self.make_trace(p, [1.002, 1.004, 1.006])
            self.assertEqual(len(read_frames(p)), 3)
            self.assertEqual(CONTROL_DT, .002)
            self.assertEqual(len(sha256(p)), 64)

    def test_bad_trace_cadence_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'trace.json.gz'
            self.make_trace(p, [0.002, 0.004, 0.007])
            with self.assertRaisesRegex(ValueError, 'uniform 2ms'):
                read_frames(p)

    def test_missing_motor_goals_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'trace.json.gz'
            self.make_trace(p, [0.002, 0.004])
            with gzip.open(p, 'wt') as out:
                json.dump({'frames': [{'time_s': 0.002, 'motor_goal': [0] * 19}]}, out)
            with self.assertRaisesRegex(ValueError, '20 motor goals'):
                read_frames(p)

    def test_triangle_margin(self):
        pts = {'ml': (0, 0), 'bl': (0.1, 0), 'br': (0, 0.1)}
        self.assertAlmostEqual(triangle_margin(pts, (.02, .02, 0)), 20., places=7)
        self.assertLess(triangle_margin(pts, (.09, .09, 0)), 0)
        self.assertTrue(math.isinf(triangle_margin({'ml': (0, 0)}, (.02, .02, 0))))

    def test_degenerate_triangle(self):
        pts = {'ml': (0, 0), 'bl': (.1, 0), 'br': (.2, 0)}
        self.assertEqual(triangle_margin(pts, (.02, .02, 0)), float('-inf'))


if __name__ == '__main__':
    unittest.main()