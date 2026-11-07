"""XGO-Rider2 replay contract; no real hardware or training."""
import json
import math
import unittest
from pathlib import Path

DOCS = Path(__file__).resolve().parents[1] / 'docs'
DATA = DOCS / 'hexapod' / 'data'
FILENAMES = {
    'rl': 'rider2-rl-drive-12s.json',
    'linear': 'rider2-linear-drive-12s.json',
    'corner': 'rider2-rl-corner-fail.json'
}
UPSTREAM_SHA = 'f534986353ce3fe451caf0da72ba06b980a59114'


class Rider2ReplayTests(unittest.TestCase):
    def case(self, key):
        return json.loads((DATA / FILENAMES[key]).read_text())

    def test_physical_frames_have_valid_measured_schema(self):
        for label in FILENAMES:
            with self.subTest(case=label):
                r = self.case(label)
                self.assertEqual(r['format'], 'xgorider2-mujoco-physical-replay-v1')
                self.assertEqual(r['upstream_commit'], UPSTREAM_SHA)
                self.assertEqual(r['robot'], 'XGO-Rider2')
                self.assertEqual(r['sample_stride'], 36)
                self.assertEqual(len(r['frames']), r['frame_count'])
                self.assertEqual(r['joint_qpos_indices'],
                                 {'extension12': 7, 'wheel11': 8, 'extension22': 9, 'wheel21': 10})
                self.assertTrue(all(len(f) == 36 and all(math.isfinite(x) for x in f)
                                    for f in r['frames']))
                times = [f[0] for f in r['frames']]
                self.assertTrue(all(b > a for a, b in zip(times, times[1:])))
                self.assertAlmostEqual(times[0], 0., places=8)
                self.assertAlmostEqual(times[-1], r['duration_s'], places=4)
                for f in r['frames']:
                    self.assertAlmostEqual(sum(q*q for q in f[4:8]), 1., delta=0.0002)
                    self.assertGreaterEqual(f[22], 0)
                    self.assertGreaterEqual(f[23], 0)
                    self.assertTrue(f[24] >= 0 and f[25] >= 0)
                self.assertIn('mj_contactForce', r['f_z_definition'])

    def test_matched_ab_and_recorded_failure(self):
        rl, lin, corner = (self.case(k) for k in ('rl', 'linear', 'corner'))
        self.assertEqual(rl['scenario'], 'drive')
        self.assertEqual(rl['scenario'], lin['scenario'])
        self.assertEqual(rl['seed'], lin['seed'])
        self.assertEqual(rl['frame_count'], lin['frame_count'])
        self.assertEqual(rl['duration_s'], lin['duration_s'])
        self.assertEqual(rl['algorithm'], 'native_trained_rl')
        self.assertEqual(lin['algorithm'], 'upstream_linear_policy')
        self.assertFalse(rl['terminated'])
        self.assertFalse(lin['terminated'])
        self.assertEqual(corner['scenario'], 'corner')
        self.assertTrue(corner['terminated'])
        self.assertAlmostEqual(corner['duration_s'], .486, places=3)

    def test_player_lists_all_recordings_and_uses_shell(self):
        js = (DOCS/'hexapod/xgorider2-rl-comparison/app.js').read_text()
        html = (DOCS/'hexapod/xgorider2-rl-comparison/index.html').read_text()
        for filename in FILENAMES.values():
            self.assertIn(filename, js)
        self.assertIn('/hexapod/player-shell.js', html)
        self.assertIn('rider-case', html)
        self.assertIn('ФИЗИЧЕСКИЙ ЭКСПЕРИМЕНТ', html)


if __name__ == '__main__':
    unittest.main()
