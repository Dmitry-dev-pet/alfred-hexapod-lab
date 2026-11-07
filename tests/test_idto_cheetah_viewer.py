"""Grounded asset contracts for Mini Cheetah measured 3D replay."""
import json
import unittest
from pathlib import Path

DATA = Path(__file__).resolve().parents[1] / 'docs' / 'hexapod' / 'data' / 'cheetah-idto-10s.json'


class CheetahReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data=json.loads(DATA.read_text())

    def test_recording_source(self):
        d=self.data
        self.assertEqual(d['format'],'mini-cheetah-idto-physical-replay-v1')
        self.assertEqual(d['sample_count'],201)
        self.assertEqual(d['sample_interval_s'],0.05)
        self.assertEqual(d['duration_s'],10)
        self.assertIn('de0629c7811aa9',d['source']['idto_commit'])

    def test_four_feet_with_measured_loads(self):
        d=self.data
        frames=d['frames'];self.assertEqual(len(frames),201)
        self.assertTrue(all(len(f)==60 for f in frames))
        counts={str(x):sum(sum(f[55+i]>.2 for i in range(4))==x for f in frames) for x in range(5)}
        self.assertEqual(counts['2'],171)
        self.assertEqual(counts['3'],29)
        self.assertEqual(counts['4'],1)
        self.assertEqual(counts,d['summary']['contact_counts'])

    def test_real_motion_not_static_illustration(self):
        f=self.data['frames']
        self.assertAlmostEqual(f[0][0],0,places=5)
        self.assertAlmostEqual(f[-1][0]-f[0][0],3.466846,places=5)
        self.assertAlmostEqual(f[-1][2],.29,delta=.06)
        for leg in range(4):
            self.assertGreater(f[0][7+12*leg+11],-.03)
            self.assertLess(f[0][7+12*leg+11],.06)
        self.assertGreater(len(set(round(x[0],3) for x in f)),150)

if __name__=='__main__':unittest.main()
