"""Fast integrity tests for real 60-second wave replay and strict negative support gate."""
import unittest,json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
BASE=Path(__file__).resolve().parent
DOCS=ROOT/'docs'
class WaveReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.r=json.loads((BASE/'results'/'wave6_60s_compact.json').read_text())
        cls.p=json.loads((DOCS/'hexapod'/'data'/'wave-60s-points.json').read_text())
        cls.c=json.loads((DOCS/'hexapod'/'data'/'cad-rig-wave-60s.json').read_text())
    def test_duration_and_independent_real_model_physics(self):
        self.assertEqual(self.r['physical_steps'],self.r['simulation_s']/self.r['dt_s']) if 'physical_steps' in self.r else self.assertEqual(self.r['results']['trajectory_frames_25Hz'],1501)
        self.assertAlmostEqual(self.r['results']['advance_m'],0.378953)
    def test_contact_histogram_preserves_negative_controls(self):
        counts=self.r['results']['force_loaded_leg_histogram'];counts={int(k):v for k,v in counts.items()}
        self.assertEqual(sum(counts.values()),30000)
        self.assertEqual(counts.get(2),1)
        self.assertLess(self.r['results']['fraction_of_2ms_ticks_at_least_5_loaded'],1)
    def test_real_pose_arrays(self):
        self.assertEqual(self.p['sample_count'],1501)
        self.assertEqual(len(self.p['frames']),1501)
        self.assertEqual(self.p['stride'],80)
        self.assertTrue(all(len(f)==80 for f in self.p['frames']))
        self.assertNotEqual(self.p['frames'][0][:3],self.p['frames'][-1][:3])
    def test_original_cad_pose_arrays(self):
        self.assertEqual(self.c['frame_stride'],189)
        self.assertEqual(self.c['samples'],1501)
        self.assertTrue(all(len(f)==189 for f in self.c['frames']))
    def test_nominal_model_data_hashes(self):
        self.assertEqual(self.p['source_qpos_sha256'],self.c['source_qpos_sha256'])
        self.assertEqual(self.p['summary']['strict_five_support_pass'],False)
    def test_manifest_integrity_when_registered(self):
        catalog=DOCS/'hexapod'/'library.json'
        cat=json.loads(catalog.read_text()) if catalog.exists() else {'entries':json.loads((ROOT/'new_entries.json').read_text())}
        entry=next((x for x in cat['entries'] if x['id']=='wave-60-nominal'),None)
        self.assertIsNotNone(entry)
        for f in entry['files']:
            path=DOCS/f['path'];b=path.read_bytes()
            self.assertEqual(len(b),f['size'])
            self.assertEqual(hashlib.sha256(b).hexdigest(),f['sha256'])
if __name__=='__main__':unittest.main()
