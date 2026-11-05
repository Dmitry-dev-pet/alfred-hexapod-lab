"""Offline integrity/regression for recorded CAD30 Wave Gate (not a dynamics proof)."""
import unittest,json,hashlib,gzip
from pathlib import Path

ROOT=Path(__file__).resolve().parent
D=json.loads(((ROOT/'feedback60_final.json') if (ROOT/'feedback60_final.json').exists() else (ROOT/'feedback60_compact.json')).read_text())
VIEW=(ROOT/'viewer_assets'/'docs') if (ROOT/'viewer_assets'/'docs').is_dir() else (ROOT.parents[2]/'docs')
ENTRY=json.loads((ROOT/'viewer_entry.json').read_text())

class GateReceiptTests(unittest.TestCase):
    def test_nominal_continuous_result(self):
        self.assertEqual(D['elapsed_seconds'],60)
        self.assertEqual(sum(map(int,D['loaded_hist'].values())),30000)
        self.assertGreater(D['progress_m'],.3)
        self.assertEqual(D['num_under5_ticks'],0)
        self.assertEqual(D['saturated_ticks'],0)
        self.assertEqual(D['stop_reason'],None)
    def test_no_false_strict_claim(self):
        self.assertFalse(D['strict_5_support_pass'])
        self.assertEqual(D['unplanned_stance_Fz_under_0p2_ticks'],165)
        self.assertEqual(sum(D['unplanned_stance_losses_by_leg'].values()),165)
        self.assertEqual(sum(D['unplanned_stance_loss_mode_counts'].values()),165)
    def test_all_legs_actually_swing(self):
        codes=('ml','bl','br','mr','fr','fl')
        for c in codes:
            self.assertGreater(D['observed_swing_no_contact_ticks_by_leg'][c],2500)
            self.assertGreater(D['observed_swing_2mm_mesh_clearance_sample_frames_by_leg'][c],0)
            self.assertGreater(D['gate_verified_touchdown'][c],10)
    def test_full_2ms_trace(self):
        if not (ROOT/'feedback60_final.2ms.json.gz').exists():
            self.skipTest('Full 2ms binary trace lives on linked Drive archive')
        with gzip.open(ROOT/'feedback60_final.2ms.json.gz','rt') as f:
            r=json.load(f)
        self.assertEqual(len(r['frames']),30000)
        bad=sum(1 for x in r['frames'] if sum(float(v)>=.2 for v in x['Fz'].values())<5)
        self.assertEqual(bad,0)
        self.assertAlmostEqual(r['frames'][-1]['t'],60,places=5)
    def test_player_integrity(self):
        self.assertEqual(ENTRY['status'],'diagnostic')
        self.assertEqual(len(ENTRY['files']),5)
        for f in ENTRY['files']:
            content=(VIEW/f['path']).read_bytes()
            self.assertEqual(len(content),f['size'])
            self.assertEqual(hashlib.sha256(content).hexdigest(),f['sha256'])
        points=json.loads((VIEW/'hexapod/data/wave-gated-60s-points.json').read_text())
        self.assertEqual(points['sample_count'],1501)
        self.assertEqual(points['summary']['unplanned_stance_Fz_under_0p2_ticks'],165)
        self.assertFalse(points['summary']['strict_five_support_pass'])
        self.assertEqual(len(points['frames'][0]),80)

if __name__=='__main__':unittest.main()
