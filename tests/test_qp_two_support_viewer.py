"""Replay data contract; does not use synthetic contacts or render fake motion."""
import hashlib,json,unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]/'docs/hexapod'
LEG_ORDER=('ml','bl','br','mr','fr','fl')

class PhysicalReplayAssets(unittest.TestCase):
    def test_entries_unique_and_hashes(self):
        catalogue=json.loads((ROOT.parent.parent/'new_entries.json').read_text())
        self.assertEqual(len(catalogue),2)
        for item in catalogue:
            self.assertTrue(item['id'].startswith('support-qp-'))
            self.assertEqual(len(item['files']),4)
            for file in item['files']:
                p=ROOT/file['path'].removeprefix('hexapod/')
                self.assertTrue(p.is_file(),str(p))
                self.assertEqual(p.stat().st_size,file['size'])
                self.assertEqual(hashlib.sha256(p.read_bytes()).hexdigest(),file['sha256'])

    def test_real_qpos_and_observed_contact_masks(self):
        for name,n in [('nominal',191),('slippery',185)]:
            sample=json.loads((ROOT/'data'/f'dynamic-{name}-points.json').read_text())
            cad=json.loads((ROOT/'data'/f'cad-rig-dynamic-{name}.json').read_text())
            self.assertEqual(sample['format'],'fnk0052-physical-points-v1')
            self.assertEqual(cad['format'],'fnk0052-mujoco-cad-visual-replay-v1')
            self.assertEqual((len(sample['frames']),sample['sample_count']), (n,n))
            self.assertEqual(len(cad['frames']),n)
            self.assertEqual(len(cad['meshes']),30)
            self.assertEqual(len(cad['bodies']),27)
            self.assertTrue(all(len(f)==80 for f in sample['frames']))
            self.assertTrue(all(len(f)==189 for f in cad['frames']))
            self.assertEqual(sample['summary']['was_walk'],False)
            for f,row in zip(sample['frames'],sample['support_samples']):
                mask=sum(1<<i for i,k in enumerate(LEG_ORDER) if k in row['loaded'])
                self.assertEqual(f[-1],mask)
                self.assertEqual(row['contacts'],len(row['loaded']))
                for leg in ('ml','mr','bl','br'):
                    self.assertEqual(row['fz_N'][leg]>.08,leg in row['loaded'])
            self.assertAlmostEqual(sample['duration_s'],cad['duration_s'])

    def test_nominal_not_mislabeled_as_forward_gait(self):
        good=json.loads((ROOT/'data/dynamic-nominal-points.json').read_text())
        bad=json.loads((ROOT/'data/dynamic-slippery-points.json').read_text())
        self.assertTrue(good['summary']['last_0p2s_four_Fz_ge_0p2N'])
        self.assertGreater(good['summary']['actual_max_true_two_support_contiguous_s'],.2)
        self.assertGreater(sum(r['genuine_two_support'] for r in good['support_samples']),5)
        self.assertFalse(good['summary']['was_walk'])
        self.assertFalse(bad['summary']['last_0p2s_four_Fz_ge_0p2N'])
        self.assertLess(bad['summary']['actual_max_true_two_support_contiguous_s'],.01)
        self.assertFalse(bad['summary']['was_walk'])

if __name__=='__main__':unittest.main()
