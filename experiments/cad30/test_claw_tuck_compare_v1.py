"""Regression tests for the recorded three-pose six-to-four MuJoCo comparison."""
import sys, json, unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'experiments/cad30')]
from claw_tuck_compare_v1 import POSES

class ClawFoldResults(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.r=json.loads((ROOT/'results/claw_tuck_compare_2026-11-04.json').read_text())
        cls.s=json.loads((ROOT/'results/claw_tuck_comparison_compact_2026-11-04.json').read_text())

    def test_all_three_generated(self):
        self.assertEqual(set(self.r),set(POSES))
        self.assertEqual(set(self.s['comparison']),set(POSES))

    def test_six_grounded_start(self):
        for name,r in self.r.items():
            frames=r['qpos_samples_10hz']
            self.assertTrue(len(frames)>=90)
            self.assertEqual(len(frames[0]['contacts']),6)
            self.assertEqual(r['start'],'initial six grounded legs; 1s physical stabilization')

    def test_identical_baseline_until_fold(self):
        before={}
        for name,r in self.r.items():
            before[name]={f['t']:f['qpos'] for f in r['qpos_samples_10hz'] if f['t']<=6.8}
        for name in ['semi_tucked','compact']:
            self.assertEqual(set(before['extended']),set(before[name]))
            for t in before['extended']:
                np.testing.assert_allclose(before['extended'][t],before[name][t],atol=1e-9)

    def test_four_supported_and_no_saturation(self):
        for name,r in self.r.items():
            self.assertTrue(r['strict_pass'],name)
            self.assertEqual(r['min_support_four_after_unload'],4,name)
            self.assertEqual(r['saturated_steps_fraction'],0.0,name)
            self.assertGreater(r['min_support_margin_during_tuck_mm'],0,name)
            self.assertGreater(r['min_front_mesh_height_mm_during_tuck'],0,name)
            self.assertLess(r['max_tilt_during_tuck_deg'],8,name)

    def test_folding_increases_margin_in_nominal(self):
        m=self.s['comparison']
        self.assertGreater(m['semi_tucked']['final_COM_margin_mm'],m['extended']['final_COM_margin_mm'])
        self.assertGreater(m['compact']['final_COM_margin_mm'],m['semi_tucked']['final_COM_margin_mm'])
        self.assertLess(m['compact']['final_joint_margin_to_1p7_deg'],1.0)
        self.assertGreater(m['semi_tucked']['final_joint_margin_to_1p7_deg'],3.0)

    def test_ik_and_collision_proxy_checks(self):
        for name,m in self.s['comparison'].items():
            self.assertGreater(m['min_torso_tibia_proxy_distance_mm'],0,name)
            self.assertGreater(m['min_inter_tibia_proxy_distance_mm'],0,name)
            self.assertTrue(all(x['error_mm']<.1 for x in self.r[name]['ik_targets'].values()))

    def test_public_threejs_data(self):
        for name in POSES:
            p=json.loads((ROOT/f'docs/hexapod/data/fold-{name}.json').read_text())
            rig=json.loads((ROOT/f'docs/hexapod/data/cad-rig-fold-{name}.json').read_text())
            self.assertEqual(p['sample_count'],93)
            self.assertEqual(p['frames'][0][-1],63)
            self.assertEqual(len(p['frames'][0]),80)
            self.assertEqual(len(rig['frames']),93)
            self.assertEqual(len(rig['meshes']),30)
            self.assertFalse(p['summary']['four_leg_gait_verified'])

if __name__=='__main__':unittest.main()
