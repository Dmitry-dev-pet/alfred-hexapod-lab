"""Data integrity and outcome regression for measured CAD30 Issue41 replays.

Requires no private CAD: it checks exported physical data shapes, metadata,
manifest hashes, and exact negative/positive classification.
"""
from __future__ import annotations
import hashlib
import json
import unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

class Issue41ViewerAssets(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        library=ROOT/'docs/hexapod/library.json'
        if library.is_file():
            cls.entries=[e for e in json.loads(library.read_text())['entries'] if e['id'].startswith('issue41-')]
        else:
            cls.entries=json.loads((ROOT/'new_entries.json').read_text())

    def test_manifest_contains_five_unique_immutable_records(self):
        self.assertEqual(len(self.entries),5)
        self.assertEqual(len(set(e['id'] for e in self.entries)),5)
        self.assertEqual(len(set(e['href'] for e in self.entries)),5)
        for entry in self.entries:
            self.assertEqual(len(entry['files']),4)
            self.assertTrue(entry['href'].startswith('/hexapod/issue41-'))
            for spec in entry['files']:
                path=ROOT/'docs'/spec['path'];data=path.read_bytes()
                self.assertEqual(len(data),spec['size'])
                self.assertEqual(hashlib.sha256(data).hexdigest(),spec['sha256'])

    def test_real_qpos_poses_and_cad_frames(self):
        for e in self.entries:
            v=e['id'].replace('issue41-','')
            points=json.loads((ROOT/f'docs/hexapod/data/issue41-{v}-points.json').read_text())
            cad=json.loads((ROOT/f'docs/hexapod/data/cad-rig-issue41-{v}.json').read_text())
            self.assertEqual(points['study'],'issue41-rear-pair-forward-partial')
            self.assertEqual(points['format'],'fnk0052-physical-points-v1')
            self.assertEqual(points['case_id'],v)
            self.assertEqual(points['stride'],80)
            self.assertGreaterEqual(points['sample_count'],150)
            self.assertEqual(points['sample_count'],len(points['frames']))
            self.assertTrue(all(len(x)==80 for x in points['frames']))
            self.assertEqual(len(points['support_samples']),points['sample_count'])
            self.assertEqual(len(points['phase_samples']),points['sample_count'])
            self.assertEqual(cad['format'],'fnk0052-mujoco-cad-visual-replay-v1')
            self.assertEqual(len(cad['meshes']),30)
            self.assertEqual(len(cad['bodies']),27)
            self.assertEqual(len(cad['frames']),points['sample_count'])
            self.assertEqual(cad['frame_stride'],189)
            self.assertTrue(all(len(x)==189 for x in cad['frames']))
            self.assertTrue(points['recording_sha256'])

    def test_fz_based_contacts_not_drawn_from_commands(self):
        for e in self.entries:
            v=e['id'].replace('issue41-','')
            pts=json.loads((ROOT/f'docs/hexapod/data/issue41-{v}-points.json').read_text())
            for frame,s in zip(pts['frames'],pts['support_samples']):
                loaded=[x for x,fz in s['fz_N'].items() if fz>.08]
                self.assertEqual(set(loaded),set(s['loaded_stance']))
                self.assertEqual(len(loaded),s['contacts'])
                for bit,name in enumerate(pts['leg_order']):
                    if name in s['fz_N']:
                        self.assertEqual(bool(int(frame[79]) & 1<<bit),s['fz_N'][name]>.08)

    def test_partial_step_not_falsely_called_completed_gait(self):
        r10=json.loads((ROOT/'docs/hexapod/data/issue41-rear10-points.json').read_text())
        self.assertEqual(r10['summary']['status'],'PARTIAL_REAR_PAIR_FORWARD_STEP_LT20MM')
        self.assertLess(r10['summary']['net_forward_COM_mm'],20)
        self.assertGreater(r10['summary']['actual_max_two_support_contiguous_s'],.15)
        self.assertGreater(min(r10['summary']['actual_rear_final_forward_mm'].values()),5)
        for v in ['rear5','mu08','mu06','mu03']:
            r=json.loads((ROOT/f'docs/hexapod/data/issue41-{v}-points.json').read_text())
            self.assertEqual(r['summary']['status'],'RESEARCH_GATE_FAILED')
        self.assertTrue(all(e['status']=='negative' for e in self.entries if e['id']!='issue41-rear10'))

if __name__=='__main__':unittest.main()
