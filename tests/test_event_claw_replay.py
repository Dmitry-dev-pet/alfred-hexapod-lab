"""Check public Three.js replay against the saved CAD30 physics receipt; no synthesis."""
import hashlib
import json
import pathlib
import unittest

ROOT=pathlib.Path(__file__).resolve().parents[1]
DATA=ROOT/'docs/hexapod/data'

class ClawReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.receipt=json.loads((ROOT/'results/event_claw_crawl_mu1p1.json').read_text())
        cls.points=json.loads((DATA/'claws-10s.json').read_text())
        cls.rig=json.loads((DATA/'cad-rig-claws-10s.json').read_text())
    def test_only_observed_qpos(self):
        samples=self.receipt['qpos_10hz']
        self.assertEqual(len(samples),len(self.points['frames']))
        self.assertEqual(len(samples),len(self.rig['frames']))
        self.assertEqual(len(samples),100)
        self.assertAlmostEqual(self.points['duration_s'],samples[-1]['t']-samples[0]['t'])
    def test_matching_cad_and_schematic_root(self):
        for point,rig in zip(self.points['frames'],self.rig['frames']):
            self.assertEqual(len(point),80)
            self.assertEqual(len(rig),189)
            for a,b in zip(point[:7],rig[:7]):
                self.assertLess(abs(a-b),1e-5)
    def test_actual_contacts_from_trace(self):
        trace=self.receipt['trace_50hz']
        code=self.points['leg_order']; ix=0
        for sample,point in zip(self.receipt['qpos_10hz'],self.points['frames']):
            t=sample['t']
            while ix+1<len(trace) and abs(trace[ix+1]['t']-t)<=abs(trace[ix]['t']-t): ix+=1
            expected=sum(1<<j for j,x in enumerate(code) if x in trace[ix]['contacts'])
            self.assertEqual(point[-1],expected)
    def test_no_false_four_leg_walk(self):
        self.assertFalse(self.receipt['strict_four_leg_walk'])
        self.assertFalse(self.points['summary']['full_four_leg_gait'])
        self.assertEqual(self.points['summary']['new_leg_cycle_coverage'],2)
        self.assertEqual({x['leg'] for x in self.points['summary']['verified_liftoff_touchdowns']},{'bl','br'})
    def test_metadata_cad_upstream(self):
        self.assertEqual(self.rig['mesh_count'],30)
        self.assertEqual(len(self.rig['bodies']),27)
        self.assertEqual(len(self.rig['meshes']),30)
        self.assertTrue(all(x['url'].startswith('https://raw.githubusercontent.com/MattiaGrigoli/') for x in self.rig['meshes']))
        self.assertEqual(self.rig['source_recording_sha256'], hashlib.sha256((ROOT/'results/event_claw_crawl_mu1p1.json').read_bytes()).hexdigest())
    def test_phase_labels_recorded(self):
        self.assertEqual([s['phase'] for s in self.receipt['qpos_10hz']],self.points['phase_samples'])

if __name__=='__main__': unittest.main()
