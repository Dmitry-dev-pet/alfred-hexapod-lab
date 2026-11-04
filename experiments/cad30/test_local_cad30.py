"""Local checks for reproducible original CAD 30/32-mesh MuJoCo exercise."""
import json,math,unittest
from pathlib import Path
import mujoco
from hexlab.cad_mujoco_v04 import git_blob_sha
ROOT=Path(__file__).resolve().parents[2]
class TestLocalCAD30(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lock=json.loads((ROOT/'data/fnk0052_cad_assets_lock.json').read_text())

    def test_30_asset_hashes_and_exactly_two_visual_absences(self):
        missing=[];count=0
        for x in self.lock['meshes']:
            p=ROOT/'source'/'meshes'/x['name']
            if not p.exists():missing.append(x['name']);continue
            self.assertEqual(p.stat().st_size,x['size_bytes'])
            self.assertEqual(git_blob_sha(p.read_bytes()),x['sha_git_blob'])
            count+=1
        self.assertEqual(count,30)
        self.assertEqual(set(missing),{'bigservo1.stl','topplate.stl'})

    def test_original_source_xml_sha(self):
        for path,info in self.lock['mjcf'].items():
            p=ROOT/'source'/path
            self.assertEqual(git_blob_sha(p.read_bytes()),info['sha_git_blob'])

    def test_mass_scenarios_load_with_true_tibia_hulls(self):
        for scenario,mass in [('lower',1.644),('nominal',1.804868),('upper',2.08)]:
            with self.subTest(scenario=scenario):
                m=mujoco.MjModel.from_xml_path(str(ROOT/'source/mjcf'/f'scene_local30_{scenario}.xml'))
                self.assertEqual(m.nmesh,30)
                self.assertEqual(m.nu,20)
                self.assertAlmostEqual(float(m.body_mass.sum()),mass,places=5)
                for s in ('ml','bl','br','mr','fr','fl'):
                    gid=m.geom(f'tibia{s}_collision_0').id
                    self.assertEqual(m.geom_type[gid],mujoco.mjtGeom.mjGEOM_MESH)

    def test_six_reproduced_gait_sims_and_servo_limit(self):
        for scenario in ('lower','nominal','upper'):
            for gait in ('tripod','wave'):
                with self.subTest(scenario=scenario,gait=gait):
                    doc=json.loads((ROOT/'results'/f'cad30_gait_{gait}_{scenario}.json').read_text())
                    self.assertTrue(doc['finished_full_duration'])
                    self.assertFalse(doc['fall_or_nonfinite'])
                    self.assertFalse(doc['body_was_actuated'])
                    self.assertFalse(doc['feet_were_pinned'])
                    self.assertEqual(doc['actual_tibia_collision_meshes'],6)
                    self.assertEqual(doc['verified_original_meshes'],30)
                    self.assertTrue(.05<doc['advance_m']<.3)
                    self.assertLessEqual(doc['max_abs_torque_nm'],.7)
                    self.assertGreater(doc['min_body_height_m'],.12)
                    self.assertLess(doc['max_tilt_deg'],2.)
                    self.assertTrue(all(math.isfinite(row['advance_m']) for row in doc['trace']))
if __name__=='__main__':unittest.main()
