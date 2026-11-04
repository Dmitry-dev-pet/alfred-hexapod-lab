"""Guard against fake CAD positions and wrong mesh quaternions."""
import json
import pathlib
import unittest
import sys
import zipfile

import numpy as np
import mujoco
import trimesh
from scipy.spatial.transform import Rotation

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from export_cad_rig_replay import load_recording, export, FORMAT

MODEL = pathlib.Path('/mnt/data/hexapod-run/model/source/mjcf/scene_local30_nominal.xml')
SOURCE_MESHES = pathlib.Path('/mnt/data/hexapod-run/model/source/meshes')
ARCHIVE = pathlib.Path('/mnt/data/HexapodLab_Tripod_60s_2026-11-04_no_CAD.zip')
EXPORT = ROOT / 'docs/hexapod/data/cad-rig-60s.json'


class CADReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.m = mujoco.MjModel.from_xml_path(str(MODEL))
        cls.d = mujoco.MjData(cls.m)
        cls.recording, cls.hash = load_recording(ARCHIVE)
        cls.export = json.loads(EXPORT.read_text())

    def test_full_physical_recording_and_provenance(self):
        a = self.export
        self.assertEqual(a['format'], FORMAT)
        self.assertEqual(a['source_qpos_sha256'],self.hash)
        self.assertEqual(len(a['frames']), 1501)
        self.assertEqual(len(a['bodies']),27)
        self.assertEqual(len(a['meshes']),30)
        self.assertEqual(len(a['geoms']),146)
        self.assertEqual(a['frame_stride'], 189)
        self.assertEqual({x['filename'] for x in a['meshes']},{p.name for p in SOURCE_MESHES.glob('*.stl')})
        self.assertTrue(all('raw.githubusercontent.com/MattiaGrigoli/autonomous_hexapod/5c9a84c' in x['url'] for x in a['meshes']))
        self.assertEqual(a['missing_meshes'],['topplate.stl','bigservo1.stl'])

    def test_body_world_poses_agree_with_mujoco_at_multiple_times(self):
        for idx in [0,1,249,750,1500]:
            sample=self.recording['qpos_samples'][idx]
            self.d.qpos[:]=np.array(sample['qpos'],dtype=float)
            self.d.qvel[:]=0
            mujoco.mj_forward(self.m,self.d)
            frame=self.export['frames'][idx]
            for i,b in enumerate(self.export['bodies']):
                actual=np.r_[self.d.xpos[b['id']],self.d.xquat[b['id']]]
                recorded=np.asarray(frame[7*i:7*i+7])
                self.assertTrue(np.allclose(actual,recorded,atol=1e-6), (idx,b['name']))

    def test_original_stl_transforms_reproduce_mujoco_compiled_vertices(self):
        # Draw 5 meshes with very different rotation conventions (including tibia).
        picks=['bottomplate.stl','acoplecircular.stl','femur.stl','tibia.stl','ultrasonic_hc_sr04__ultrasonic_hc_sr04_ultrasonic_1_solid2.stl']
        by_name={v['filename']:v for v in self.export['meshes']}
        for name in picks:
            asset=by_name[name]
            raw=trimesh.load(SOURCE_MESHES / name,process=False).vertices
            r=Rotation.from_quat([*asset['mesh_quat_wxyz'][1:],asset['mesh_quat_wxyz'][0]])
            corrected=r.inv().apply(raw-np.asarray(asset['mesh_pos']))
            mesh_id=asset['id']
            compiled=self.m.mesh_vert[self.m.mesh_vertadr[mesh_id]:self.m.mesh_vertadr[mesh_id]+self.m.mesh_vertnum[mesh_id]]
            from scipy.spatial import cKDTree
            distance=cKDTree(compiled).query(corrected,k=1)[0]
            self.assertLess(distance.max(),1e-7,name)

    def test_visuals_reference_moving_bodies_and_not_public_binary(self):
        ids={b['id'] for b in self.export['bodies']}
        m_ids={m['id'] for m in self.export['meshes']}
        for g in self.export['geoms']:
            self.assertIn(g['body'],ids)
            if g['kind']=='mesh':self.assertIn(g['mesh'],m_ids)
            self.assertEqual(len(g['pos']),3)
            self.assertEqual(len(g['quat_wxyz']),4)
            self.assertEqual(len(g['rgba']),4)
        self.assertFalse(list((ROOT / 'docs').rglob('*.stl')))


if __name__=='__main__':unittest.main()