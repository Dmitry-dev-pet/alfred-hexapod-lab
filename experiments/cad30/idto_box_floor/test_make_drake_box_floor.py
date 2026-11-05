import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from make_drake_box_floor import build_drake_ground_model, sha256


class BoxFloorTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.source = self.root / 'src'
        (self.source / 'mjcf').mkdir(parents=True)
        (self.source / 'meshes').mkdir()
        self.original = self.source / 'mjcf' / 'scene_local30_nominal.xml'
        self.original.write_text('<mujoco><worldbody><geom name="floor" type="plane" pos="0 0 0" friction="1.1 0.005 0.001"/></worldbody></mujoco>')
        self.mesh = self.source / 'meshes' / 'private_placeholder.obj'
        self.mesh.write_text('synthetic placeholder mesh\n')
        self.old_hash = sha256(self.original)

    def tearDown(self):
        self.tmp.cleanup()

    def test_box_floor_keeps_top_height_and_original_files(self):
        out = self.root / 'out'
        r = build_drake_ground_model(self.source, out)
        g = ET.parse(out / 'mjcf' / 'scene_local30_nominal.xml').find('./worldbody/geom')
        self.assertEqual(g.get('type'), 'box')
        self.assertEqual([float(v) for v in g.get('size').split()], [2.0, 2.0, 0.05])
        self.assertEqual([float(v) for v in g.get('pos').split()], [0.0, 0.0, -0.05])
        self.assertEqual(g.get('friction'), '1.1 0.005 0.001')
        self.assertEqual(sha256(self.original), self.old_hash)
        self.assertEqual(sha256(self.mesh), sha256(out/'meshes'/'private_placeholder.obj'))
        self.assertEqual(r['changed_model_files'], ['mjcf/scene_local30_nominal.xml'])
        self.assertEqual(r['floor_original_z_m'], r['floor_box_top_z_m'])

    def test_not_allowed_to_overwrite_or_nest(self):
        out = self.root / 'out'
        out.mkdir()
        with self.assertRaises(FileExistsError):
            build_drake_ground_model(self.source,out)
        with self.assertRaises(ValueError):
            build_drake_ground_model(self.source,self.source/'nested')

    def test_tilted_floor_rejected(self):
        self.original.write_text('<mujoco><worldbody><geom name="floor" type="plane" quat="0.707 0.707 0 0"/></worldbody></mujoco>')
        with self.assertRaises(ValueError):
            build_drake_ground_model(self.source,self.root/'out')
        self.assertFalse((self.root/'out').exists())

    def test_no_plane_rejected(self):
        self.original.write_text('<mujoco><worldbody><geom name="floor" type="box"/></worldbody></mujoco>')
        with self.assertRaises(ValueError):
            build_drake_ground_model(self.source,self.root/'out')

if __name__ == '__main__':
    unittest.main()
