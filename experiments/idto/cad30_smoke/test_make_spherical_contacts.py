import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from make_spherical_contacts import LEG_CODES, rewrite_collisions


class ProxyTests(unittest.TestCase):
    def test_six_spheres_keep_inertial(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'base_local30_nominal.xml'
            root = ET.Element('mujoco')
            for code in LEG_CODES:
                body = ET.SubElement(root, 'body', name='tibia'+code)
                ET.SubElement(body, 'inertial', mass='0.04')
                ET.SubElement(body, 'geom', name='tibia'+code+'_collision_0', type='mesh', mesh='private.obj', **{'class':'collision'})
                ET.SubElement(body, 'geom', name='visual_'+code, type='box', mass='0')
            ET.ElementTree(root).write(path)
            result = rewrite_collisions(path, {code:[0.1, 0.0, -0.01] for code in LEG_CODES})
            out = ET.parse(path)
            self.assertEqual(result['removed_visual_and_auxiliary_geometries'],6)
            self.assertEqual(len(list(out.iter('inertial'))),6)
            self.assertEqual(len(list(out.iter('geom'))),6)
            self.assertTrue(all(g.get('type')=='sphere' for g in out.iter('geom')))

    def test_missing_foot_fails(self):
        with self.assertRaises(ValueError):
            rewrite_collisions(Path('/file/not/used.xml'), {'ml':[0,0,0]})


if __name__ == '__main__':
    unittest.main()