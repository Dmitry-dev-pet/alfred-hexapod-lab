"""Pure stdlib checks; CAD-pose model is explicitly NOT a linkage solution."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / 'research/navbot_en01'
sys.path.insert(0,str(HERE))
from cad_pose_v1 import build, dimensions, EVIDENCE, V0, V1

class CADPoseV1Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.evidence=json.loads(EVIDENCE.read_text(encoding='utf-8'))
    def test_pinned_cad_symmetry(self):
        d=dimensions(self.evidence)
        self.assertAlmostEqual(d['wheel_radius_m'],0.0185)
        self.assertAlmostEqual(d['hip_half_span_m'],0.0296)
        self.assertAlmostEqual(d['tire_half_span_m'],0.03715)
        self.assertAlmostEqual(d['wheel_outer_offset_m'],0.00755)
        self.assertAlmostEqual(d['wheel_dz_m'],-0.036414475612,places=8)
    def test_fail_closed_on_unknown_source(self):
        bad=copy.deepcopy(self.evidence)
        bad['source']['git_blob_sha1']='f'*40
        with self.assertRaises(ValueError):dimensions(bad)
        bad=copy.deepcopy(self.evidence)
        bad['entities']['472']['origin_mm'][1] += 5
        with self.assertRaises(ValueError):dimensions(bad)
        bad=copy.deepcopy(self.evidence)
        bad['measurements']['tire_cylinder_radius_candidates_mm']=[18.5,18.5]
        with self.assertRaises(ValueError):dimensions(bad)
    def test_generate_is_reproducible_and_v0_untouched(self):
        orig=V0.read_bytes()
        with tempfile.TemporaryDirectory() as tmp:
            a=Path(tmp)/'first.xml'
            b=Path(tmp)/'second.xml'
            one=build(EVIDENCE,V0,a)
            two=build(EVIDENCE,V0,b)
            self.assertEqual(a.read_bytes(),b.read_bytes())
            self.assertEqual(a.read_bytes(),V1.read_bytes())
            self.assertEqual(one['model_sha256'],two['model_sha256'])
            self.assertEqual(ET.parse(a).getroot().get('model'),'NavBot_EN01_CAD_POSE_v1_SURROGATE_NOT_LINKAGE')
            tree=ET.parse(a).getroot()
            self.assertEqual(len(tree.findall('./actuator/motor')),4)
            self.assertEqual(len(tree.findall('.//freejoint')),1)
            for name in ('left_wheel_geom','right_wheel_geom'):
                geom=next(x for x in tree.findall('.//geom') if x.get('name')==name)
                self.assertAlmostEqual(float(geom.get('size').split()[0]),0.0185)
        self.assertEqual(orig,V0.read_bytes())

if __name__=='__main__':unittest.main()
