"""Static validation. Does NOT imply MuJoCo dynamic tests passed."""
import math
import unittest
from xml.etree import ElementTree as ET
from hexlab.mujoco_model import create_model, LINKS, LEGS, SEGMENT_MASSES_KG, TORSO_MASS_KG, FOOT_MASS_KG

class ModelStructureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root=create_model()
    def test_18_joints(self):
        joints=self.root.findall('.//body/joint')
        self.assertEqual(len(joints),18)
        self.assertEqual(len(set(x.attrib['name'] for x in joints)),18)
    def test_18_motors(self):
        motors=self.root.findall('./actuator/position')
        self.assertEqual(len(motors),18)
        self.assertEqual({x.attrib['joint'] for x in motors},{x.attrib['name'] for x in self.root.findall('.//body/joint')})
    def test_6_foot_contacts(self):
        self.assertEqual(len(self.root.findall('./sensor/touch')),6)
        self.assertEqual(len(self.root.findall('.//site[@type="sphere"]')),7) # including COM site
    def test_link_geometry(self):
        leg=LEGS[0][0]
        coxa=self.root.find(f'.//body[@name="{leg}_coxa"]/geom')
        femur=self.root.find(f'.//body[@name="{leg}_femur"]/geom')
        tibia=self.root.find(f'.//body[@name="{leg}_tibia"]/geom')
        for expected, geom in zip(LINKS,[coxa,femur,tibia]):
            values=[float(x) for x in geom.attrib['fromto'].split()]
            self.assertAlmostEqual(math.dist(values[:3],values[3:]),expected,places=6)
    def test_keyframe_dim(self):
        key=self.root.find('./keyframe/key')
        self.assertEqual(len(key.attrib['qpos'].split()),25)
        self.assertEqual(len(key.attrib['ctrl'].split()),18)
    def test_mass_and_limits_are_explicit(self):
        total=TORSO_MASS_KG + len(LEGS)*(sum(SEGMENT_MASSES_KG)+FOOT_MASS_KG)
        self.assertTrue(1.3 < total < 1.6)
        for motor in self.root.findall('./actuator/position'):
            self.assertIn('forcerange',motor.attrib)

if __name__=='__main__': unittest.main()