"""Static integrity checks only; deliberately does not claim a MuJoCo physics run."""
import math
import pathlib
import unittest
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parents[1]
XML = ROOT / "research/navbot_en01/model_v0.xml"


class NavBotEN01StaticModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tree = ET.parse(XML)
        cls.root = cls.tree.getroot()

    def test_four_actuators_only(self):
        motors = self.root.findall("./actuator/motor")
        self.assertEqual(len(motors), 4)
        self.assertEqual({m.get("joint") for m in motors},
                         {"hip_left", "hip_right", "wheel_left", "wheel_right"})
        self.assertEqual(len(self.root.findall(".//freejoint")), 1)

    def test_parts_sum_to_published_total_mass(self):
        masses = [float(g.attrib["mass"]) for g in self.root.findall(".//geom")
                  if "mass" in g.attrib]
        self.assertEqual(len(masses), 5)
        self.assertAlmostEqual(sum(masses), 0.322, places=6)
        self.assertAlmostEqual(sorted(masses)[-1], 0.180, places=6)

    def test_limits_and_wheel_radius(self):
        motors = {m.get("joint"): m for m in self.root.findall("./actuator/motor")}
        for n in ("hip_left", "hip_right"):
            limits = [float(x) for x in motors[n].get("ctrlrange").split()]
            self.assertAlmostEqual(limits[1], 4.5 * 0.0980665, delta=0.0001)
            self.assertAlmostEqual(limits[0], -limits[1])
        for n in ("wheel_left", "wheel_right"):
            self.assertEqual(motors[n].get("ctrlrange"), "-0.030 0.030")
        for n in ("left_wheel_geom", "right_wheel_geom"):
            g = next(g for g in self.root.findall(".//geom") if g.get("name") == n)
            self.assertAlmostEqual(float(g.get("size").split()[0]), 0.024)

    def test_sensor_and_source_are_explicit(self):
        self.assertEqual(len(self.root.findall("./sensor/gyro")), 1)
        notes = (XML.parent / "MODEL_NOTES.md").read_text(encoding="utf-8")
        self.assertIn("3139f2fb32b38d10611feb905e16f8aa012f2f2e", notes)
        self.assertIn("CPU PHYSICS EXERCISED", notes)
        self.assertIn("UNVERIFIED", notes)


if __name__ == "__main__":
    unittest.main()
