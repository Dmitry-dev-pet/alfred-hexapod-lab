import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from research.navbot_en01.servo_mapping import normal_positions,jump_positions,cad_angle,logical_angle_since_squat
from research.navbot_en01.feedback_model import make_model
from research.navbot_en01.linkage_model import create

HAS_PHYSICS=all(importlib.util.find_spec(n) is not None for n in ('mujoco','numpy'))


class SourceMappingTests(unittest.TestCase):
    def test_normal_position_clamps_and_integer_conversion(self):
        self.assertEqual(normal_positions(38),(2110,1986))
        self.assertEqual(normal_positions(40),(2115,1980))
        self.assertEqual(normal_positions(80),(2451,1644))
        self.assertEqual(normal_positions(-100),(2110,1986))
        self.assertEqual(normal_positions(1000),(2510,1586))

    def test_jump_integer_mirror_asymmetry(self):
        self.assertEqual(jump_positions('initial'),(2463,1632))
        self.assertEqual(jump_positions('return'),(2127,1968))

    def test_no_unmeasured_absolute_cad_zero(self):
        with self.assertRaises(ValueError):cad_angle(2110,1)
        self.assertEqual(logical_angle_since_squat(2048,1),0)
        self.assertAlmostEqual(cad_angle(2110,1,squat_cad_angle_rad=-.2),-.2+logical_angle_since_squat(2110,1))

    def test_nominal_model_bytes_frozen_and_profile_explicit(self):
        self.assertEqual(make_model(),create())
        with self.assertRaises(ValueError):make_model('unknown')

    @unittest.skipUnless(HAS_PHYSICS,'MuJoCo/numpy required in research runtime')
    def test_profile_mass_is_declared_sensitivity(self):
        import mujoco
        for p,expected in [('nominal',.322),('mass-090',.2898),('mass-110',.3542)]:
            m=mujoco.MjModel.from_xml_string(make_model(p))
            self.assertAlmostEqual(float(sum(m.body_mass)),expected,places=10)

    @unittest.skipUnless(HAS_PHYSICS,'MuJoCo/numpy required in research runtime')
    def test_lateral_drop_latches_wheel_cut(self):
        from research.navbot_en01.feedback_probe import run
        with tempfile.TemporaryDirectory() as t:
            p=Path(t)/'raw.json';s=run('fall',p,duration=.04,position_gain=.3,gated=True)
            self.assertEqual(s['safety_wheel_cut_steps'],40)
            for frame in json.loads(p.read_text())['frames']:
                self.assertEqual(frame['ctrl'][1],0)
                self.assertEqual(frame['ctrl'][3],0)


if __name__=='__main__':unittest.main()
