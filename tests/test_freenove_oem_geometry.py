"""Auditable effective-joint geometry from OEM Freenove control.py.

No measured part profiles, masses, inertia, servo angle calibration or
contact parameters are asserted here.
"""
from __future__ import annotations
import math
import unittest
from pathlib import Path
from hexlab.mujoco_model import (
    LEGS, LINKS, OEM_BODY_STRUCTURE_MM, OEM_FOOT_XY_MM,
    OEM_NEUTRAL_LOCAL_X_MM, OEM_NEUTRAL_LOCAL_Z_MM,
    FOOT_RADIUS_M, NEUTRAL_ROOT_HEIGHT_M, neutral_leg_angles,
)


class OfficialGeometryTests(unittest.TestCase):
    def test_oem_hip_locations_and_heading(self):
        self.assertEqual(len(LEGS),6)
        for (name, mount, heading), (x, y) in zip(LEGS,OEM_BODY_STRUCTURE_MM):
            self.assertAlmostEqual(mount[0]*1000,x,places=6)
            self.assertAlmostEqual(mount[1]*1000,y,places=6)
            self.assertAlmostEqual(mount[2]*1000,-14,places=6)
            self.assertTrue(name.startswith('leg'))
            projection = (94 if name not in ('leg2','leg5') else 85)
            self.assertAlmostEqual(x,projection*math.cos(math.radians(heading)),delta=0.5)
            self.assertAlmostEqual(y,projection*math.sin(math.radians(heading)),delta=0.5)

    def test_three_segment_lengths(self):
        self.assertEqual(tuple(round(v*1000) for v in LINKS),(33,90,110))

    def test_neutral_leg_ik(self):
        yaw,hip,knee = neutral_leg_angles()
        l1,l2,l3 = LINKS
        r = l1+l2*math.cos(hip)+l3*math.cos(hip+knee)
        z = -(l2*math.sin(hip)+l3*math.sin(hip+knee))
        self.assertAlmostEqual(yaw,0)
        self.assertAlmostEqual(r*1000,OEM_NEUTRAL_LOCAL_X_MM,places=4)
        self.assertAlmostEqual(z*1000,OEM_NEUTRAL_LOCAL_Z_MM,places=4)
        self.assertAlmostEqual(NEUTRAL_ROOT_HEIGHT_M,0.066,places=6)

    def test_manufacturer_neutral_foot_locations(self):
        for (_, (x,y,_z), heading),(tx,ty) in zip(LEGS,OEM_FOOT_XY_MM):
            radial = OEM_NEUTRAL_LOCAL_X_MM / 1000
            predicted_x = (x+radial*math.cos(math.radians(heading)))*1000
            predicted_y = (y+radial*math.sin(math.radians(heading)))*1000
            self.assertLess(math.hypot(predicted_x-tx,predicted_y-ty),0.35)

    @unittest.skipUnless(__import__('importlib').util.find_spec('mujoco'), 'MuJoCo optional')
    def test_mujoco_forward_kinematics_matches_oem(self):
        import mujoco
        model = mujoco.MjModel.from_xml_path(str(Path(__file__).resolve().parents[1] / 'models/freenove_fnk0052_approx.xml'))
        d = mujoco.MjData(model)
        keyid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY,'stand')
        mujoco.mj_resetDataKeyframe(model,d,keyid)
        mujoco.mj_forward(model,d)
        self.assertEqual(model.nu,18)
        for (name,_,_), (tx,ty) in zip(LEGS,OEM_FOOT_XY_MM):
            siteid = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,f'{name}_foot_site')
            actual = d.site_xpos[siteid]
            self.assertLess(math.hypot(actual[0]*1000-tx,actual[1]*1000-ty),0.35)
            self.assertAlmostEqual(actual[2],FOOT_RADIUS_M,places=5)


    @unittest.skipUnless(__import__('importlib').util.find_spec('mujoco'), 'MuJoCo optional')
    def test_stand_has_contacts_and_does_not_fall_for_one_second(self):
        import mujoco
        model = mujoco.MjModel.from_xml_path(str(Path(__file__).resolve().parents[1] / 'models/freenove_fnk0052_approx.xml'))
        d = mujoco.MjData(model)
        keyid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY,'stand')
        mujoco.mj_resetDataKeyframe(model,d,keyid)
        steps = int(1.0 / model.opt.timestep)
        contacts = 0
        for _ in range(steps):
            mujoco.mj_step(model,d)
            contacts += bool(d.ncon)
        self.assertGreater(contacts,steps*0.9)
        self.assertGreater(float(d.qpos[2]),0.050)
        self.assertLess(abs(float(d.qpos[0])),0.010)
        self.assertLess(abs(float(d.qpos[1])),0.010)


if __name__ == '__main__':
    unittest.main()