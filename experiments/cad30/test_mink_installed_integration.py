"""Actual Mink solver gate on archived CAD30 pose B (not a dynamics/robot test)."""
from __future__ import annotations
import os
from pathlib import Path
import unittest
from mink_cad30_probe import benchmark

MODEL = Path(os.environ.get('CAD30_MODEL_XML','source/mjcf/scene_local30_nominal.xml'))
POSE = Path(os.environ.get('CAD30_REPLAY_ZIP','FNK0052_claw_tuck_ABC_2026-11-04_v2_no_CAD.zip'))

@unittest.skipUnless(MODEL.is_file() and POSE.is_file(), 'private CAD30 and replay not restored')
class InstalledMinkRealPose(unittest.TestCase):
    def test_mink_solver_is_installed_and_executes(self):
        result = benchmark(MODEL, POSE)
        self.assertEqual(result['mujoco_version'],'3.14.0')
        self.assertEqual(result['mink']['mink_version'],'1.4.0')
        self.assertEqual(result['mink']['status'],'RUN')
        self.assertEqual(result['mink']['solver'],'daqp')
        self.assertTrue(result['mink']['kinematic_gate_pass'])
        self.assertLess(result['mink']['max_foot_drift_mm'],0.002)
        self.assertLess(result['mink']['max_hinge_delta_rad'],0.02)
        self.assertFalse(result['mink']['hardware_approved'])
        self.assertEqual({x['joint'] for x in result['research_1p7rad_violations']}, {'motor31','motor27'})

if __name__=='__main__': unittest.main()