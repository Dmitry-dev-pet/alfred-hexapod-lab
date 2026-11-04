"""Real CAD30 checks; skip safely when private binary source is not restored."""
from __future__ import annotations
import os
from pathlib import Path
import unittest
import numpy as np
from cad30_pose_b_jacobian_check import run, prepare
from wbc_contact_correction import micro_correct_stance

MODEL = Path(os.environ.get('CAD30_MODEL_XML', 'source/mjcf/scene_local30_nominal.xml'))
REPLAY = Path(os.environ.get('CAD30_REPLAY_ZIP','FNK0052_claw_tuck_ABC_2026-11-04_v2_no_CAD.zip'))

@unittest.skipUnless(MODEL.is_file() and REPLAY.is_file(), 'private CAD30 and replay must be restored locally')
class TestGenuineCAD30PoseB(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.r = run(MODEL,REPLAY)

    def test_real_model_loaded_and_four_contacts(self):
        self.assertEqual(self.r['environment']['mujoco'],'3.14.0')
        self.assertEqual(self.r['environment']['nmesh'],30)
        self.assertEqual(self.r['state']['four_stance_legs'],['ml','mr','bl','br'])

    def test_analytic_jacobian_matches_mj_forward_on_24_dofs(self):
        self.assertEqual(self.r['finite_differences']['tested_generalized_dofs'],24)
        self.assertLess(self.r['finite_differences']['max_jacobian_error'],1e-7)
        self.assertLess(self.r['finite_differences']['rmse_jacobian'],1e-8)

    def test_stance_feet_fixed_after_small_correction_in_forward_kinematics(self):
        t=self.r['micro_correction']; self.assertTrue(t['approved_kinematic'])
        self.assertGreater(min(t['base_only_foot_drift_mm']),.3)
        self.assertLess(t['max_compensated_drift_mm'],.002)
        self.assertLess(t['joint_max_abs_delta_rad'],.02)

    def test_model_range_must_not_be_mistaken_for_conservative_hardware_range(self):
        offenders=self.r['state']['joints_outside_research_IK_1p7_rad']
        self.assertEqual({x['name'] for x in offenders}, {'motor31','motor27'})

    def test_veto_big_body_shift_at_actual_pose(self):
        m,d,anchors,jb,jq,jids,dofs,qaddrs,lo,hi,t=prepare(MODEL,REPLAY)
        out=micro_correct_stance(joint_positions_rad=d.qpos[qaddrs],lower_limits_rad=lo,
          upper_limits_rad=hi,stance_jacobian_base=jb,stance_jacobian_joints=jq,
          desired_base_delta=np.array([0.,.005,0.,0.,0.,0.]))
        self.assertFalse(out.approved_kinematic)
        self.assertTrue(np.allclose(out.joint_delta_rad,0))

if __name__=='__main__':unittest.main()
