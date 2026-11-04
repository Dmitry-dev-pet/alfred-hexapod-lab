"""Real CAD30 numerical checks; skip if user-owned model is not restored."""
from __future__ import annotations
import json,os,unittest
from pathlib import Path
import numpy as np
from mink_contact_preflight import (load_context,StepPreflight,STANCE,
    required_com_shift_for_middle_after_rear_steps)
from mink_contact_recovery import assess_recovery

MODEL=Path(os.environ.get('CAD30_MODEL_XML','source/mjcf/scene_local30_nominal.xml'))
SAFE=Path(os.environ.get('CAD30_SAFE_STATE','results/cad30_safe_pose_B_nominal.json'))
DYNAMIC=Path(os.environ.get('CAD30_DYNAMIC_RESULT','results/cad30_one_rear_dynamics_full.json'))

@unittest.skipUnless(MODEL.is_file() and SAFE.is_file(),'restore private CAD30 model and saved safe-state JSON')
class RealModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ctx=load_context(MODEL,SAFE)

    def test_four_physically_loaded_legs_and_eighteen_actuated_leg_joints(self):
        self.assertEqual(set(self.ctx.foot_world),set(STANCE))
        self.assertEqual(len(self.ctx.joint_qpos_addrs),18)
        self.assertEqual(self.ctx.model.nmesh,30)
        self.assertLessEqual(float(max(abs(self.ctx.qpos[self.ctx.joint_qpos_addrs]))),1.7)

    def test_both_rear_candidates_have_24_valid_quasistatic_ik_nodes(self):
        for leg in ('bl','br'):
            with self.subTest(leg=leg):
                path=StepPreflight(self.ctx).attempt_rear_step(leg,(0,0,.024))
                self.assertEqual(path['status'],'ONE_STEP_STATIC_NODES_PASS')
                self.assertEqual(len(path['nodes']),24)
                self.assertGreater(min(row['margin_mm'] for row in path['nodes']),5)
                self.assertLess(max(row['max_abs_joint_rad'] for row in path['nodes']),1.7)
                self.assertLess(max(row['minimum_motor_fraction'] for row in path['nodes']),.9)
                self.assertTrue(all(row['gate_passed'] for row in path['nodes']))

    def test_without_preparatory_body_change_rear_lift_is_vetoed(self):
        check=StepPreflight(self.ctx).attempt_rear_step('bl',(0,0,0))
        self.assertEqual(check['status'],'BLOCKED_AT_NODE')
        self.assertFalse(check['blocking_node']['gate_passed'])

    def test_future_middle_lifts_not_authorized_after_only_20mm_rear_moves(self):
        x=required_com_shift_for_middle_after_rear_steps(self.ctx,.020)
        self.assertGreater(x['ml']['minimum_com_shift_for_5mm_margin_mm'],90.)
        self.assertGreater(x['mr']['minimum_com_shift_for_5mm_margin_mm'],90.)

    @unittest.skipUnless(DYNAMIC.is_file(),'full independently integrated MuJoCo state not attached')
    def test_feedback_compares_same_dynamic_state_to_unchanged_goal(self):
        recorded=json.loads(DYNAMIC.read_text())
        hold=assess_recovery(self.ctx,recorded,0.)
        half=assess_recovery(self.ctx,recorded,.5)
        one=assess_recovery(self.ctx,recorded,1.)
        over=assess_recovery(self.ctx,recorded,2.)
        self.assertFalse(hold['recovery_passed_local_dynamics'])
        self.assertTrue(half['recovery_passed_local_dynamics'])
        self.assertTrue(one['recovery_passed_local_dynamics'])
        self.assertFalse(over['recovery_passed_local_dynamics'])
        self.assertFalse(one['autonomous_second_step_authorized'])
        self.assertLessEqual(one['max_joint_abs_rad'],1.7)
        self.assertEqual(one['saturated_timesteps'],0)
        self.assertLess(hold['Fz_at_end_N']['br'],.08)
        self.assertGreater(one['Fz_at_end_N']['br'],.45)
        self.assertTrue(one['min_0p2s_all_four_loaded'])

if __name__=='__main__':unittest.main()