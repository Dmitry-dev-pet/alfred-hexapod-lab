"""Integration regressions against owner-restored genuine CAD30/MuJoCo source.

Run with CAD30_MODEL_XML and CAD30_SAFE_STATE_JSON. No original third-party
meshes in GitHub. Never treat simulated feasibility as hardware certification.
"""
from __future__ import annotations
import os,unittest
from pathlib import Path
import numpy as np
from four_step_lookahead_v1 import make_plan,terminal_options,load_context
from four_step_physical_v1 import rollout
from four_step_contact_recovery_v1 import run_batch

MODEL=Path(os.environ.get('CAD30_MODEL_XML','source/mjcf/scene_local30_nominal.xml'))
STATE=Path(os.environ.get('CAD30_SAFE_STATE_JSON','cad30_safe_pose_B_nominal.json'))

@unittest.skipUnless(MODEL.is_file() and STATE.is_file(),'Genuine private CAD30 model/state required')
class FourStepIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ctx=load_context(MODEL,STATE)
        cls.plan,cls.nodes=make_plan(cls.ctx)
        cls.physics=rollout(cls.ctx,cls.nodes,node_seconds=.18)
        cls.feedback=run_batch(cls.ctx,cls.nodes,cls.physics)

    def test_sampled_quasistatic_full_four_step_math_candidate(self):
        self.assertEqual(self.plan['status'],'FOUR_STEP_SAMPLED_QUASISTATIC_CANDIDATE')
        self.assertFalse(self.plan['physical_gait_validated'])
        self.assertFalse(self.plan['autonomous_motor_execution_authorized'])
        c=self.plan['cycle'];self.assertEqual(c['swung_legs'],['ml','mr','bl','br'])
        self.assertGreater(c['COM_forward_progress_from_after_prep_mm'],20)
        self.assertGreater(c['minimum_sampled_support_margin_mm'],5)
        self.assertLess(c['maximum_sampled_motor_fraction'],.9)
        self.assertLess(c['maximum_joint_absolute_rad'],1.7)
        self.assertLess(c['maximum_anchored_toe_target_error_mm'],.2)

    def test_preparatory_rear_placements_not_hidden_as_cycle(self):
        self.assertEqual(self.plan['prep']['extra_preparation_swings'],['bl','br'])
        self.assertEqual(self.plan['prep']['waypoints'],90)
        self.assertEqual(self.plan['cycle']['waypoints'],69)
        self.assertEqual(len(self.nodes['prep_waypoints']),90)
        self.assertEqual(len(self.nodes['cycle_waypoints']),69)
        self.assertLess(self.plan['cycle']['COM_forward_progress_from_original_safe_snapshot_mm'],0)
        touchdown_legs=[x['phase'].split('_')[0] for x in self.nodes['cycle_waypoints'] if x['phase'].endswith('touchdown_4')]
        self.assertEqual(touchdown_legs,['ml','mr','bl','br'])

    def test_end_configuration_allows_at_least_one_optimistic_next_liftoff(self):
        self.assertTrue(self.plan['cycle']['terminal_viable_at_least_one_quasistatic'])
        opts=terminal_options(self.ctx,self.nodes['final_qpos'])
        self.assertTrue(opts['bl']['possible_now_static'])
        self.assertTrue(opts['br']['possible_now_static'])
        self.assertFalse(opts['ml']['possible_now_static'])
        self.assertFalse(opts['mr']['possible_now_static'])

    def test_real_free_base_physics_aborts_before_unverified_cycle(self):
        p=self.physics
        self.assertEqual(p['status'],'DYNAMIC_CHECK_ABORTED')
        self.assertIn('post-touchdown lost 4',p['stopped_reason'])
        self.assertEqual(p['prep_waypoints_executed'],24)
        self.assertEqual(p['cycle_waypoints_executed'],0)
        self.assertEqual(p['min_loaded_stance_legs'],3)
        self.assertGreater(p['min_physical_support_margin_mm'],0)
        self.assertEqual(p['motor_saturation_count'],0)
        self.assertFalse(p['true_gait_verified'])

    def test_feedback_restores_contacts_only_in_bounded_range(self):
        a=self.feedback['cases']
        for x in ('0','0.5','1.0','3.0'):
            self.assertEqual(a[x]['status'],'DYNAMIC_RECOVERY_VETO',msg=x)
        for x in ('1.5','2.0'):
            self.assertEqual(a[x]['status'],'DYNAMIC_RECOVERY_PASS',msg=x)
            self.assertTrue(a[x]['tail_0p2_s_all_four'])
            self.assertGreater(a[x]['end_fz_N']['br'],.2)
            self.assertEqual(a[x]['saturations'],0)
        self.assertNotIn('bl',a['3.0']['loaded_end'])
        self.assertFalse(self.feedback['autonomous_followup_authorized'])

if __name__=='__main__':unittest.main()