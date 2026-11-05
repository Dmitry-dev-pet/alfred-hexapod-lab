"""Issue #41: actual model gates for CAD30 forward rear-pair contact excursion.

Private source CAD must be supplied externally; test suite SKIPs rather than
pretending synthetic XML represents physical Freenove CAD30.
"""
from __future__ import annotations
import os
import unittest
from pathlib import Path
from two_support_transfer import run_probe

MODEL=Path(os.environ.get('CAD30_MODEL_XML','/private/CAD30-MODEL-NOT-AVAILABLE.xml'))
STATE=Path(os.environ.get('CAD30_SAFE_STATE','/private/CAD30-POSE-NOT-AVAILABLE.json'))
BASE=dict(rear_advance_mm=10,landing_body_shift_mm=6,forward_accel=4.5,
          forward_weight=20.,qp_duration_s=.02,return_ramp_s=.24,
          final_hold_s=.7,soft_joint_gain=5.,soft_joint_threshold=1.63)

@unittest.skipUnless(MODEL.exists() and STATE.exists(),'Private CAD30 model and archived physical B pose not available')
class ForwardRearPairPhysicalTests(unittest.TestCase):
    def test_original_QP_baseline_still_reproduces(self):
        r=run_probe(MODEL,STATE,record_qpos=False)
        self.assertEqual(r['status'],'TRANSIENT_TWO_SUPPORT_AND_FOUR_SUPPORT_RECOVERED')
        self.assertAlmostEqual(r['longest_contiguous_true_two_support_seconds'],.238,places=3)
        self.assertEqual(r['last_0p2s_four_reserve_steps'],100)
        self.assertFalse(r['partial_rear_pair_step_pass'])

    def test_forward_step_is_physical_not_command_mask(self):
        r=run_probe(MODEL,STATE,**BASE,record_qpos=True,record_physics_steps=True)
        self.assertEqual(r['status'],'PARTIAL_REAR_PAIR_FORWARD_STEP_LT20MM')
        self.assertTrue(r['partial_rear_pair_step_pass'])
        self.assertFalse(r['target_20mm_net_forward_pass'])
        self.assertEqual(len(r['physics_steps_2ms']),1780)
        self.assertEqual(len(r['qpos_samples_20ms']),178)
        self.assertGreater(r['longest_forward_airborne_continuous_seconds'],.1)
        flying=[p for p in r['physics_steps_2ms'] if p['two_support_with_real_rear_clearance']
                and min(p['rear_site_y_delta_mm'].values())>=2.]
        self.assertGreaterEqual(len(flying),25)
        for row in flying:
            self.assertEqual(row['loaded'],['ml','mr'])
            self.assertEqual(row['pairs']['bl'],0)
            self.assertEqual(row['pairs']['br'],0)
            self.assertGreaterEqual(min(row['rear_site_z_delta_mm'].values()),2.)
        for name in ('bl','br'):
            self.assertGreater(r['final_rear_footholds'][name]['forward_from_initial_mm'],5.)
            self.assertGreaterEqual(r['final_rear_footholds'][name]['Fz_N'],.2)
            self.assertGreaterEqual(r['final_rear_footholds'][name]['floor_pairs'],1)
        self.assertEqual(r['last_0p2s_four_reserve_steps'],100)
        self.assertEqual(r['below_two_support_steps'],0)
        self.assertEqual(r['clipped_PD_physics_steps'],0)
        self.assertLessEqual(r['max_joint_abs_rad'],1.7001)
        self.assertLessEqual(r['max_motor_tau_Nm'],.7)
        self.assertGreater(r['net_forward_COM_mm'],0.)
        self.assertLess(r['net_forward_COM_mm'],20.)
        self.assertFalse(r['QP_solver_failed'])

    def test_20mm_rear_stride_cannot_pass_fixed_pose_preflight(self):
        r=run_probe(MODEL,STATE,**{**BASE,'rear_advance_mm':20},record_qpos=False)
        self.assertEqual(r['status'],'TERMINAL_PREFLIGHT_BLOCKED')
        self.assertFalse(r['landing_preflight']['gate_passed'])
        self.assertGreater(r['landing_preflight']['max_contact_target_error_mm'],.2)

    def test_mu08_fails_even_if_four_stance_recovers(self):
        r=run_probe(MODEL,STATE,**BASE,mu=.8,record_qpos=False)
        self.assertEqual(r['status'],'RESEARCH_GATE_FAILED')
        self.assertFalse(r['partial_rear_pair_step_pass'])
        self.assertGreater(r['below_two_support_steps'],0)

    def test_mu06_qp_failure(self):
        r=run_probe(MODEL,STATE,**BASE,mu=.6,record_qpos=False)
        self.assertEqual(r['status'],'RESEARCH_GATE_FAILED')
        self.assertTrue(r['QP_solver_failed'])
        self.assertGreater(r['below_two_support_steps'],0)

if __name__=='__main__': unittest.main()
