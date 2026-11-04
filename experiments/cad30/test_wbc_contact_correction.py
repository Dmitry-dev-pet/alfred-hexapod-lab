"""Synthetic Jacobian tests; these do not require original CAD/MuJoCo."""
from __future__ import annotations
import unittest
import numpy as np
from wbc_contact_correction import micro_correct_stance


def toy_model(base_delta=None, reference=None, joint_limit=1.0):
    """4 stance legs with J_j=-I in their respective 3-joint blocks.

    Toy units are algebraic m/rad, not actual Freenove Jacobians. Two remaining
    legs have zero rows, so they may follow independently requested increments.
    """
    n=18
    jb=np.zeros((12,6));jj=np.zeros((12,n))
    for leg in range(4):
        jb[3*leg:3*leg+3,:3]=np.eye(3)
        jj[3*leg:3*leg+3,3*leg:3*leg+3]=-np.eye(3)
    return dict(joint_positions_rad=np.zeros(n),
        lower_limits_rad=np.full(n,-joint_limit),upper_limits_rad=np.full(n,joint_limit),
        stance_jacobian_base=jb,stance_jacobian_joints=jj,
        desired_base_delta=np.array(base_delta if base_delta is not None else [0,.001,0,0,0,0],float),
        desired_joint_delta=np.zeros(n) if reference is None else np.array(reference,float))


class KinematicMicroAdjustmentTests(unittest.TestCase):
    def test_four_grounded_legs_compensate_same_body_translation(self):
        result=micro_correct_stance(**toy_model())
        self.assertTrue(result.approved_kinematic,result.reason)
        d=np.array(result.joint_delta_rad)
        for i in range(4):
            self.assertAlmostEqual(d[3*i+1],.001,delta=1e-8)
        self.assertLess(result.max_contact_linear_residual_m, 1e-7)
        self.assertLess(np.max(abs(d[12:])),1e-7)

    def test_suspended_leg_joints_can_follow_separate_small_reference(self):
        ref=np.zeros(18);ref[12:18]=[.003,-.002,.001,.002,-.001,.003]
        result=micro_correct_stance(**toy_model(reference=ref))
        self.assertTrue(result.approved_kinematic)
        self.assertTrue(np.allclose(np.array(result.joint_delta_rad)[12:],ref[12:],atol=1e-7))

    def test_large_correction_vetoes_without_commands(self):
        result=micro_correct_stance(**toy_model(base_delta=[0,.03,0,0,0,0]))
        self.assertFalse(result.approved_kinematic)
        self.assertTrue(all(v==0 for v in result.joint_delta_rad))

    def test_unreachable_stance_jacobian_vetoes_without_commands(self):
        p=toy_model();p['stance_jacobian_joints'][:]=0
        result=micro_correct_stance(**p)
        self.assertFalse(result.approved_kinematic)
        self.assertGreater(result.max_contact_linear_residual_m,.0009)

    def test_joint_rate_bound_blocks_unreachable_contact_preservation(self):
        result=micro_correct_stance(**toy_model(), max_joint_delta_rad=.0002)
        self.assertFalse(result.approved_kinematic)
        self.assertTrue(all(v==0 for v in result.joint_delta_rad))

    def test_joint_hard_limit_veto(self):
        p=toy_model();p['joint_positions_rad'][1]=1.
        result=micro_correct_stance(**p)
        self.assertFalse(result.approved_kinematic)

    def test_rotation_envelope_veto(self):
        result=micro_correct_stance(**toy_model(base_delta=[0,0,0,.02,0,0]))
        self.assertFalse(result.approved_kinematic)

    def test_invalid_jacobian_rejected(self):
        p=toy_model();p['stance_jacobian_base']=np.ones((8,6))
        with self.assertRaises(ValueError):micro_correct_stance(**p)


if __name__=='__main__':
    unittest.main()
