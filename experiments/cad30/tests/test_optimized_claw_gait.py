"""Numerical checks, including real CAD30 dynamics; third-party STL stay private."""
from __future__ import annotations
import sys
import unittest
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'experiments'/'cad30'))
sys.path.insert(0,str(ROOT))
from optimized_claw_gait import (optimal_com_target,choose_one_or_two,init_model,
    physical_contact_forces,force_oracle,balance_forces_qp,plan_shift_with_model,
    support_margin_xy)

class MathAndPhysicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.m,cls.d,cls.legs=init_model(1.1)
        cls.force,cls.foot=physical_contact_forces(cls.m,cls.d)
        cls.com=(np.asarray(cls.d.xipos)*np.asarray(cls.m.body_mass)[:,None]).sum(axis=0)/cls.m.body_mass.sum()

    def test_com_projection_is_nearest_feasible(self):
        f=np.array([[0,0],[1,0],[0,1]])
        target=optimal_com_target(np.array([.4,.4]),f,.10)
        self.assertAlmostEqual(target[0],.4,places=5)
        self.assertAlmostEqual(target[1],.4,places=5)
        outside=optimal_com_target(np.array([.8,.8]),f,.05)
        self.assertGreaterEqual(support_margin_xy(f,outside),.05-1e-6)

    def test_one_lift_does_not_need_shift(self):
        plan=choose_one_or_two(1,self.foot,self.com[:2],1.1)
        self.assertEqual(len(plan['claws']),1)
        self.assertAlmostEqual(float(np.linalg.norm(plan['com_shift_m'])),0.,places=6)

    def test_two_front_lifts_require_backwards_com(self):
        plan=choose_one_or_two(2,self.foot,self.com[:2],1.1)
        self.assertEqual(set(plan['claws']),{'fl','fr'})
        self.assertLess(plan['com_shift_m'][1],-.015)
        self.assertGreaterEqual(plan['planned_support_margin_m'],.016-1e-5)

    def test_static_feasibility_differs_for_one_vs_two(self):
        single=force_oracle(self.m,self.d,self.legs,['fl'],1.1)
        double=force_oracle(self.m,self.d,self.legs,['fl','fr'],1.1)
        self.assertTrue(single['feasible'])
        self.assertFalse(double['feasible'])
        self.assertLess(double['margin_m'],0)

    def test_force_optimizer_respects_wrench_and_motors(self):
        res=force_oracle(self.m,self.d,self.legs,['fl'],1.1)
        self.assertTrue(res['feasible']);self.assertTrue(res['force_balancing']['used_balanced_slsqp'])
        self.assertLess(res['force_balancing']['actual_max_torque_fraction'],.93)
        self.assertAlmostEqual(sum(res['fz_targets_N'].values()),self.m.body_mass.sum()*9.81,delta=.005)
        self.assertEqual(res['fz_targets_N']['fl'],0.)

    def test_generated_shift_is_not_fixed_40_mm(self):
        plan=plan_shift_with_model(2,1.1)
        self.assertIsNotNone(plan['shift_m'])
        self.assertEqual(len(plan['candidate_receipts']),5)
        self.assertNotAlmostEqual(plan['shift_m'],.04,places=4)
        self.assertTrue(all(x['score'] is not None for x in plan['candidate_receipts'] if x['feasible']))

if __name__=='__main__':unittest.main(verbosity=2)