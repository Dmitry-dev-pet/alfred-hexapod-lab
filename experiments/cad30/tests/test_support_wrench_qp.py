"""Reproduction tests; restore private CAD30/source in working directory first."""
from __future__ import annotations
import os
import sys
import unittest
from pathlib import Path
import numpy as np
from scipy.optimize import linprog

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments'/'cad30'))
from support_wrench_qp import load_stand,solve_stand,build_constraints,solve_exact_circular_cone,minimize_motor_fraction,support_margin_xy

ROOT=Path(os.environ.get('CAD30_ROOT',Path.cwd()))

class ContactFeasibilityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.stand=load_stand(ROOT)

    def test_01_physical_source_and_mass(self):
        s=self.stand
        self.assertEqual(len(s.codes),6)
        self.assertEqual(s.model.nu,20)
        self.assertAlmostEqual(s.model_mass_kg,1.8048682,places=5)
        self.assertEqual(s.original_stand_proxy['last_0p4s_min_contact_legs'],6)
        self.assertEqual(s.G.shape,(20,18))

    def test_02_rne_reconstructs_standing_actuator_torques(self):
        # If inverse dynamics had incorporated solver contacts a second time,
        # this check fails by ~0.3Nm on several joints.
        self.assertLess(self.stand.initial_torque_residual,0.01)

    def test_03_six_leg_equilibrium(self):
        s=self.stand
        sol=solve_stand(s,(),1.1)
        self.assertTrue(sol['LP_feasible'])
        self.assertLessEqual(sol['constraint_max_violation'],1e-7)
        self.assertLess(sol['force_balance_error_N'],1e-6)
        self.assertLess(sol['torque_balance_error_Nm'],1e-6)
        self.assertLess(sol['max_actuator_tau_fraction'],1.)
        self.assertAlmostEqual(sum(sol['vertical_forces_N'].values()),s.model_mass_kg*9.81,delta=1e-4)

    def test_04_each_one_leg_loss_is_feasible(self):
        for c in self.stand.codes:
            with self.subTest(c=c):
                result=solve_stand(self.stand,(c,),1.1)
                self.assertTrue(result['LP_feasible'])
                self.assertEqual(result['vertical_forces_N'][c],0.)
                self.assertLess(result['max_actuator_tau_fraction'],1.)
                self.assertGreater(result['support_polygon_margin_mm'],0)

    def test_05_removing_both_front_contacts_is_provably_infeasible(self):
        s=self.stand
        inner=solve_stand(s,('fr','fl'),1.1)
        outer=solve_stand(s,('fr','fl'),1.1,friction_approx='outer32')
        self.assertFalse(inner['LP_feasible'])
        self.assertFalse(outer['LP_feasible'])
        self.assertLess(inner['support_polygon_margin_mm'],0)
        # Even with actuator limits disabled, geometry alone rules it out.
        self.assertFalse(solve_stand(s,('fr','fl'),1.1,torque_cap=False)['LP_feasible'])

    def test_06_exact_cone_recovers_borderline_case(self):
        s=self.stand
        inner=solve_stand(s,('br','mr'),.3)
        self.assertFalse(inner['LP_feasible'])
        sol=solve_exact_circular_cone(s,('br','mr'),.3)
        self.assertTrue(sol['numerically_feasible'])
        self.assertLessEqual(sol['exact_cone_violation'],1e-8)
        self.assertAlmostEqual(sol['max_tau_fraction'],1.,places=6)

    def test_07_contact_geometry_support_margin(self):
        s=self.stand
        full=support_margin_xy(s.positions[:,:2],s.com[:2]); self.assertGreater(full,0.18)
        m=np.array([s.positions[i,:2] for i,c in enumerate(s.codes) if c not in ('fr','fl')])
        self.assertLess(support_margin_xy(m,s.com[:2]),0.)

    def test_08_friction_solution_is_inside_true_cone(self):
        s=self.stand;mu=.3
        case=solve_stand(s,('fl',),mu)
        self.assertTrue(case['LP_feasible'])
        f=np.array(list(case['forces_N'].values()))
        self.assertTrue(np.all(np.hypot(f[:,0],f[:,1])<=mu*f[:,2]+1e-6))

    def test_09_motor_headroom_optimization_not_confused_with_qp_torque(self):
        s=self.stand
        q=solve_stand(s,('ml','bl'),1.1)
        self.assertTrue(q['LP_feasible'])
        # QP tracks old measured forces and may hit a motor bound; that does
        # NOT imply the physically feasible torque-minimizing solution must.
        self.assertLess(q['minimum_required_motor_fraction']['min_motor_fraction'],.60)
        self.assertGreater(q['minimum_required_motor_fraction']['optimistic_remaining_motor_fraction'],.4)

    def test_10_refined_inner_friction_valid(self):
        s=self.stand
        for mu in [1.1,.6,.3]:
            for loss in [('fr',),('ml','mr')]:
                with self.subTest(mu=mu,loss=loss):
                    x=solve_stand(s,loss,mu)
                    self.assertTrue(x['LP_feasible'])
                    forces=np.array(list(x['forces_N'].values()))
                    self.assertTrue(np.all(np.linalg.norm(forces[:,:2],axis=1)<=mu*forces[:,2]+1e-6))

    def test_11_no_silent_wrong_leg_or_bad_mu(self):
        with self.assertRaises(ValueError):solve_stand(self.stand,('xx',),1.1)
        with self.assertRaises(ValueError):solve_stand(self.stand,('fl','fl'),1.1)
        with self.assertRaises(ValueError):solve_stand(self.stand,(),-0.1)

if __name__=='__main__':unittest.main(verbosity=2)