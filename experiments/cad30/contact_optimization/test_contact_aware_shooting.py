"""Regression checks for contact-aware, physics-in-the-loop middle-leg controller.

Private integration tests are SKIP, not PASS, if CAD and 2-ms private physics
traces are missing. No real robot is connected or actuated by these tests.
"""
from __future__ import annotations
import copy
import json
import os
import sys
import unittest
from pathlib import Path
import numpy as np
from contact_aware_shooting import (smoothstep,triangle_margin_mm,minimal_com_translation,
    PhysicalTrial, Command, accepted, objective)

class TestContactGeometry(unittest.TestCase):
    def test_smoothstep_monotone(self):
        v=[smoothstep(t) for t in np.linspace(0,1,101)]
        self.assertEqual(v[0],0);self.assertEqual(v[-1],1)
        self.assertTrue(np.all(np.diff(v)>=0))

    def test_minimum_com_shift_reaches_requested_margin(self):
        # Known four-foot-style three-toe triangle, not model CAD substitute.
        xy={'mr':np.array([.19,.0]),'bl':np.array([-.16,-.13]),'br':np.array([.16,-.13])}
        initial=np.array([.01,-.08])
        original_margin=triangle_margin_mm(xy,initial)
        self.assertLess(original_margin,15.)
        shift=minimal_com_translation(initial,xy,15.)
        margin=triangle_margin_mm(xy,initial+shift)
        self.assertGreaterEqual(margin,15.-1e-6)
        # Smaller shifts cannot satisfy the active linear inequality.
        self.assertLess(triangle_margin_mm(xy,initial+shift*.99),15.)

    def test_missing_support_not_reported_as_pass(self):
        xy={'mr':[.19,0],'bl':[-.16,-.13]}
        self.assertEqual(triangle_margin_mm(xy,[0.,-.08]),float('-inf'))

    def test_guards_reject_missed_support_or_joint_limits(self):
        good={'first_guard_violation':None,'swing_min_tripod_Fz_N':1.2,
              'swing_min_tripod_margin_mm':20.,'ml_mesh_height_over_2mm_no_contact_s_sampled':.6,
              'ml_in_air_forward_mm':18.,'max_ik_toe_error_mm':.5,
              'max_stance_toe_marker_xy_drift_mm':.6,'actual_ml_world_forward_mm':22.,
              'final_four_loaded_last_hold_fraction':1.,
              'final_Fz_N':{k:2. for k in ('ml','mr','bl','br')},
              'max_raw_leg_tau_Nm':.5,'max_leg_joint_abs_rad':1.6,
              'command':vars(Command(25.,32.,18.)),'accepted':True}
        self.assertTrue(accepted(good))
        for key,bad in [('swing_min_tripod_Fz_N',0.),('max_leg_joint_abs_rad',1.8),
                        ('max_ik_toe_error_mm',4.),('actual_ml_world_forward_mm',10.),
                        ('max_stance_toe_marker_xy_drift_mm',7.)]:
            x=copy.deepcopy(good);x[key]=bad;self.assertFalse(accepted(x),msg=key)
        fail=copy.deepcopy(good);fail['first_guard_violation']={'reason':'lost_br'}
        self.assertFalse(accepted(fail))
        self.assertLess(objective(good),objective(fail))


@unittest.skipUnless(all(os.environ.get(k) for k in ('CAD30_ROOT','HEXAPOD_REPO_ROOT','HEXAPOD_MATH_TRACE','HEXAPOD_PHYSICS_TRACE')),
 'Private CAD+recorded qpos/qvel trace unavailable; do not silently substitute synthetic physics')
class TestRealCAD30Simulation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cmd=Command(26.,28.211554388708002,16.419039367295706)
        cls.args=[Path(os.environ[k]) for k in ('CAD30_ROOT','HEXAPOD_REPO_ROOT','HEXAPOD_MATH_TRACE','HEXAPOD_PHYSICS_TRACE')]
    def test_actual_middle_lift_at_mu_1p1(self):
        t=PhysicalTrial(*self.args,mu=1.1)
        r=t.rollout(self.cmd)
        self.assertTrue(r['accepted'],r['first_guard_violation'])
        self.assertGreaterEqual(r['actual_ml_world_forward_mm'],20.)
        self.assertGreaterEqual(r['swing_min_tripod_Fz_N'],.2)
        self.assertGreaterEqual(r['ml_mesh_height_over_2mm_no_contact_s_sampled'],.25)
    def test_slippery_counterexample_mu_0p6(self):
        t=PhysicalTrial(*self.args,mu=.6)
        r=t.rollout(self.cmd)
        self.assertFalse(r['accepted'])
        self.assertIsNotNone(r['first_guard_violation'])

if __name__=='__main__':unittest.main()
