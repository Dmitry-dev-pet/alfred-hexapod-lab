"""Tests for event-driven CAD30 contact selection; no MPC claims."""
from __future__ import annotations
import sys
import unittest
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'experiments'/'cad30')]
from event_contact_planner import choose_contact_set, run, touchdown_metric


def candidate(**kwargs):
    base={'new_lifts':['fl'],'feasible':True,'reachable':True,'margin_m':.040,
          'min_motor_fraction':.30,'load_std_N':.50,'predicted_step_sum_m':.035}
    return {**base,**kwargs}


class ContactChoiceTest(unittest.TestCase):
    def test_never_lift_if_torque_reserve_inadequate(self):
        self.assertIsNone(choose_contact_set([candidate(min_motor_fraction=.98)]))

    def test_never_lift_if_support_margin_inadequate(self):
        self.assertIsNone(choose_contact_set([candidate(margin_m=-.002)]))
        self.assertIsNone(choose_contact_set([candidate(margin_m=.003)]))

    def test_never_select_unreachable_landing(self):
        self.assertIsNone(choose_contact_set([candidate(reachable=False)]))

    def test_select_lower_cost_not_fixed_leg_order(self):
        a=candidate(new_lifts=['fl'],min_motor_fraction=.7,margin_m=.02)
        b=candidate(new_lifts=['mr'],min_motor_fraction=.2,margin_m=.05)
        self.assertEqual(choose_contact_set([a,b])['new_lifts'],['mr'])
        self.assertEqual(choose_contact_set([b,a])['new_lifts'],['mr'])

    def test_allow_two_lifts_when_feasible(self):
        a=candidate(new_lifts=['mr'],predicted_step_sum_m=.035)
        b=candidate(new_lifts=['ml','mr'],predicted_step_sum_m=.07,margin_m=.044)
        self.assertEqual(choose_contact_set([a,b])['new_lifts'],['ml','mr'])

    def test_empty_or_infeasible_selection(self):
        self.assertIsNone(choose_contact_set([]))
        self.assertIsNone(choose_contact_set([{'new_lifts':['fl'],'feasible':False}]))

    def test_polygon_can_reject_outside_com(self):
        import numpy as np
        pts=[np.array([0.,0.,0.]),np.array([1.,0.,0.]),np.array([0.,1.,0.])]
        self.assertLess(touchdown_metric(pts,np.array([2.,2.,.1])),0)
        self.assertGreater(touchdown_metric(pts,np.array([.25,.25,.1])),0)


class PhysicalSmokeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.result=run(seed=23,mu=1.1,seconds=4.,max_parallel=2,force_feedback=False)

    def test_actual_walking_without_clocked_tripod(self):
        d=self.result
        self.assertTrue(d['finished'])
        self.assertGreater(d['advance_cm'],3.)
        self.assertGreater(d['liftoffs'],5)
        self.assertFalse(d['fall_guard_triggered'])
        self.assertTrue(d['not_MPC'])

    def test_logged_force_and_torque_reserve(self):
        d=self.result
        self.assertGreater(d['candidate_oracle_checks'],d['liftoffs'])
        self.assertLess(d['peak_tau_nm'],.7)
        self.assertGreaterEqual(d['min_contacts_2ms'],0)
        self.assertTrue(all(e['min_motor_fraction']<.82 for e in d['events'] if e['event']=='optimized_liftoff'))

    def test_synchronized_touchdown_and_support(self):
        d=self.result
        self.assertTrue(any(e['event']=='touchdown_observed' for e in d['events']))
        self.assertTrue(any(len(e['new_lifts'])==2 for e in d['events'] if e['event']=='optimized_liftoff'))
        self.assertLess(d['fraction_zero_2ms'],.001)


if __name__=='__main__':unittest.main()