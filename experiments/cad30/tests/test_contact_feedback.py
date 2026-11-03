"""Checks for the experimental contact-gated gait and its original receipts.

Unit tests do not substitute for MuJoCo replay; 3 mass scenarios and friction
sensitivity were executed and stored before this file was generated.
"""
from pathlib import Path
import json
import sys
import unittest

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from local_contact_gait import CODES, LEG_RING, LocalLeg, may_lift, sample_plan


class TestContactGait(unittest.TestCase):
    def setUp(self):
        self.local={c:LocalLeg(c) for c in CODES}
        self.feet={'fl':(.18,-.12,0),'ml':(0,-.19,0),
                   'bl':(-.18,-.12,0),'br':(-.18,.12,0),
                   'mr':(0,.19,0),'fr':(.18,.12,0)}

    def read(self,file):
        return json.loads((ROOT/'results'/file).read_text())

    def test_six_unique_neighbor_order(self):
        self.assertEqual(set(LEG_RING),set(CODES))
        self.assertEqual(len(LEG_RING),6)

    def test_smooth_continuous_plan_wrap(self):
        q=np.array([[0,0,0],[1,1,1],[2,2,2],[1,1,1]],dtype=float)
        np.testing.assert_allclose(sample_plan(q,0),q[0])
        np.testing.assert_allclose(sample_plan(q,1),q[0])
        np.testing.assert_allclose(sample_plan(q,.125),np.ones(3)*.5)
        self.assertLess(float(np.max(np.abs(sample_plan(q,.99999)-q[0]))),.001)

    def test_liftoff_requires_real_contacts(self):
        ok,reason=may_lift('ml',self.local,set(CODES),np.zeros(2),self.feet,.02,0)
        self.assertTrue(ok,reason)
        ok,reason=may_lift('ml',self.local,set(CODES)-{'br'},np.zeros(2),self.feet,.02,0)
        self.assertFalse(ok)
        self.assertEqual(reason,'less_than_five_remaining_contacts')
        ok,reason=may_lift('ml',self.local,set(CODES)-{'ml'},np.zeros(2),self.feet,.02,0)
        self.assertFalse(ok)
        self.assertEqual(reason,'no_contact_at_liftoff')

    def test_neighbor_inhibition_and_tilt_gate(self):
        self.local['bl'].state='swing'
        ok,why=may_lift('ml',self.local,set(CODES),np.zeros(2),self.feet,.02,0)
        self.assertEqual((ok,why),(False,'neighbor_swing'))
        self.local['bl'].state='stance'
        ok,why=may_lift('ml',self.local,set(CODES),np.zeros(2),self.feet,.02,6)
        self.assertEqual((ok,why),(False,'body_tilt'))
        ok,why=may_lift('ml',self.local,set(CODES),np.ones(2)*4,self.feet,.02,0)
        self.assertEqual((ok,why),(False,'unsafe_five_leg_support_polygon'))

    def test_three_masses_completed_with_confirmed_touchdowns(self):
        for case in ('lower','nominal','upper'):
            with self.subTest(case=case):
                x=self.read(f'cad30_gait_contact_{case}.json')
                self.assertTrue(x['finished_full_duration'])
                self.assertFalse(x['fall_or_nonfinite'])
                self.assertEqual(x['missed_touchdown_steps'],0)
                self.assertGreaterEqual(x['touchdowns'],14)
                self.assertGreaterEqual(x['min_grounded_legs'],3)
                self.assertLessEqual(x['max_abs_torque_nm'],.70001)
                self.assertTrue(.08 < x['advance_m'] < .12)
                self.assertEqual(len(x['trace']),401)
                self.assertEqual(x['feedback']['centrally_phased_oscillators'],False)
                self.assertFalse(x['feedback']['biologically_validated_neuroWalknet'])

    def test_landing_ablation_is_reproducible(self):
        no_target=self.read('cad30_gait_contact_nominal_no_landing_target.json')
        zero=self.read('cad30_gait_contact_nominal_extension0mm.json')
        corrected=self.read('cad30_gait_contact_nominal.json')
        self.assertGreater(no_target['missed_touchdown_steps'],100)
        self.assertEqual(zero['missed_touchdown_steps'],0)
        self.assertEqual(corrected['missed_touchdown_steps'],0)
        self.assertGreater(zero['advance_m'],no_target['advance_m']*2)
        self.assertLess(abs(corrected['advance_m']-zero['advance_m']),.003)

    def test_low_friction_exposes_contact_loss(self):
        x=self.read('cad30_gait_contact_nominal_mu0p3.json')
        self.assertEqual(x['min_grounded_legs'],0)
        self.assertTrue(x['finished_full_duration'])
        self.assertGreater(x['max_body_tilt_deg'],1.)
        y=self.read('cad30_gait_contact_nominal_dwell100ms.json')
        self.assertLess(y['advance_m'],self.read('cad30_gait_contact_nominal.json')['advance_m'])


if __name__=='__main__':
    unittest.main()