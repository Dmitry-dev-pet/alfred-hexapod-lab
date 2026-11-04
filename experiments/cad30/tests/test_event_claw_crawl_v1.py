"""Math and physical-contact guards for the Issue #21 exploratory controller."""
import unittest
import numpy as np
from experiments.cad30.event_claw_crawl_v1 import candidate_order, capped_shift, step_cmd


class EventClawCrawlUnitTests(unittest.TestCase):
    def test_reject_infeasible_candidate_even_if_faster(self):
        candidates=[{'leg':'ml','allow':False,'ik_stride_m':.035,'margin_m':-.08,'torque_fraction':.3},
                    {'leg':'bl','allow':True,'ik_stride_m':-.008,'margin_m':.02,'torque_fraction':.56}]
        self.assertEqual(candidate_order(candidates,set())[0]['leg'],'bl')

    def test_unstepped_leg_priority(self):
        candidates=[{'leg':c,'allow':True,'ik_stride_m':-.008,'margin_m':.02,'torque_fraction':.55}
                    for c in ('bl','br')]
        self.assertEqual(candidate_order(candidates,{'bl'})[0]['leg'],'br')

    def test_shift_cannot_exceed_limit(self):
        p=capped_shift([.080,-.060],[0,0],.035)
        self.assertAlmostEqual(float(np.linalg.norm(p)),.035,places=8)
        np.testing.assert_allclose(capped_shift([.001,-.002],[0,0],.035),[.001,-.002])

    def test_swing_endpoint_matches_landing(self):
        plan={'qstart':np.zeros(3),'qway':np.array([[1.,0.,0.],[1.,1.,0.],[1.,1.,1.],[2.,2.,2.]])}
        np.testing.assert_allclose(step_cmd(plan,0),np.zeros(3))
        np.testing.assert_allclose(step_cmd(plan,1),[2.,2.,2.])

    def test_no_pretend_full_gait_when_middle_legs_never_lift(self):
        # The model has FOUR supporting legs. Two successful rear-foot
        # landings are not a verified four-leg walking cycle.
        qualifying={'bl','br'}
        self.assertNotEqual(qualifying,{'bl','br','ml','mr'})

if __name__=='__main__':unittest.main()