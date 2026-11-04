import unittest
from reflex_unload_middle import clamp_rate, leg_local_press, ready_for_swing, experiment
class LoadControlUnitTests(unittest.TestCase):
    def test_press_rejects_when_not_requested_or_at_capacity(self):
        self.assertEqual(leg_local_press(.1,False,6.),0.)
        self.assertEqual(leg_local_press(6.,True,6.),0.)
        self.assertGreater(leg_local_press(1.,True,6.),0.)
    def test_bounded_change(self):
        self.assertAlmostEqual(clamp_rate(.001,.020,.0005),.0015)
    def test_gate_requires_three_loaded_contacts_and_stable_gyro(self):
        base=([2.,3.,2.],[True,True,True],[False,False],1.2)
        self.assertTrue(ready_for_swing(.2,*base,candidate_contact=True))
        self.assertFalse(ready_for_swing(2.,*base,candidate_contact=True))
        self.assertFalse(ready_for_swing(.2,*base,candidate_contact=False))
        self.assertFalse(ready_for_swing(.2,*base,gyro_speed_rad_s=.9))
        self.assertFalse(ready_for_swing(.2,[2.,.2,2.],[True]*3,[False]*2,1.2))
        self.assertFalse(ready_for_swing(.2,[2.,3.,2.],[True]*3,[False,True],1.2))
    def test_nan_rejected(self):
        self.assertFalse(ready_for_swing(float('nan'),[2.,2.,2.],[True]*3,[False]*2,1.2))
class IntegrationTests(unittest.TestCase):
    def test_nominal_reflex_defers_middle_liftoff(self):
        r=experiment(record=False)
        self.assertTrue(r['complete'])
        self.assertIsNone(r['release_at_s'])
        self.assertFalse(r['front_any_contact_after_request'])
        self.assertEqual(r['min_support_contacts_after_request'],4)
        self.assertLess(r['mean_oracle_margin_final_1s_m'],0)
        self.assertEqual(r['saturation_fraction'],0)
    def test_nominal_reflex_redistributes_measured_load(self):
        control=experiment(mode='no_feedback')
        test=experiment(mode='reflex')
        self.assertLess(test['mean_candidate_load_final_1s_N'],control['mean_candidate_load_final_1s_N']-.5)
        self.assertLess(test['mean_oracle_margin_final_1s_m'],0)
if __name__=='__main__':unittest.main()
