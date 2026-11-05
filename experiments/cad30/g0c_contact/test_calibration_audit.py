import unittest
from calibration_audit import idto_soft_contact_force, extract_torque_peaks, torque_posthoc_gate, parse_contact_probe

class CalibrationAuditTests(unittest.TestCase):
    def test_force_at_zero_gap_is_not_zero(self):
        self.assertAlmostEqual(idto_soft_contact_force(0.), 13.862943611, places=7)
    def test_idto_ghost_force_at_20mm_positive_gap(self):
        self.assertAlmostEqual(idto_soft_contact_force(.020),2.5385602208594498,places=7)
        self.assertGreater(idto_soft_contact_force(.002),11.)
    def test_force_approaches_zero_at_large_gap(self):
        self.assertLess(idto_soft_contact_force(.1),.001)
    def test_reject_invalid_contact_model_params(self):
        with self.assertRaises(ValueError):idto_soft_contact_force(0.,smoothing_m=0.)
    def test_torque_gate_pass_does_not_certify_hard_bound(self):
        peaks=[.01]*26; peaks[7]=.5; peaks[25]=.18
        s='Maximum iterations reached\nMax torques: ['+', '.join(map(str,peaks))+']'
        report=torque_posthoc_gate(s)
        self.assertEqual(report['ex_post_cap_check'],'PASS')
        self.assertFalse(report['certified_torque_feasible'])
        self.assertEqual(report['run_convergence'],'MAX_ITERATIONS_REACHED')
    def test_torque_gate_rejects_leg_overshoot(self):
        peaks=[.01]*26;peaks[7]=.701
        self.assertEqual(torque_posthoc_gate('Max torques: ['+','.join(map(str,peaks))+']')['ex_post_cap_check'],'FAIL')
    def test_torque_gate_rejects_head_overshoot(self):
        peaks=[.01]*26;peaks[24]=.21
        self.assertEqual(torque_posthoc_gate('Max torques: ['+','.join(map(str,peaks))+']')['ex_post_cap_check'],'FAIL')
    def test_incomplete_vector_rejected(self):
        with self.assertRaises(ValueError):extract_torque_peaks('Max torques: [0, 0]')
    def test_parse_contact_timeline(self):
        report=parse_contact_probe('drake_fz tibiaml 3\ntimeline 1 17 4 0.164\ntimeline 2 18 4 0.163')
        self.assertEqual(len(report['timeline']),2)
        self.assertEqual(report['first_per_foot_N']['ml'],3.)
    def test_missing_tick_rejected(self):
        with self.assertRaises(ValueError):parse_contact_probe('drake_fz tibiaml 3\ntimeline 1 17 4 0.164\ntimeline 3 18 4 0.163')

if __name__=='__main__':unittest.main()
