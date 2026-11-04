"""Physical CAD30 2-support recovery tests; skip without private owner CAD.

Do not substitute synthetic data for real MuJoCo: these tests explicitly
SKIP if the proprietary original CAD files or archived state are absent.
"""
from __future__ import annotations
import os,sys,unittest
from pathlib import Path
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'experiments/cad30'))
from two_support_transfer import run_probe,THRESHOLD_FZ_N,RECONTACT_RESERVE_FZ_N
from dynamic_support_qp import LEGS

MODEL=Path(os.environ.get('CAD30_MODEL_XML','/nonexistent/private/CAD30/model.xml'))
STATE=Path(os.environ.get('CAD30_SAFE_STATE','/nonexistent/private/CAD30/pose.json'))
PHYSICAL=MODEL.is_file() and STATE.is_file()

class ContractTests(unittest.TestCase):
    def test_real_leg_contract(self):
        self.assertEqual(tuple(LEGS),('ml','mr','bl','br'))
        self.assertLess(THRESHOLD_FZ_N,RECONTACT_RESERVE_FZ_N)

    def test_no_unbounded_qp_phase(self):
        with self.assertRaises(ValueError):
            run_probe(MODEL,STATE,qp_duration_s=.5)

    def test_negative_duration(self):
        with self.assertRaises(ValueError):
            run_probe(MODEL,STATE,qp_duration_s=0.)

    @unittest.skipUnless(PHYSICAL,'PRIVATE CAD30 original source + safe state not available')
    def test_true_two_contacts_and_verified_four_return(self):
        r=run_probe(MODEL,STATE,mu=1.1,qp_duration_s=.12,return_target_mm=0)
        self.assertEqual(r['status'],'TRANSIENT_TWO_SUPPORT_AND_FOUR_SUPPORT_RECOVERED')
        self.assertGreaterEqual(r['longest_contiguous_true_two_support_seconds'],.2)
        self.assertEqual(r['last_0p2s_four_reserve_steps'],100)
        self.assertTrue(r['last_0p2s_all_four_Fz_ge_0p2N'])
        self.assertEqual(r['below_two_support_steps'],0)
        self.assertFalse(r['QP_solver_failed'])
        self.assertEqual(r['clipped_PD_physics_steps'],0)
        self.assertLess(r['max_joint_abs_rad'],1.7)
        self.assertLess(r['max_motor_tau_Nm'],.7)
        self.assertEqual(len(r['qpos_samples_20ms']),191)
        self.assertFalse(r['is_full_walk'])
        good=[x for x in r['trace_20ms'] if x['genuine_two_support']]
        self.assertTrue(good)
        for x in good:
            self.assertEqual(x['floor_contact_pairs']['bl'],0)
            self.assertEqual(x['floor_contact_pairs']['br'],0)
            self.assertGreaterEqual(x['rear_clearance_mm']['bl'],2)
            self.assertGreaterEqual(x['rear_clearance_mm']['br'],2)

    @unittest.skipUnless(PHYSICAL,'PRIVATE CAD30 original source + safe state not available')
    def test_lower_friction_is_not_falsely_accepted(self):
        r=run_probe(MODEL,STATE,mu=.6,qp_duration_s=.12,return_target_mm=0,record_qpos=False)
        self.assertEqual(r['status'],'RESEARCH_GATE_FAILED')
        self.assertFalse(r['last_0p2s_all_four_Fz_ge_0p2N'])
        self.assertTrue(r['QP_solver_failed'] or r['below_two_support_steps']>0)

if __name__=='__main__':unittest.main()
