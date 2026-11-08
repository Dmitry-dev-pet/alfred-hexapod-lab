"""Reproducibility & non-overclaim assertions for actual free-root V11 physics receipts."""
import json,unittest
from pathlib import Path
P=Path(__file__).resolve().parent
S=json.loads((P/'SUMMARY_V12.json').read_text())
C=S['cases']
class PhysicsEvidence(unittest.TestCase):
    def test_full_53_run_matrix_provenance(self):
        self.assertEqual(S['data_files_count'],53)
        self.assertEqual(S['variant_counts'],{'nominal_full':29,'upper_mass_full':10,'friction_035_full':7,'torque_035_full':7})
        self.assertEqual(S['model_version'],'MuJoCo 3.14.0')
        self.assertEqual(S['source_scene_sha256'],'fe9d7c292b199f19b11da220e74f826e05fb5433a43c5b46c1bc620eb34f6df3')
        self.assertTrue(S['free_root'] and S['physics_computed_not_animation'])
        self.assertFalse(S['original_Freenove_PWM_limits_calibrated'])
    def test_true_dynamic_body_yaw_with_six_loaded_feet(self):
        for name,sign in [('stand_yaw_plus15',1),('stand_yaw_minus15',-1)]:
            r=C['nominal_full'][name]
            self.assertTrue(r['no_fall'])
            self.assertEqual(r['min_real_contacts'],6)
            self.assertLess(r['max_torque_Nm'],.2)
            self.assertAlmostEqual(r['actual_yaw_at_active_end_deg'],15*sign,delta=.5)
            self.assertLess(r['max_loaded_contact_slip_speed_mm_s'],1)
    def test_actual_gaits_do_not_pass_contacts_or_calibration(self):
        nom=C['nominal_full']
        for name in ['tripod_forward','tripod_turn','tripod_forward_turn','wave_forward','wave_turn']:
            self.assertTrue(nom[name]['no_fall'])
            self.assertLess(nom[name]['min_real_contacts'],3)
        self.assertGreater(nom['tripod_turn']['actual_yaw_unwrapped_deg'],200)
        self.assertAlmostEqual(nom['tripod_turn']['command_odometry_yaw_deg'],62.5,delta=.5)
        self.assertLess(nom['wave_turn']['command_odometry_yaw_deg'],31)
        self.assertGreater(nom['wave_turn']['actual_yaw_unwrapped_deg'],50)
    def test_sensitivity_physical_and_not_cosmetic(self):
        nom=C['nominal_full']['tripod_forward']
        reduced=C['friction_035_full']['tripod_forward']
        self.assertGreater(reduced['loaded_contact_fraction_speed_gt_5mm_s'],.4)
        self.assertGreater(reduced['loaded_contact_fraction_speed_gt_5mm_s'],nom['loaded_contact_fraction_speed_gt_5mm_s']+.3)
        self.assertEqual(reduced['min_real_contacts'],1)
        lowtau=C['torque_035_full']['tripod_forward']
        self.assertGreater(lowtau['saturation_fraction'],.6)
        self.assertGreater(lowtau['saturation_fraction'],nom['saturation_fraction']+.4)
        self.assertGreater(C['upper_mass_full']['tripod_forward']['saturation_fraction'],nom['saturation_fraction'])
    def test_static_mixed_body_contact_warning(self):
        combo=C['nominal_full']['stand_roll_pitch10']
        self.assertTrue(combo['no_fall'])
        self.assertEqual(combo['min_real_contacts'],2)
        self.assertLess(combo['min_support_margin_mm'],0)
    def test_runner_does_not_drive_base_or_weld_feet(self):
        src=(P/'run_physics_v11.py').read_text()
        self.assertIn('mj.mj_step(m,d)',src)
        self.assertNotIn('mj.mj_forward(m,d)\n        # fake physical',src)
        self.assertNotIn('d.qpos[:3]=qtarget',src)
        self.assertNotIn('d.qpos[3:7]=qtarget',src)
        self.assertNotIn('data.xpos[',src)
        # Renderer-only GroundLock is excluded from this physics test.
        self.assertNotIn('ground_lock',src.lower())
    def test_private_original_meshes_not_in_evidence_code(self):
        self.assertFalse(list(P.glob('**/*.stl')))
        self.assertFalse(list(P.glob('**/*.xml')))
        self.assertFalse(S['runtime_browser_mujoco'])
if __name__=='__main__':unittest.main()
