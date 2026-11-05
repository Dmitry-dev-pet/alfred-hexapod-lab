import copy
import unittest
from acceptance_screen import screen

class AcceptanceTests(unittest.TestCase):
    def setUp(self):
        self.r={
            'root_world_y_translation_mm':20.05,
            'terminal_leg_joint_closure_max_rad':0.005,
            'terminal_velocity_closure_max_rad_per_s':0.08,
            'max_abs_leg_joint_rad':1.62,
            'max_abs_leg_tau_Nm':0.68,
            'max_abs_head_tau_Nm':0.01,
            'toe_landmark_world_y_delta_mm':{x:20.0 for x in ('ml','mr','bl','br')},
            'swing_kinematic_screen':{x:{'passes_clearance_2mm':True,
                    'all_other_working_feet_within_2mm_on_sampled_swing_nodes':True}
                for x in ('ml','mr','bl','br')},
        }
    def test_pass_never_proves_physical_gait(self):
        ans=screen(self.r)
        self.assertEqual(ans['status'],'SAMPLED_SCREEN_ONLY_NOT_GAIT')
        self.assertFalse(ans['physical_gait_proven'])
    def test_torque_cap(self):
        self.r['max_abs_leg_tau_Nm']=0.71
        self.assertIn('LEG_TORQUE_MODEL_CAP',screen(self.r)['reasons'])
    def test_velocity_closure(self):
        self.r['terminal_velocity_closure_max_rad_per_s']=0.12
        self.assertIn('VELOCITIES_NOT_PERIODIC',screen(self.r)['reasons'])
    def test_not_four_swings(self):
        del self.r['swing_kinematic_screen']['br']
        self.assertIn('NO_SAMPLED_AIRBORNE_BR',screen(self.r)['reasons'])
    def test_strict_progress(self):
        self.r['root_world_y_translation_mm']=19.99
        self.assertIn('LESS_THAN_20MM_ROOT_ADVANCE',screen(self.r)['reasons'])
    def test_nonfinite(self):
        self.r['max_abs_leg_tau_Nm']=float('nan')
        self.assertEqual(screen(self.r)['status'],'INVALID')
    def test_foot_period(self):
        self.r['toe_landmark_world_y_delta_mm']['mr']=40.0
        self.assertIn('FOOT_NOT_PERIODIC_MR',screen(self.r)['reasons'])

if __name__=='__main__':unittest.main()
