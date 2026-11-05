"""These tests exercise only the *global reporting screen* with synthetic data.

Passing tests must never be reported as a physical gait or solver success.
"""
import unittest
from global_screen import screen


def synthetic():
    def state(y):
        return {
            "root_xyz_m": [0.0, y, .16],
            "com_xyz_m": [0.0, y, .15],
            "body_quat_wxyz": [1,0,0,0],
            "leg_q_rad": [.2]*18,
            "leg_qd_rad_s": [0.]*18,
            "body_lin_velocity_world_m_s": [0,0,0],
            "body_ang_velocity_world_rad_s": [0,0,0],
            "loaded_legs": ["ml","mr","bl","br"],
            "front_loaded": False,
            "toe_world_xyz_m": {
                "ml": [-.2,y+.1,0], "mr": [.2,y+.1,0],
                "bl": [-.2,y-.1,0], "br": [.2,y-.1,0],
            }
        }
    return {
        "schema": "cad30-periodic-screen-v1",
        "source": "SYNTHETIC_TEST_ONLY",
        "preparation": {"original_six_com_y_m": 0.0, "cycle_start_com_y_m": -.02},
        "start": state(-.02),
        "end": state(.01),
        "swing_sequence": ["ml","mr","bl","br"],
        "bounds": {
            "max_raw_tau_Nm": .5,
            "max_abs_leg_q_rad": 1.5,
            "min_sampled_support_margin_mm": 7,
        }
    }


class GlobalScreenTests(unittest.TestCase):
    def test_synthetic_screen_only(self):
        c = synthetic()
        out = screen(c)
        self.assertEqual(out["status"], "GLOBAL_SCREEN_PASS_NOT_PHYSICAL_PROOF")
        self.assertEqual(out["projected_net_after_three_periods_mm"], 70.0)

    def test_old_local_ml_mr_ml_bl_is_rejected(self):
        c = synthetic()
        c["swing_sequence"] = ["ml","mr","ml","bl"]
        self.assertIn("NOT_FOUR_DISTINCT_SWINGS", screen(c)["reasons"])

    def test_backwards_prep_cannot_be_hidden(self):
        c = synthetic()
        c["preparation"]["original_six_com_y_m"] = .16
        self.assertIn("NET_INCLUDING_PREP_LESS_20MM", screen(c)["reasons"])

    def test_zero_body_motion_not_gait(self):
        c = synthetic()
        c["end"]["root_xyz_m"][1] = c["start"]["root_xyz_m"][1]
        self.assertIn("ROOT_COM_TRANSLATION_DISAGREE", screen(c)["reasons"])

    def test_joint_pose_must_close(self):
        c = synthetic()
        c["end"]["leg_q_rad"][0] += .07
        self.assertIn("JOINTS_NOT_PERIODIC", screen(c)["reasons"])

    def test_joint_velocity_must_close(self):
        c = synthetic()
        c["end"]["leg_qd_rad_s"][4] = .15
        self.assertIn("JOINT_SPEED_NOT_PERIODIC", screen(c)["reasons"])

    def test_base_velocity_must_close(self):
        c = synthetic()
        c["end"]["body_lin_velocity_world_m_s"][1] = .05
        self.assertIn("ROOT_LIN_SPEED_NOT_PERIODIC", screen(c)["reasons"])

    def test_feet_must_translate_with_body(self):
        c = synthetic()
        c["end"]["toe_world_xyz_m"]["br"][1] -= .020
        self.assertIn("FOOT_BR_NOT_TRANSLATED", screen(c)["reasons"])

    def test_period_boundary_requires_four_real_supports(self):
        c = synthetic()
        c["end"]["loaded_legs"].remove("br")
        self.assertIn("NOT_FOUR_SUPPORTS_AT_PERIOD_BOUNDARIES", screen(c)["reasons"])

    def test_front_leg_must_be_unloaded(self):
        c = synthetic()
        c["end"]["front_loaded"] = True
        self.assertIn("FRONT_LEGS_NOT_LIFTED", screen(c)["reasons"])

    def test_model_limit_cannot_be_ignored(self):
        c = synthetic()
        c["bounds"]["max_raw_tau_Nm"] = .75
        self.assertIn("TORQUE_RESEARCH_LIMIT", screen(c)["reasons"])

    def test_incomplete_trace_rejected(self):
        c = synthetic()
        del c["end"]["leg_q_rad"]
        self.assertEqual(screen(c)["status"],"REJECT")


if __name__ == "__main__":
    unittest.main()
