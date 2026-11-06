"""Local contract tests for the read-only Lite3 MJCF auditor."""
import unittest

from scripts.research.xgo_lite3_audit import audit_env, audit_model, git_blob_hash


class Lite3AuditTests(unittest.TestCase):
    def test_git_blob_hash(self):
        self.assertEqual(git_blob_hash(b"test\n"), "9daeafb9864cf43055ae93beb0afd6c7d144bfa4")

    def test_tiny_mjcf_report(self):
        xml = b'''<mujoco model="lite3"><default><position forcerange="-0.31 0.31"/></default>
        <worldbody><body name="body"><inertial mass="0.12"/>
        <joint name="FL_hip_joint"/><geom name="FL_shank_collision"/></body></worldbody>
        <actuator><position joint="FL_hip_joint"/></actuator></mujoco>'''
        a = audit_model(xml)
        self.assertEqual(a["actuator_count"], 1)
        self.assertEqual(a["leg_joint_count"], 1)
        self.assertEqual(a["shank_collision_geoms"], ["FL_shank_collision"])
        self.assertEqual(a["model_mass_kg"], 0.12)
        self.assertEqual(a["position_default_force_range"], "-0.31 0.31")
        self.assertFalse(a["verified_upstream_xml_blob"])

    def test_rejects_non_mjcf(self):
        with self.assertRaises(ValueError):
            audit_model(b"<robot/>")

    def test_env_extract(self):
        src = '''cfg.sim.mujoco.timestep = 0.004
cfg.decimation = 5
joint_pos_action.scale = 0.25
cfg.observations["actor"].terms["phase"].params["period"] = 0.4'''
        self.assertEqual(audit_env(src), {
            "mujoco_timestep_seconds": 0.004,
            "policy_decimation": 5,
            "nominal_policy_hz": 50.0,
            "action_scale": 0.25,
            "foot_phase_period_s": 0.4,
        })


if __name__ == "__main__":
    unittest.main()
