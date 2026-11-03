"""Scientific guardrails for community CAD and FNK0052 mass estimation."""
import copy
import math
import unittest

from hexlab.community_mass_v04 import (
    REF_PATH, SCENARIOS_PATH, load, validate, totals, reference_breakdown,
)


class CommunityMassTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ref = load(REF_PATH)
        cls.scenario = load(SCENARIOS_PATH)

    def test_original_cad_mass_matches_link_sum(self):
        validate(self.ref, self.scenario)
        groups = reference_breakdown(self.ref)
        self.assertAlmostEqual(sum(groups.values()), 1.130196604, places=6)
        self.assertAlmostEqual(groups["coxa"], 6 * 0.082623, places=6)
        self.assertAlmostEqual(groups["tibia"], 6 * 0.0425984, places=6)

    def test_18_big_2_small_servo_shapes_already_in_cad(self):
        self.assertEqual(sum(x["servo_mesh_instances"]["big"] for x in self.ref["links"]), 18)
        self.assertEqual(sum(x["servo_mesh_instances"]["small"] for x in self.ref["links"]), 2)
        # Each rotating coxa group already has TWO big servo shapes.
        self.assertEqual(self.ref["links"][3]["servo_mesh_instances"]["big"], 2)

    def test_20_actuated_joints_urdf(self):
        self.assertEqual(
            len([j for j in self.ref["joints"] if j["type"] == "revolute"]), 20
        )

    def test_estimated_assembly_is_not_a_measured_mass(self):
        self.assertFalse(self.scenario["is_measured_mass"])
        est = totals(self.scenario)
        self.assertEqual(est, {"lower": 1.644, "nominal": 1.804868, "upper": 2.08})
        self.assertGreater(est["lower"], self.scenario["reference_cad_urdf_mass_kg"])

    def test_scenario_cannot_silently_drop_packs(self):
        broken = copy.deepcopy(self.scenario)
        next(x for x in broken["calculations"]["rows"] if x["id"] == "18650_high_current_cells")["count"] = 2
        with self.assertRaises(ValueError):
            validate(self.ref, broken)

    def test_scenario_must_reproduce_calculated_totals(self):
        broken = copy.deepcopy(self.scenario)
        broken["calculations"]["scenario_totals_kg"]["nominal"] += 0.2
        with self.assertRaises(ValueError):
            validate(self.ref, broken)

    def test_physics_gate_not_automatically_passed_by_community_estimate(self):
        self.assertTrue(all(x.get("inertia_kg_m2") is not None for x in self.ref["links"]))
        self.assertTrue(any("calibrated" in x.lower() or "unknown" in x.lower()
                            for x in self.scenario["warnings"]))
        # A source URDF inertial *number* does not certify an accurate physical model.
        self.assertIn("UNVALIDATED", self.ref["data_role"])


if __name__ == "__main__":
    unittest.main()
