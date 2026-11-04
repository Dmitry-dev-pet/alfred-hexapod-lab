"""Evidence audit and Freenove kinematic-reference tests (no MuJoCo needed)."""
from __future__ import annotations

import copy
import math
import unittest

from hexlab.physical_v04 import (
    load_manifest, manufacturer_leg_fk_mm, mass_report,
    ready_for_physics_comparison, validate_manifest,
)


class PhysicalReconstructionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.doc = load_manifest()

    def test_known_link_lengths_are_cited_kinematics_not_cad(self):
        datum = self.doc["geometry"]["effective_leg_link_lengths_mm"]
        self.assertEqual(datum["value"], [33, 90, 110])
        self.assertEqual(datum["unit"], "mm")
        self.assertEqual(datum["source"], "vendor_control")
        self.assertIn("not individual", datum["meaning"])

    def test_six_vendor_reference_points_and_frames(self):
        geo = self.doc["geometry"]
        self.assertEqual(len(geo["default_foot_reference_points_mm"]["value"]), 6)
        self.assertEqual(geo["leg_frame_transform_angles_deg"]["value"],
                         [54, 0, -54, -126, 180, 126])
        self.assertIsNone(geo["actual_servo_axis_positions_mm"]["value"])

    def test_bill_of_materials_has_18_plus_2_servos_and_four_cells(self):
        parts = {row["id"]: row for row in self.doc["mass_ledger"]}
        self.assertEqual(parts["leg_servos"]["count"], 18)
        self.assertEqual(parts["head_servos"]["count"], 2)
        self.assertEqual(parts["battery_holders"]["count"], 2)
        self.assertEqual(parts["batteries_18650"]["count"], 4)

    def test_no_invented_verified_mass(self):
        for config in self.doc["configurations"]:
            result = mass_report(self.doc, config)
            self.assertIsNone(result["total_mass_kg"])
            self.assertFalse(result["mass_budget_complete"])
            self.assertTrue(result["unknown_components"])
        self.assertFalse(ready_for_physics_comparison(self.doc))

    def test_legacy_1402g_is_a_separate_assumption(self):
        old = self.doc["prior_v03_approximation"]
        calculated = old["torso_mass_kg"] + 6 * (
            sum(old["per_leg_segment_masses_kg"]) + old["per_foot_mass_kg"]
        )
        self.assertAlmostEqual(calculated, old["total_simple_geom_mass_kg"])

    def test_manufacturer_fk_reference_zero_angles(self):
        # Freenove angle_to_coordinate uses angle_a as leg-local vertical-plane angle.
        x, y, z = manufacturer_leg_fk_mm(0, 0, 0)
        self.assertAlmostEqual(x, 0)
        self.assertAlmostEqual(y, 0)
        self.assertAlmostEqual(z, 233)
        x, y, z = manufacturer_leg_fk_mm(90, 0, 0)
        self.assertAlmostEqual(x, 0)
        self.assertAlmostEqual(y, 233)
        self.assertAlmostEqual(z, 0, places=12)

    def test_manufacturer_fk_reference_swing(self):
        x, y, z = manufacturer_leg_fk_mm(90, 30, 60)
        self.assertAlmostEqual(x, 90 * 0.5 + 110, places=9)
        self.assertAlmostEqual(y, 33 + 90 * math.cos(math.pi / 6), places=9)
        self.assertAlmostEqual(z, 0, places=9)

    def test_mass_must_have_provenance(self):
        modified = copy.deepcopy(self.doc)
        modified["mass_ledger"][0]["mass_each_kg"] = 0.06
        modified["mass_ledger"][0]["source"] = None
        with self.assertRaises(ValueError):
            validate_manifest(modified)

    def test_cannot_mark_unknown_geometry_measured_without_evidence(self):
        modified = copy.deepcopy(self.doc)
        modified["geometry"]["actual_servo_axis_positions_mm"]["value"] = [1, 2, 3]
        with self.assertRaises(ValueError):
            validate_manifest(modified)

    def test_negative_mass_is_rejected(self):
        modified = copy.deepcopy(self.doc)
        modified["mass_ledger"][0]["mass_each_kg"] = -0.02
        with self.assertRaises(ValueError):
            validate_manifest(modified)


if __name__ == "__main__":
    unittest.main()
