"""Synthetic STEP smoke only; does NOT claim the original EN01 CAD passed."""
from __future__ import annotations

import hashlib
import importlib.util
import pathlib
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "navbot_en01_audit_step", ROOT / "research/navbot_en01/audit_step.py")
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)


class StepAuditTests(unittest.TestCase):
    def test_missing_file_fails_closed(self):
        with self.assertRaisesRegex(ValueError, "regular local STEP"):
            AUDIT.inspect_step(pathlib.Path("does-not-exist-en01.stp"))

    def test_synthetic_box_step_and_digest(self):
        try:
            import cadquery as cq
        except ImportError:
            self.skipTest("CadQuery optional on unit-only CI")
        with tempfile.TemporaryDirectory() as tmp:
            path = pathlib.Path(tmp) / "synthetic-box.stp"
            cq.exporters.export(cq.Workplane("XY").box(20, 10, 5), str(path))
            sha = hashlib.sha256(path.read_bytes()).hexdigest()
            receipt = AUDIT.inspect_step(path, sha)
            self.assertEqual(receipt["solid_count"], 1)
            self.assertEqual(receipt["step_sha256"], sha)
            self.assertEqual(receipt["overall_bbox_importer_units"],
                             [-10.0, 10.0, -5.0, 5.0, -2.5, 2.5])
            self.assertIn("NOT-joint-axes-masses", receipt["verification_status"])
            with self.assertRaisesRegex(ValueError, "SHA256 mismatch"):
                AUDIT.inspect_step(path, "0" * 64)


if __name__ == "__main__":
    unittest.main()
