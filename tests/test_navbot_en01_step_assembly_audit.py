"""Offline source-format tests. No original third-party STEP is committed."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "research" / "navbot_en01"))
from step_assembly_audit import analyze_bytes


def synthetic_step(axis="0.,0.,1.", second_z=66.15):
    records = []
    for i, (label, z) in enumerate((("QUANUM GIMBAL MOTOR 2208 ASM:1", 0),
                                    ("QUANUM GIMBAL MOTOR 2208 ASM:2", second_z))):
        off = 10 + i * 100
        records.extend([
            f"#{off}=NEXT_ASSEMBLY_USAGE_OCCURRENCE('{label}','{label}','{label}',#90,#91,'{label}');",
            f"#{off+1}=PRODUCT_DEFINITION_SHAPE($,$,#{off});",
            f"#{off+2}=CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#{off+3},#{off+1});",
            f"#{off+3}=( REPRESENTATION_RELATIONSHIP($,$,#1,#2) REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#{off+4}) SHAPE_REPRESENTATION_RELATIONSHIP() );",
            f"#{off+4}=ITEM_DEFINED_TRANSFORMATION($,$,#{off+5},#{off+6});",
            f"#{off+5}=AXIS2_PLACEMENT_3D('',#{off+7},#{off+9},#{off+10});",
            f"#{off+6}=AXIS2_PLACEMENT_3D('',#{off+8},#{off+9},#{off+10});",
            f"#{off+7}=CARTESIAN_POINT('',(0.,0.,0.));",
            f"#{off+8}=CARTESIAN_POINT('',(1.,2.,{z}));",
            f"#{off+9}=DIRECTION('',({axis}));",
            f"#{off+10}=DIRECTION('',(1.,0.,0.));",
        ])
    return ("ISO-10303-21;\nDATA;\n" + "\n".join(records) + "\nENDSEC;\n").encode("ascii")


class StepAssemblyAuditTests(unittest.TestCase):
    def test_pair_span_and_frames(self):
        result = analyze_bytes(synthetic_step(), expected_blob=None)
        self.assertEqual(result["counts"]["assembly_placements"], 2)
        self.assertAlmostEqual(result["pairs"]["wheel_motor_assembly"]["origin_distance_mm"], 66.15)
        self.assertEqual(result["pairs"]["wheel_motor_assembly"]["offset_mm"], [0., 0., 66.15])
        self.assertEqual(result["status"], "SOURCE-CAD-FRAMES-ONLY_NOT-KINEMATIC-OR-PHYSICS-VALIDATED")

    def test_forged_original_fails_closed(self):
        with self.assertRaisesRegex(ValueError, "SHA1 mismatch"):
            analyze_bytes(synthetic_step())

    def test_nonorthogonal_axes_rejected(self):
        with self.assertRaisesRegex(ValueError, "nonorthogonal"):
            analyze_bytes(synthetic_step(axis="1.,0.,0."), expected_blob=None)

    def test_missing_transform_rejected(self):
        raw = synthetic_step().replace(b"REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION",
                                       b"IGNORED_REPRESENTATION_RELATIONSHIP_WITH_TRANSFORM")
        with self.assertRaisesRegex(ValueError, "No placement relation"):
            analyze_bytes(raw, expected_blob=None)

    def test_missing_assembly_usage_rejected(self):
        raw = b"ISO-10303-21;\nDATA;\n" + b"".join(
            f"#{i}=CARTESIAN_POINT('',(0.,0.,0.));\n".encode() for i in range(20)
        )
        with self.assertRaisesRegex(ValueError, "No traceable placements"):
            analyze_bytes(raw, expected_blob=None)


if __name__ == "__main__":
    unittest.main()
