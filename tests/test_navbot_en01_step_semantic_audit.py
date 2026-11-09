"""Fast stdlib tests for the STEP entity-level provenance/kinematics audit.

Only synthetic geometry is used here. These tests do NOT validate real EN01 CAD
or physical MuJoCo behavior.
"""
import importlib.util
from pathlib import Path
import tempfile
import unittest

AUDIT_PATH = Path(__file__).resolve().parents[1] / "research/navbot_en01/step_semantic_audit.py"
SPEC = importlib.util.spec_from_file_location("navbot_en01_semantic_audit", AUDIT_PATH)
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)


class StepSemanticAuditTests(unittest.TestCase):
    def test_reject_unexpected_blob(self):
        raw = (b"ISO-10303-21;\n#1=PRODUCT('Test','Test',$,());\nENDSEC;\n" * 4)
        with self.assertRaisesRegex(ValueError, "blob SHA1 mismatch"):
            AUDIT.analyze_bytes(raw)

    def test_synthetic_part_linkage_and_cylinder_axis(self):
        source = r"""ISO-10303-21;
#1=PRODUCT('2208\X2\7535673A8F6E80CE\X0\','wheel tire',$,(#100));
#2=PRODUCT_DEFINITION_FORMATION('',$,#1);
#3=PRODUCT_DEFINITION('wheel','wheel',#2,#200);
#4=PRODUCT_DEFINITION_SHAPE('',$,#3);
#5=SHAPE_DEFINITION_REPRESENTATION(#4,#6);
#6=SHAPE_REPRESENTATION('',(#99),#300);
#7=SHAPE_REPRESENTATION_RELATIONSHIP('SRR','None',#6,#8);
#8=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#9),#400);
#9=MANIFOLD_SOLID_BREP('body',#10);
#10=CLOSED_SHELL('',(#11));
#11=ADVANCED_FACE('',(#23),#12,.T.);
#12=CYLINDRICAL_SURFACE('',#13,18.5);
#13=AXIS2_PLACEMENT_3D('placement',#14,#15,#16);
#14=CARTESIAN_POINT('',(0.,0.,0.));
#15=DIRECTION('',(0.,1.,0.));
#16=DIRECTION('',(0.,0.,1.));
#17=( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );
END-ISO-10303-21;"""
        result = AUDIT.analyze_bytes(source.encode("ascii"), expected_blob=None)
        self.assertTrue(result["units"]["length_mm_declared"])
        self.assertEqual(result["counts"]["solids"], 1)
        self.assertEqual(result["counts"]["linked_brep_parts"], 1)
        faces = result["wheel_tire"]["candidate_cylinders"]
        self.assertEqual(len(faces), 1)
        self.assertEqual(faces[0]["radius_mm"], 18.5)
        self.assertEqual(faces[0]["axis_point_mm"], [0.0, 0.0, 0.0])
        self.assertEqual(faces[0]["axis_direction"], [0.0, 1.0, 0.0])

    def test_fail_closed_if_missing_or_symlink(self):
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder) / "missing.step"
            with self.assertRaisesRegex(ValueError, "regular local file"):
                AUDIT.analyze(p)
            p.symlink_to("nowhere")
            with self.assertRaisesRegex(ValueError, "regular local file"):
                AUDIT.analyze(p)


if __name__ == "__main__":
    unittest.main()
