"""Synthetic CAD assembly test; does not substitute for real HWeigo CAD."""
import importlib.util
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import cadquery as cq

P=Path(__file__).resolve().parents[1]/"research/bb8/step_ingest.py"
spec=importlib.util.spec_from_file_location("step_ingest",P)
step_ingest=importlib.util.module_from_spec(spec)
spec.loader.exec_module(step_ingest)


class TestStepIngest(unittest.TestCase):
    def test_placed_parts_are_separate_and_scaled(self):
        with TemporaryDirectory() as tmp:
            tmp=Path(tmp)
            ass=cq.Assembly(name="synthetic_test")
            ass.add(cq.Workplane("XY").box(10,20,30),name="box",
                    loc=cq.Location(cq.Vector(50,0,0)))
            ass.add(cq.Workplane("XY").sphere(6), name="sphere",
                    loc=cq.Location(cq.Vector(-50,0,0)))
            src=tmp/"test.step"
            ass.save(str(src),exportType="STEP")
            output=step_ingest.convert(src,tmp/"output","mm",linear_deflection=.25)
            self.assertEqual(output["status"], "STATIC_GEOMETRY_ONLY_NOT_PHYSICS")
            self.assertEqual(output["solid_count"],2)
            centers=[(p["aabb_min_m"][0]+p["aabb_max_m"][0])/2 for p in output["parts"]]
            self.assertAlmostEqual(max(centers),.05,places=5)
            self.assertAlmostEqual(min(centers),-.05,places=5)
            self.assertTrue(all((tmp/"output"/p["file"]).stat().st_size>84 for p in output["parts"]))
            self.assertEqual(len(list((tmp/"output").glob("*.stl"))),2)
            self.assertEqual(json.loads((tmp/"output"/"static_solids.json").read_text())["solid_count"],2)
            self.assertFalse(output["joint_axes_verified"])

    def test_explicit_units_required(self):
        with TemporaryDirectory() as tmp:
            path=Path(tmp)/"one.step"
            with self.assertRaises(ValueError):
                step_ingest.convert(path,Path(tmp)/"out","guess")

    def test_rejects_wrong_step_hash(self):
        with TemporaryDirectory() as tmp:
            tmp=Path(tmp)
            src=tmp/"one.step"
            cq.Workplane("XY").box(1,1,1).val().exportStep(str(src))
            with self.assertRaisesRegex(ValueError,"STEP SHA-256 mismatch"):
                step_ingest.convert(src,tmp/"out","mm", expected_step_sha256="0"*64)

    def test_rejects_wrong_sldasm_hash(self):
        with TemporaryDirectory() as tmp:
            tmp=Path(tmp)
            src=tmp/"dummy.sldasm";src.write_bytes(b"not the original")
            with self.assertRaisesRegex(ValueError,"SLDASM hash mismatch"):
                step_ingest.convert(tmp/"missing.step",tmp/"out","mm",
                          source_sldasm_path=src,expected_sldasm_sha1="a"*40)

if __name__=="__main__":unittest.main()
