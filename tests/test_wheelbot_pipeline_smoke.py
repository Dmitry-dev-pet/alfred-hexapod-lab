"""Real-data integration proof for the replay pipeline: existing Wheelbot v1."""
from pathlib import Path
import hashlib
import json
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import replay_pipeline as pipe
import smoke_wheelbot_pipeline as smoke


class RealWheelbotReplaySmoke(unittest.TestCase):
    def test_recorded_physics_asset_is_pinned_and_complete(self):
        source = ROOT / smoke.SOURCE
        old, trace = smoke.archive_trace(source)
        self.assertEqual(old["frame_count"], 200)
        self.assertEqual(len(trace["samples"]), 200)
        self.assertEqual(trace["samples"][0]["qpos"], old["frames"][0][1:10])
        self.assertEqual(trace["samples"][-1]["qpos"], old["frames"][-1][1:10])
        self.assertFalse("simulated" in trace["derivation"].lower())

    def test_full_archive_packaging_keeps_canonical_library_untouched(self):
        old_catalog = (ROOT / pipe.CATALOG).read_bytes()
        with tempfile.TemporaryDirectory() as work:
            receipt = smoke.run_smoke(ROOT, Path(work))
            self.assertEqual(receipt["status"], "ARCHIVAL_PIPELINE_SMOKE_PASS")
            self.assertFalse(receipt["physical_run_executed"])
            self.assertFalse(receipt["new_replay_published"])
            self.assertEqual(receipt["source_frames"], 200)
            self.assertEqual(receipt["added"], 1)
            old = json.loads(old_catalog)
            new = json.loads((Path(work) / pipe.CATALOG).read_text())
            self.assertEqual(new["entries"][:-1], old["entries"])
            self.assertEqual(new["revision"], old["revision"] + 1)
        self.assertEqual((ROOT / pipe.CATALOG).read_bytes(), old_catalog)

    def test_idempotent_packaging_with_same_archived_frames(self):
        with tempfile.TemporaryDirectory() as work:
            root = Path(work)
            job, _ = smoke.assemble(ROOT, root)
            first, _ = pipe.package(root, job)
            catalog = (root / pipe.CATALOG).read_bytes()
            second, _ = pipe.package(root, job)
            self.assertEqual(first["added"], 1)
            self.assertEqual(second["added"], 0)
            self.assertEqual((root / pipe.CATALOG).read_bytes(), catalog)

    def test_changed_extracted_qpos_is_blocked(self):
        with tempfile.TemporaryDirectory() as work:
            root = Path(work)
            job, _ = smoke.assemble(ROOT, root)
            trace_path = root / job["trace"]["path"]
            trace = json.loads(trace_path.read_text())
            trace["samples"][5]["qpos"][0] += 1
            trace_path.write_text(json.dumps(trace))
            with self.assertRaisesRegex(ValueError, "not bound"):
                pipe.package(root, job)
            self.assertFalse((root / "docs/hexapod" / smoke.ID).exists())

    def test_archival_experiment_never_runs_a_simulation(self):
        with tempfile.TemporaryDirectory() as work:
            root = Path(work)
            job, _ = smoke.assemble(ROOT, root)
            self.assertIn("archive only", job["commands"]["simulate"][-1])
            result = pipe.run(root, job, skip_execution=True)
            self.assertEqual(result["added"], 1)
            self.assertFalse(result["published"])


if __name__ == "__main__":
    unittest.main()
