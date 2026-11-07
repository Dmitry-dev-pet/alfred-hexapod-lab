"""Offline stdlib tests: physics receipt -> immutable Android catalog entry."""
import hashlib
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
import replay_pipeline as pipe


class PipelineTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.docs = self.root / "docs" / "hexapod"
        self.docs.mkdir(parents=True)
        source = self.docs / "wheelbot-self-righting-v1"
        source.mkdir()
        (source / "index.html").write_text('<script src="/hexapod/wheelbot-self-righting-v1/app.js"></script>')
        (source / "app.js").write_text('fetch("/hexapod/data/wheelbot-self-righting-v1.json");')
        (self.docs / "data").mkdir()
        (self.root / "results").mkdir()
        (self.docs / "library.json").write_text(json.dumps({
            "schema": "hexapod-library/v1", "revision": 32,
            "entries": [{"id": "old", "title": "Legacy", "detail": "old", "duration_s": 1,
                         "href": "/hexapod/", "files": []}]
        }))
        self.job = {
            "schema": pipe.SCHEMA, "id": "wheelbot-next-v1", "title": "Wheelbot new",
            "detail": "MuJoCo verified physics", "status": "failed",
            "robot": "Mini Wheelbot", "engine": "MuJoCo", "duration_s": 1.0,
            "commands": {"simulate": [sys.executable, "-c", "pass"],
                         "export": [sys.executable, "-c", "pass"]},
            "trace": {"path": "results/wheelbot-next-v1.json", "samples_field": "samples",
                      "time_field": "t", "qpos_field": "qpos"},
            "replay": {"path": "hexapod/data/wheelbot-next-v1.json",
                       "format": "mini-wheelbot-mujoco-physical-replay-v1",
                       "trace_sha256_field": "trace_sha256"},
            "player": {"template": "wheelbot-self-righting-v1"},
        }
        self.spec = self.root / "job.json"
        self.spec.write_text(json.dumps(self.job))
        self.old_root = pipe.ROOT
        pipe.ROOT = self.root
        self.addCleanup(setattr, pipe, "ROOT", self.old_root)
        self.prepare_physics()

    def prepare_physics(self):
        raw = {"samples": [{"t": t, "qpos": [1, 2, 3, 4]} for t in [0, .5, 1.0]]}
        blob = json.dumps(raw).encode()
        (self.root / self.job["trace"]["path"]).write_bytes(blob)
        replay = {"format": self.job["replay"]["format"], "physics_engine": "MuJoCo",
                  "duration_s": 1.0, "frames": [[0, 1], [1, 2], [2, 3]],
                  "trace_sha256": hashlib.sha256(blob).hexdigest(), "frame_count": 3}
        (self.root / "docs" / self.job["replay"]["path"]).write_text(json.dumps(replay))

    def test_spec_and_full_package_keep_old_and_preserve_immutable_records(self):
        spec = pipe.load_spec(self.spec)
        first, paths = pipe.package(self.root, spec)
        self.assertEqual((first["revision"], first["added"]), (33, 1))
        self.assertIn("docs/hexapod/wheelbot-next-v1/index.html", paths)
        self.assertIn("wheelbot-next-v1.json", (self.docs / "wheelbot-next-v1" / "app.js").read_text())
        self.assertEqual((self.docs / "library.json").read_text().count('"id"'), 2)
        before = (self.docs / "library.json").read_bytes()
        second, _ = pipe.package(self.root, spec)
        self.assertEqual((second["revision"], second["added"]), (33, 0))
        self.assertEqual((self.docs / "library.json").read_bytes(), before)

    def test_absent_physics_is_rejected(self):
        (self.root / self.job["trace"]["path"]).unlink()
        with self.assertRaisesRegex(ValueError, "Missing regular"):
            pipe.package(self.root, self.job)
        self.assertFalse((self.docs / "wheelbot-next-v1").exists())

    def test_modified_trace_is_rejected(self):
        with (self.root / self.job["trace"]["path"]).open("ab") as out:
            out.write(b" ")
        with self.assertRaisesRegex(ValueError, "not bound"):
            pipe.package(self.root, self.job)

    def test_non_monotonic_samples_are_rejected(self):
        path = self.root / self.job["trace"]["path"]
        data = json.loads(path.read_text())
        data["samples"][1]["t"] = 0.0
        path.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, "Non-monotone"):
            pipe.package(self.root, self.job)

    def test_nonfinite_qpos_is_rejected(self):
        path = self.root / self.job["trace"]["path"]
        raw = json.loads(path.read_text())
        raw["samples"][0]["qpos"][0] = float("nan")
        path.write_text(json.dumps(raw))
        with self.assertRaisesRegex(ValueError, "non-finite"):
            pipe.package(self.root, self.job)

    def test_wrong_replay_reference_does_not_create_page(self):
        path = self.root / "docs" / self.job["replay"]["path"]
        data = json.loads(path.read_text())
        data["trace_sha256"] = "0" * 64
        path.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, "not bound"):
            pipe.package(self.root, self.job)
        self.assertFalse((self.docs / "wheelbot-next-v1").exists())

    def test_renderer_must_reference_new_file(self):
        template = self.docs / "wheelbot-self-righting-v1" / "app.js"
        template.write_text('console.log("wheelbot-self-righting-v1");')
        with self.assertRaisesRegex(ValueError, "does not reference"):
            pipe.package(self.root, self.job)

    def test_refuses_existing_changed_page(self):
        target = self.docs / "wheelbot-next-v1"
        target.mkdir()
        (target / "index.html").write_text("evil")
        with self.assertRaisesRegex(ValueError, "Immutable renderer"):
            pipe.package(self.root, self.job)

    def test_invalid_spec_paths_are_rejected(self):
        self.job["trace"]["path"] = "results/../../secrets.json"
        self.spec.write_text(json.dumps(self.job))
        with self.assertRaisesRegex(ValueError, "Unsafe path"):
            pipe.load_spec(self.spec)

    def test_pipeline_executes_both_commands(self):
        calls = []
        with patch.object(pipe.subprocess, "run", side_effect=lambda argv, **kw: calls.append(argv)):
            output = pipe.run(self.root, self.job)
        self.assertEqual(calls, [self.job["commands"]["simulate"], self.job["commands"]["export"]])
        self.assertFalse(output["published"])

    def test_publish_requires_viewer_branch_before_computing(self):
        with patch.object(pipe, "git", return_value="main"):
            with self.assertRaisesRegex(ValueError, "viewer branch"):
                pipe.run(self.root, self.job, do_publish=True)


if __name__ == "__main__":
    unittest.main()
