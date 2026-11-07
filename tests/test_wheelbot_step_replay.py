"""Wheelbot V2 real source-trace exporter and row-format pipeline tests.

These unit fixtures are schematic data ONLY, not claims of new MuJoCo trials.
For real replay, source raw frames+summary must come from PR #86 native runner.
"""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import replay_pipeline as pipe
import export_wheelbot_step_replay as wb


class WheelbotStepExporterTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.trace = self.root / "roll_+90_upstream_cap.json"
        self.summary = self.root / "summary.json"
        self.out = self.root / "out" / "new.json"
        self.raw = {
            "info": {
                "status": "PROVISIONAL_GEOMETRY_MOTOR_AND_CONTROLLER_MAPPING",
                "mode": "upstream",
                "start_roll_deg": 90,
                "speed_cap_assumed": True,
                "numerical_warning": None,
                "meets_provisional_up_criterion": False,
                "motor_limits_hardware_validated": False,
                "source_joint_label_parity_validated": False,
                "simulated_duration_s": 0.1,
                "final_tilt_deg": 90,
                "min_tilt_deg": 81.4,
                "longest_upright_s": 0,
            },
            "frames": [
                [t, .0, .0, .07, 1., .0, .0, .0, t*.3, t*2,
                 90., 100*t, .1, 1]
                for t in (0.0, 0.05, 0.1)
            ]
        }
        self.write_fixture()

    def write_fixture(self):
        raw = (json.dumps(self.raw, separators=(",", ":")) + "\n").encode()
        self.trace.write_bytes(raw)
        summary = {
            "classification": wb.EXPECTED_CLASS,
            "upstream_commit": wb.COMMIT,
            "source_model_xml_sha256": "0" * 64,
            "runs": [{"frames_file": self.trace.name, "frames_sha256": wb.sha(raw)}],
        }
        self.summary.write_text(json.dumps(summary))

    def test_converts_actual_qpos_slots_without_inventing_angles(self):
        receipt = wb.export(self.trace, self.summary, self.out)
        d = json.loads(self.out.read_text())
        self.assertEqual(receipt["frames"], 3)
        self.assertEqual(d["status"], "failed")
        self.assertEqual(d["source"]["trace_sha256"], wb.sha(self.trace.read_bytes()))
        for original, displayed in zip(self.raw["frames"], d["frames"]):
            self.assertEqual(displayed, original[:10] + [original[11], original[12]])
            self.assertEqual(len(displayed), 12)
        self.assertEqual(wb.export(self.trace, self.summary, self.out), receipt)

    def test_rejects_modified_trace_not_matching_receipt(self):
        self.raw["frames"][0][8] += .7
        self.trace.write_text(json.dumps(self.raw))
        with self.assertRaisesRegex(ValueError, "receipt"):
            wb.export(self.trace, self.summary, self.out)
        self.assertFalse(self.out.exists())

    def test_rejects_numerical_instability_or_false_success(self):
        self.raw["info"]["numerical_warning"] = {"count": 1}
        self.write_fixture()
        with self.assertRaisesRegex(ValueError, "not the capped"):
            wb.export(self.trace, self.summary, self.out)
        self.raw["info"]["numerical_warning"] = None
        self.raw["info"]["meets_provisional_up_criterion"] = True
        self.write_fixture()
        with self.assertRaisesRegex(ValueError, "not claim"):
            wb.export(self.trace, self.summary, self.out)

    def test_rejects_incomplete_joint_state(self):
        self.raw["frames"][0].pop(8)
        self.write_fixture()
        with self.assertRaisesRegex(ValueError, "full physical state"):
            wb.export(self.trace, self.summary, self.out)

    def test_array_layout_validates_real_qpos_and_time_without_objects(self):
        path = self.root / "results" / self.trace.name
        path.parent.mkdir()
        path.write_bytes(self.trace.read_bytes())
        job = {
            "duration_s": 0.1,
            "trace": {"path": "results/" + self.trace.name,
                      "samples_field": "frames", "sample_format": "array",
                      "time_index": 0, "qpos_slice": [1, 10]}
        }
        self.assertEqual(pipe.verify_trace(self.root, job), wb.sha(path.read_bytes()))
        self.raw["frames"][1][0] = 0.0
        path.write_text(json.dumps(self.raw))
        with self.assertRaisesRegex(ValueError, "Non-monotone"):
            pipe.verify_trace(self.root, job)

    def test_review_only_job_refuses_publication_before_physics(self):
        job = pipe.load_spec(ROOT / "experiments/replay-pipeline/wheelbot-v2-step-source.json")
        self.assertEqual(job["publish_policy"], "review_only")
        with self.assertRaisesRegex(ValueError, "review_only"):
            pipe.run(self.root, job, do_publish=True)

    def test_declarative_job_is_loadable_without_source_code_installed(self):
        path = ROOT / "experiments/replay-pipeline/wheelbot-v2-step-source.json"
        job = pipe.load_spec(path)
        self.assertEqual(job["status"], "failed")
        self.assertEqual(job["trace"]["qpos_slice"], [1, 10])
        self.assertEqual(job["player"]["template"], "wheelbot-self-righting-v1")


if __name__ == "__main__":
    unittest.main()
