"""Static checks for the EN01 connector to the existing Android 3D replay pipeline."""
import json
import pathlib
import unittest

ROOT=pathlib.Path(__file__).resolve().parents[1]
MODES=("stand","crouch","jump","jump-fast")

class EN01ReplayContractTests(unittest.TestCase):
    def test_four_declarative_jobs_are_review_only(self):
        for mode in MODES:
            p=ROOT/"experiments/replay-pipeline"/("navbot-en01-"+mode+"-20261108.json")
            job=json.loads(p.read_text(encoding="utf-8"))
            self.assertEqual(job["schema"],"hexapod-replay-job/v1")
            self.assertEqual(job["publish_policy"],"review_only")
            self.assertEqual(job["player"]["template"],"navbot-en01-template")
            self.assertEqual(job["commands"]["simulate"][1],"research/navbot_en01/probe.py")
            self.assertEqual(job["commands"]["export"][1],"research/navbot_en01/export_replay.py")
            self.assertEqual(job["status"],"failed" if "jump" in mode else "research")
            self.assertEqual(job["replay"]["trace_sha256_field"],"source.trace_sha256")

    def test_renderer_uses_measured_four_axis_qpos(self):
        p=ROOT/"docs/hexapod/navbot-en01-template/app.js"
        code=p.read_text(encoding="utf-8")
        for token in ("left.hip.rotation.y=v(8)","left.wheel.rotation.y=v(9)",
                      "right.hip.rotation.y=v(10)","right.wheel.rotation.y=v(11)",
                      "/hexapod/data/navbot-en01-template.json","data.source?.trace_sha256"):
            self.assertIn(token,code)
        self.assertNotIn(".mp4",code)
        self.assertNotIn("new Video",code)

    def test_pipeline_and_provenance(self):
        script=(ROOT/"scripts/navbot_en01_replays.py").read_text()
        self.assertIn("from replay_pipeline import ROOT, load_spec, run",script)
        self.assertIn("do_publish=False",script)
        exporter=(ROOT/"research/navbot_en01/export_replay.py").read_text()
        for token in ("mujoco_version","qpos","trace_sha256","immutable replay"):
            self.assertIn(token,exporter)

if __name__=="__main__":unittest.main()
