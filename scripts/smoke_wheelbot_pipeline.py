#!/usr/bin/env python3
"""Prove replay packaging with a pinned *existing* Mini Wheelbot recording.

This is an ARCHIVAL INTEGRATION TEST, NOT a new physics simulation. It reuses
all 200 existing measured frames without inventing movement or uploading assets.
Every generated file lives in a temporary, isolated directory by default.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import replay_pipeline as pipeline

SOURCE = "docs/hexapod/data/wheelbot-self-righting-v1.json"
SOURCE_GIT_BLOB = "5f0f16c49fcc70d4913fb3c0549138a3292d6f10"
TEMPLATE = "wheelbot-self-righting-v1"
ID = "wheelbot-v1-pipeline-smoke-20261107"
COLUMNS = [
    "time_s", "root_xyz_m_3", "root_quat_wxyz_4",
    "drive_joint_rad", "reaction_joint_rad",
    "reaction_omega_rad_s", "reaction_command_Nm"
]


def git_blob_sha(data: bytes) -> str:
    return hashlib.sha1(f"blob {len(data)}\0".encode() + data).hexdigest()


def archive_trace(source: Path) -> tuple[dict, dict]:
    content = source.read_bytes()
    pipeline.require(git_blob_sha(content) == SOURCE_GIT_BLOB,
                     "Pinned real-physics source has been modified")
    original = json.loads(content)
    pipeline.require(original.get("format") == "mini-wheelbot-mujoco-physical-replay-v1",
                     "Unexpected Wheelbot data format")
    pipeline.require(original.get("physics_engine") == "MuJoCo", "Not a MuJoCo recording")
    pipeline.require(original.get("frame_columns") == COLUMNS, "Unexpected Wheelbot layout")
    frames = original.get("frames")
    pipeline.require(isinstance(frames, list) and len(frames) == 200 and
                     original.get("frame_count") == len(frames),
                     "Missing archived physical frames")
    samples = []
    for frame in frames:
        pipeline.require(isinstance(frame, list) and len(frame) == 12 and
                         all(pipeline.number(v) for v in frame),
                         "Corrupt original pose/actuator frame")
        samples.append({"t": frame[0], "qpos": frame[1:10]})
    trace = {
        "schema": "wheelbot-archived-frames-extraction/v1",
        "source_git_blob_sha1": SOURCE_GIT_BLOB,
        "derivation": "Exact stored root and joint coordinates from archived player frames; not original solver trace",
        "samples": samples,
    }
    return original, trace


def assemble(source_root: Path, temporary_root: Path) -> tuple[dict, dict]:
    """Build a real-archive fixture in a throwaway checkout-shaped directory."""
    source = source_root / SOURCE
    original, trace = archive_trace(source)
    docs = temporary_root / "docs" / "hexapod"
    docs.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source_root / pipeline.CATALOG, docs / "library.json")
    template = source_root / "docs" / "hexapod" / TEMPLATE
    target = docs / TEMPLATE
    shutil.copytree(template, target)
    data_dir = docs / "data"
    data_dir.mkdir()
    result_dir = temporary_root / "results"
    result_dir.mkdir()
    raw_name = "results/" + ID + "-trace.json"
    raw_bytes = (json.dumps(trace, ensure_ascii=False, separators=(",", ":")) + "\n").encode("utf-8")
    (temporary_root / raw_name).write_bytes(raw_bytes)
    replay_name = "hexapod/data/" + ID + ".json"
    replay = {
        **original,
        "title": "Mini Wheelbot · архивная запись · проверка replay pipeline",
        "verification": {
            "raw_trace_sha256": hashlib.sha256(raw_bytes).hexdigest(),
            "original_replay_git_blob_sha1": SOURCE_GIT_BLOB,
            "archival_only": True,
            "new_physics_was_run": False,
            "note": "All physical frames are copied unchanged from existing v1 recorded MuJoCo viewer data"
        },
    }
    (temporary_root / "docs" / replay_name).write_text(
        json.dumps(replay, ensure_ascii=False, separators=(",", ":")) + "\n", encoding="utf-8")
    job = {
        "schema": pipeline.SCHEMA,
        "id": ID,
        "title": "Mini Wheelbot · архивный тест pipeline",
        "detail": "9,951 с · существующая MuJoCo-запись · новый расчёт не выполнялся",
        "robot": "Mini Wheelbot (approximate v1 reconstruction)",
        "engine": "MuJoCo",
        "status": "research",
        "duration_s": original["duration_s"],
        "commands": {"simulate": [sys.executable, "-c", "raise SystemExit('archive only')"],
                     "export": [sys.executable, "-c", "raise SystemExit('archive only')"]},
        "trace": {"path": raw_name, "samples_field": "samples",
                  "time_field": "t", "qpos_field": "qpos"},
        "replay": {"path": replay_name,
                   "format": original["format"],
                   "trace_sha256_field": "verification.raw_trace_sha256"},
        "player": {"template": TEMPLATE,
                   "replacements": {"Hexapod Lab · Mini Wheelbot · вставание":
                                    "Hexapod Lab · Mini Wheelbot · архивный тест"}},
    }
    return job, original


def run_smoke(source_root: Path, temporary_root: Path) -> dict:
    job, original = assemble(source_root, temporary_root)
    receipt, _ = pipeline.package(temporary_root, job)
    saved = json.loads((temporary_root / "docs" / job["replay"]["path"]).read_text())
    pipeline.require(saved["frames"] == original["frames"], "Archived physics frames were mutated")
    entries = json.loads((temporary_root / pipeline.CATALOG).read_text())["entries"]
    pipeline.require(any(item["id"] == ID and item["status"] == "research" for item in entries),
                     "Replay missing from isolated library")
    return {"status": "ARCHIVAL_PIPELINE_SMOKE_PASS", "physical_run_executed": False,
            "new_replay_published": False, "source_frames": len(original["frames"]),
            "source_blob_sha1": SOURCE_GIT_BLOB, **receipt}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--output-dir", type=Path,
                    help="Keep isolated catalog/page assets here; never push to GitHub")
    args = ap.parse_args()
    if args.output_dir:
        path = args.output_dir.resolve()
        pipeline.require(not path.exists(), "Output must be a new directory")
        path.mkdir(parents=True)
        result = run_smoke(ROOT, path)
    else:
        with tempfile.TemporaryDirectory(prefix="wheelbot-replay-proof-") as temp:
            result = run_smoke(ROOT, Path(temp))
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
