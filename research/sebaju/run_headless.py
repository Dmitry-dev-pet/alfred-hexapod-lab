#!/usr/bin/env python3
"""Reproduce upstream SeBaJu dynamics without opening its interactive viewer.

The upstream MJCF/controller are NOT vendored. Simulation steps are performed
inside the original controller; this adapter replaces only viewer and sleep,
records measured MuJoCo state, and applies an optional physical roll torque.
"""
from __future__ import annotations

import argparse
import hashlib
import contextlib
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import time
from unittest.mock import patch

UPSTREAM_URL = "https://github.com/k3nkreate/SeBaJu_BOT_5845.git"
UPSTREAM_SHA = "7bbc38f4b26b8ecce7ac0ee37e06164e851dc600"
CONTROLLER = "SeBaJu_Ctrl_BalJumpAchieve.py"
MODEL = "SeBaJu_BOT(II).xml"
PHYSICS_VERSION = "3.14.0"


UPSTREAM_FILE_BLOBS = {
    MODEL: "c0abe8d5f9663813e2782fbd44fb38c40d7ea048",
    CONTROLLER: "42397bb5e613a12ecbe6074030bb917f3d03533a",
}


def git_blob_sha(raw: bytes) -> str:
    return hashlib.sha1(b"blob " + str(len(raw)).encode() + b"\0" + raw).hexdigest()


def require_upstream(path: Path, offline_snapshot: bool = False) -> Path:
    """Require pinned git checkout or an exact two-file GitHub blob verified snapshot.

    Offline mode exists for sandboxed Python/CPU workers with no git network.
    It verifies *both* model and controller byte-for-byte; never trusts a
    merely user-supplied commit name or a directory name as provenance.
    """
    path = path.expanduser().resolve()
    if offline_snapshot:
        for name, expected in UPSTREAM_FILE_BLOBS.items():
            got = git_blob_sha((path / name).read_bytes())
            if got != expected:
                raise RuntimeError(f"Offline SeBaJu blob mismatch: {name}: {got} != {expected}")
        return path
    if not path.is_dir():
        path.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(["git", "clone", "--no-checkout", UPSTREAM_URL, str(path)], check=True)
        subprocess.run(["git", "-C", str(path), "checkout", "--detach", UPSTREAM_SHA], check=True)
    got = subprocess.check_output(
        ["git", "-C", str(path), "rev-parse", "HEAD"], text=True
    ).strip()
    if got != UPSTREAM_SHA:
        raise RuntimeError(f"Unexpected SeBaJu revision: {got}; expected {UPSTREAM_SHA}")
    dirty = subprocess.run(
        ["git", "-C", str(path), "diff", "--quiet", "HEAD", "--", MODEL, CONTROLLER],
        check=False,
    ).returncode
    if dirty != 0:
        raise RuntimeError("Upstream MuJoCo model/controller were modified locally")
    for name in (CONTROLLER, MODEL):
        if not (path / name).is_file():
            raise FileNotFoundError(path / name)
    return path


def whole_robot_airborne(t: float, wheel_gap_m: float, ground_contact_geoms: list[str]) -> bool:
    """Jump only: both wheels clear AND no robot part contacts the floor.

    The bounded window prevents a post-tip body resting on the floor from
    being incorrectly classified as an hours-long jump just because wheels
    are off the ground.
    """
    return 5.0 <= t < 9.0 and wheel_gap_m >= 0.005 and not ground_contact_geoms


def longest_true_interval(records: list[dict], field: str) -> tuple[int, int]:
    """Return inclusive-start/exclusive-end indices of longest contiguous run."""
    best = (0, 0)
    start = None
    for i in range(len(records) + 1):
        active = i < len(records) and bool(records[i][field])
        if active and start is None:
            start = i
        if not active and start is not None:
            if i - start > best[1] - best[0]:
                best = (start, i)
            start = None
    return best


def analyze(records: list[dict], dt: float, trial: str, tip_start: float) -> dict:
    if not records:
        raise ValueError("No physical MuJoCo observations")
    flight = longest_true_interval(records, "in_flight")
    a, b = flight
    jump_start = records[a]["t"] if b > a else None
    jump_end = records[b - 1]["t"] + dt if b > a else None
    touchdown = next((s["t"] for s in records[b:] if all(s["grounded"])), None) if b > a else None
    recovered_at = None
    stable_steps = 0
    for s in records:
        if touchdown is None or s["t"] < touchdown:
            continue
        stable = s["tilt_deg"] < 10 and all(s["grounded"]) and s["wheel_speed"] < 0.5
        stable_steps = stable_steps + 1 if stable else 0
        if stable_steps * dt >= 0.3:
            recovered_at = s["t"]
            break
    max_clearance = max(s["wheel_gap_min_m"] for s in records if s["t"] >= 5.0)
    summary = {
        "jump_detected": (b - a) * dt >= 0.020 and max_clearance >= 0.005,
        "takeoff_s": jump_start,
        "flight_duration_s": round((b - a) * dt, 4),
        "max_min_wheel_clearance_m": round(max_clearance, 5),
        "touchdown_s": touchdown,
        "post_landing_stable_s": recovered_at,
        "landing_recovery_observed": recovered_at is not None,
        "final_tilt_deg": round(records[-1]["tilt_deg"], 2),
        "max_tilt_deg": round(max(s["tilt_deg"] for s in records), 2),
    }
    if trial == "tip":
        tipped = next((i for i, s in enumerate(records)
                       if s["t"] >= tip_start and s["tilt_deg"] >= 60), None)
        upright_since = 0
        righted_at = None
        if tipped is not None:
            for s in records[tipped + 1:]:
                stable = s["tilt_deg"] <= 10 and all(s["grounded"]) and s["wheel_speed"] < 0.5
                upright_since = upright_since + 1 if stable else 0
                if upright_since * dt >= 0.5:
                    righted_at = s["t"]
                    break
        summary.update({
            "physical_tip_observed": tipped is not None,
            "tip_detected_s": records[tipped]["t"] if tipped is not None else None,
            "self_righted": righted_at is not None if tipped is not None else None,
            "self_righted_at_s": righted_at,
            "tip_assessment": "measured" if tipped is not None else "inconclusive-no-tip",
        })
    return summary


class FakeCamera:
    """Just the camera properties used by the original GUI loop."""
    def __init__(self):
        self.lookat = [0.0, 0.0, 0.0]


class PhysicalRecorder:
    """Passive-viewer-compatible hook; the source controller owns mj_step."""
    def __init__(self, model, data, args):
        import mujoco
        import numpy as np

        self.mujoco = mujoco
        self.np = np
        self.model, self.data, self.args = model, data, args
        self.cam = FakeCamera()
        self.dt = float(model.opt.timestep)
        self.ground = model.geom("ground").id
        self.wheels = [model.geom(x).id for x in ("left_wheel_geom", "right_wheel_geom")]
        self.torso = model.body("torso").id
        self.joints = [model.joint(x).id for x in
                       ("left_hip", "left_knee", "right_hip", "right_knee")]
        self.wheel_joints = [model.joint(x).id for x in
                             ("left_wheel_joint", "right_wheel_joint")]
        self.records = []
        self.frames = []
        self.next_frame = 0.0
        self.steps = 0
        self.tip_ticks = 0

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.data.xfrc_applied[self.torso, :] = 0
        return False

    def is_running(self):
        return float(self.data.time) < self.args.duration - self.dt * 0.5

    def sync(self):
        m, d, mj, np = self.model, self.data, self.mujoco, self.np
        t = float(d.time)
        self.steps += 1
        # Applied on the NEXT mj_step by the upstream controller. Never teleport.
        d.xfrc_applied[self.torso, :] = 0
        if self.args.trial == "tip" and self.args.tip_start <= t < self.args.tip_start + self.args.tip_seconds:
            d.xfrc_applied[self.torso, 3] = self.args.tip_torque
            self.tip_ticks += 1

        wheel_centres = [d.geom_xpos[g].tolist() for g in self.wheels]
        gap = min(c[2] - 0.060 for c in wheel_centres)
        grounded = [False, False]
        normal_n = [0.0, 0.0]
        ground_contact_geoms = []
        for ci in range(d.ncon):
            contact = d.contact[ci]
            pair = {int(contact.geom1), int(contact.geom2)}
            if self.ground not in pair:
                continue
            other = next(iter(pair - {self.ground}), None)
            if other is not None:
                ground_contact_geoms.append(mj.mj_id2name(m, mj.mjtObj.mjOBJ_GEOM, other))
            for i, wheel in enumerate(self.wheels):
                if wheel in pair:
                    grounded[i] = True
                    force = np.zeros(6, dtype=float)
                    mj.mj_contactForce(m, d, ci, force)
                    normal_n[i] += abs(float(force[0]) * float(contact.frame[2]))
        z_axis = float(d.xmat[self.torso].reshape(3, 3)[2, 2])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, z_axis))))
        speeds = [abs(float(d.qvel[m.jnt_dofadr[j]])) for j in self.wheel_joints]
        if not (math.isfinite(tilt) and math.isfinite(gap) and
                np.all(np.isfinite(d.qpos)) and np.all(np.isfinite(d.qvel))):
            raise FloatingPointError(f"Nonfinite state at t={t}")
        observation = {
            "t": round(t, 6), "tilt_deg": tilt, "wheel_gap_min_m": gap,
            "grounded": grounded, "wheel_speed": sum(speeds) / 2,
            # A wheel in the air is NOT full-body flight when a tipped
            # torso/leg rests on the floor. Limit detection to source jump.
            "in_flight": whole_robot_airborne(t, gap, ground_contact_geoms),
        }
        self.records.append(observation)
        if t + self.dt / 2 < self.next_frame:
            return
        self.next_frame += 1 / self.args.fps
        self.frames.append({
            "t": round(t, 6),
            "qpos": [round(float(x), 8) for x in d.qpos],
            "torso_xyz": [round(float(x), 8) for x in d.xpos[self.torso]],
            "torso_wxyz": [round(float(x), 8) for x in d.xquat[self.torso]],
            "joint_anchors_xyz": [[round(float(x), 8) for x in d.xanchor[j]]
                                  for j in self.joints],
            "wheel_centres_xyz": [[round(float(x), 8) for x in xyz]
                                  for xyz in wheel_centres],
            "grounded": grounded,
            "ground_contact_geoms": sorted(set(ground_contact_geoms)),
            "wheel_vertical_normal_N": [round(x, 4) for x in normal_n],
            "wheel_gap_min_m": round(gap, 6),
            "tilt_deg": round(tilt, 3),
        })


def run(args):
    import mujoco
    import mujoco.viewer

    if mujoco.__version__ != PHYSICS_VERSION:
        raise RuntimeError(f"Need MuJoCo {PHYSICS_VERSION}, found {mujoco.__version__}")
    path = require_upstream(args.upstream, args.offline_snapshot)
    spec = importlib.util.spec_from_file_location("original_sebaju_controller", path / CONTROLLER)
    controller = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(controller)
    if not controller.JUMP_ENABLE or controller.XML_FILE != MODEL:
        raise RuntimeError("Unexpected upstream controller configuration")
    controller.SLOW_FACTOR = 0.0

    args.out.mkdir(parents=True, exist_ok=True)
    recorders = []
    def passive(model, data):
        recorder = PhysicalRecorder(model, data, args)
        recorders.append(recorder)
        return recorder

    with (args.out / "source_controller.log").open("w", encoding="utf-8") as log:
        with patch.object(mujoco.viewer, "launch_passive", passive), \
             patch.object(time, "sleep", lambda _: None), \
             contextlib.redirect_stdout(log):
            controller.run()
    if len(recorders) != 1:
        raise RuntimeError(f"Expected one actual upstream simulation, got {len(recorders)}")
    observed = recorders[0]
    summary = analyze(observed.records, observed.dt, args.trial, args.tip_start)
    data = {
        "schema": "alfred-sebaju-mujoco-physical-trace/v1",
        "source_repo": UPSTREAM_URL,
        "source_commit": UPSTREAM_SHA,
        "controller": CONTROLLER,
        "model": MODEL,
        "engine": "MuJoCo", "engine_version": mujoco.__version__,
        "trial": args.trial, "duration_s": round(float(observed.data.time), 6),
        "physics_dt_s": observed.dt, "fps_target": args.fps,
        "steps": observed.steps, "frames": observed.frames,
        "joint_anchor_names": ["left_hip", "left_knee", "right_hip", "right_knee"],
        "wheel_names": ["left", "right"],
        "assumptions": [
            "Original public upstream model/controller; not validated on real hardware",
            "Physical observations captured after source controller mj_step",
            "Source controller output recorded separately; frame sampling is not interpolation",
            "No redistribution of upstream MJCF or source controller",
        ],
    }
    if args.trial == "tip":
        data["disturbance"] = {
            "kind": "world-frame roll torque on torso",
            "start_s": args.tip_start,
            "duration_s": args.tip_seconds,
            "torque_Nm": args.tip_torque,
            "applied_steps": observed.tip_ticks,
        }
    (args.out / "physical_trace.json").write_text(
        json.dumps(data, ensure_ascii=False, separators=(",", ":")) + "\n",
        encoding="utf-8",
    )
    receipt = {
        "schema": "alfred-sebaju-experiment-receipt/v1",
        "source_commit": UPSTREAM_SHA,
        "engine": mujoco.__version__,
        "trial": args.trial,
        "steps": observed.steps,
        "frames": len(observed.frames),
        "physics_dt_s": observed.dt,
        "duration_s": data["duration_s"],
        "summary": summary,
        "third_party_code_vendored": False,
        "source_verification": "pinned-git-checkout" if not args.offline_snapshot else "two-pinned-github-blobs",
        "result_status": "measured-simulation-only",
    }
    (args.out / "receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps(receipt, ensure_ascii=False, indent=2))
    return receipt


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--trial", choices=["jump", "tip"], default="jump")
    p.add_argument("--upstream", type=Path, default=Path.home() / ".cache/alfred-hexapod/sebaju-source")
    p.add_argument("--offline-snapshot", action="store_true", help="Verify pinned source files by git blob SHA, no git network")
    p.add_argument("--out", type=Path, default=Path(".cache/sebaju/jump"))
    p.add_argument("--duration", type=float, default=None)
    p.add_argument("--fps", type=int, default=50)
    p.add_argument("--tip-start", type=float, default=12.0)
    p.add_argument("--tip-seconds", type=float, default=0.15)
    p.add_argument("--tip-torque", type=float, default=8.0)
    args = p.parse_args()
    args.duration = args.duration or (18.0 if args.trial == "tip" else 13.0)
    if not (8 <= args.duration <= 60 and 1 <= args.fps <= 100):
        p.error("duration must be 8-60 s, fps must be 1-100")
    if args.trial == "tip" and not (8 <= args.tip_start < args.duration - 1
                                   and 0 < args.tip_seconds <= 1
                                   and 0 < args.tip_torque <= 30):
        p.error("invalid physical tipping experiment parameters")
    run(args)


if __name__ == "__main__":
    main()
