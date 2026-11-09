#!/usr/bin/env python3
"""Headless, evidence-first MicroDuck CPU MuJoCo + original ONNX probe.

This is an independent research experiment. It uses the *official* upstream
MJCF and PolicyInference observation/action implementation, not EN01 geometry.
The default XML position motors are a diagnostic approximation of BAM.
All outcomes, including failures, are written as review-only JSON.
"""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys

import mujoco
import numpy as np

UPSTREAM_COMMIT = "bae39f5900b318f7e87fc485c30a31213cfa14eb"
POLICY_REVISION = "v8"
POLICY_SHA256 = "2fc7f8ee6acd7d0d3f66a99927b5d5a56589aeed02ec714de329b252d5ccf492"
CONTROL_DT = 0.020
DT = 0.005
CASES = ("stand", "sit", "fold", "roll180", "pitch180", "side90", "push")


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def upstream_inference(rl_root):
    source = rl_root / "scripts" / "infer_policy.py"
    spec = importlib.util.spec_from_file_location("microduck_upstream_infer_policy", source)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot import official inference script: {source}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def foot_on_floor(model, data):
    for i in range(data.ncon):
        contact = data.contact[i]
        a = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1) or ""
        b = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2) or ""
        if ("floor" in (a, b)) and ("foot" in a or "foot" in b):
            return True
    return False


def sample(model, data, root_qpos, root_dof):
    q = np.asarray(data.qpos[root_qpos + 3:root_qpos + 7], dtype=float)
    r = np.empty(9, dtype=float)
    mujoco.mju_quat2Mat(r, q)
    tilt = math.degrees(math.acos(float(np.clip(r[8], -1.0, 1.0))))
    root_z = float(data.qpos[root_qpos + 2])
    speed = float(np.linalg.norm(data.qvel[root_dof:root_dof + 3]))
    rate = float(np.linalg.norm(data.qvel[root_dof + 3:root_dof + 6]))
    return {
        "trunk_z_m": root_z,
        "tilt_deg": tilt,
        "linear_speed_mps": speed,
        "angular_speed_rps": rate,
        "foot_contact": foot_on_floor(model, data),
        "qpos": np.asarray(data.qpos).round(9).tolist(),
        "qvel": np.asarray(data.qvel).round(9).tolist(),
        "ctrl": np.asarray(data.ctrl).round(9).tolist(),
    }


def is_stable(frame):
    return (
        0.09 <= frame["trunk_z_m"] <= 0.19
        and frame["tilt_deg"] <= 20.0
        and frame["linear_speed_mps"] <= 0.25
        and frame["angular_speed_rps"] <= 1.0
        and frame["foot_contact"]
    )


def run_case(args, upstream, name):
    # Upstream default scene includes robot_groundcontact.xml, not robot_walk.xml.
    model = mujoco.MjModel.from_xml_path(str(args.scene))
    model.opt.timestep = DT
    data = mujoco.MjData(model)
    key = "STAND" if name in ("stand", "push", "roll180", "pitch180", "side90") else name.upper()
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, key)
    if key_id < 0:
        raise ValueError(f"Missing official keyframe: {key}")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    free_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "trunk_base_freejoint")
    root_qpos = int(model.jnt_qposadr[free_id])
    root_dof = int(model.jnt_dofadr[free_id])
    joint_indices = [
        int(model.jnt_qposadr[model.actuator_trnid[i, 0]])
        for i in range(model.nu)
    ]
    if name in ("roll180", "pitch180", "side90"):
        # Controlled synthetic drop from 18 cm: rotate the body, settle under
        # gravity with motors holding the starting joint pose, THEN enable RL.
        orientations = {
            "roll180": (0.0, 1.0, 0.0, 0.0),
            "pitch180": (0.0, 0.0, 1.0, 0.0),
            "side90": (math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0),
        }
        data.qpos[root_qpos + 2] = 0.18
        data.qpos[root_qpos + 3:root_qpos + 7] = orientations[name]
        data.qvel[:] = 0
        data.ctrl[:] = [data.qpos[i] for i in joint_indices]
        mujoco.mj_forward(model, data)
        for _ in range(round(0.6 / DT)):
            mujoco.mj_step(model, data)

    # Use the official observation ordering, last-action semantics and
    # 61→14 ONNX policy adapter; do not reimplement the trained policy.
    policy = upstream.PolicyInference(
        model, data,
        walking_onnx_path=str(args.policy),
        use_projected_gravity=True,
        new_cmd_obs=True,
    )
    if policy.get_observations().shape != (61,) or model.nu != 14:
        raise RuntimeError("Policy contract mismatch: expected 61 observations/14 actuators")
    policy.set_vel_cmd(0.0, 0.0, 0.0)
    mujoco.mj_forward(model, data)

    frames = []
    count = round(args.seconds / CONTROL_DT)
    for step in range(count):
        if name == "push" and step == round(1.0 / CONTROL_DT):
            # A synthetic velocity perturbation, not a calibrated contact force.
            data.qvel[root_dof] = 0.7
        action = policy.infer()
        policy.apply_action(action)
        for _ in range(round(CONTROL_DT / DT)):
            mujoco.mj_step(model, data)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise RuntimeError(f"Numerical instability in {name} at step {step}")
        frames.append({"t": round((step + 1) * CONTROL_DT, 6),
                       **sample(model, data, root_qpos, root_dof)})

    # Continuous one-second gate, measured AFTER perturbation for push.
    gate_frames = round(1.0 / CONTROL_DT)
    eligible = [f for f in frames if f["t"] > (1.0 if name == "push" else 0.0)]
    time_to_recover = None
    for j in range(gate_frames - 1, len(eligible)):
        if all(is_stable(f) for f in eligible[j - gate_frames + 1:j + 1]):
            time_to_recover = eligible[j]["t"] - (1.0 if name == "push" else 0.0)
            break
    stable_at_end = len(eligible) >= gate_frames and all(
        is_stable(f) for f in eligible[-gate_frames:]
    )
    warnings = {str(i): int(s.number) for i, s in enumerate(data.warning) if s.number}
    return {
        "case": name,
        "status": "PASS" if stable_at_end else "FAIL",
        "time_to_first_1s_stable_s": time_to_recover,
        "stable_at_end_1s": stable_at_end,
        "warnings": warnings,
        "frames": frames,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rl-root", type=Path, required=True)
    parser.add_argument("--policy", type=Path, required=True)
    parser.add_argument("--scene", type=Path, default=None)
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument("--output", type=Path, default=Path("results/microduck/standup-cpu.json"))
    args = parser.parse_args()
    args.rl_root = args.rl_root.resolve()
    args.policy = args.policy.resolve()
    if args.scene is None:
        args.scene = args.rl_root / "src/mjlab_microduck/robot/microduck/scene.xml"
    args.scene = args.scene.resolve()
    if not 2.0 <= args.seconds <= 30.0:
        parser.error("--seconds must be between 2 and 30")
    if sha256(args.policy) != POLICY_SHA256:
        raise RuntimeError("ONNX checksum mismatch; refusing unpinned policy")
    upstream = upstream_inference(args.rl_root)
    cases = [run_case(args, upstream, name) for name in CASES]
    report = {
        "schema": "microduck-cpu-benchmark-v1",
        "publication_policy": "review_only",
        "model_repo": "pollen-robotics/microduck_rl",
        "model_commit": UPSTREAM_COMMIT,
        "scene": str(args.scene.relative_to(args.rl_root)),
        "scene_sha256": sha256(args.scene),
        "policy_repo": "pollen-robotics/microduck-policies",
        "policy_revision": POLICY_REVISION,
        "policy_sha256": POLICY_SHA256,
        "mujoco_version": mujoco.__version__,
        "physics": "CPU MuJoCo; official ground-contact MJCF; approximate XML position actuators (NOT BAM)",
        "sample_rate_hz": 50,
        "seconds_per_case": args.seconds,
        "thresholds": {"trunk_z_m": [0.09, 0.19], "max_tilt_deg": 20,
                       "max_linear_speed_mps": 0.25, "max_angular_speed_rps": 1.0,
                       "continuous_stability_s": 1.0, "foot_floor_contact": True},
        "cases": cases,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, separators=(",", ":")) + "\n")
    for case in cases:
        print(f"{case['case']:>9}: {case['status']}, 1s-stable={case['time_to_first_1s_stable_s']}")
    print(f"review_only: {args.output} ({args.output.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
