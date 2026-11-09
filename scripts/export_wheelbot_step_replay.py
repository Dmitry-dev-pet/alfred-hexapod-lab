#!/usr/bin/env python3
"""Convert REAL Mini Wheelbot V2 STEP-axis MuJoCo qpos to existing Android format.

Requires both raw roll_*.json AND the run summary.json produced by the pinned
research/mini_wheelbot_v2/simulate_step_source_roll.py from PR #86.
No interpolation, invented wheel angles, or new physical simulation here.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

COMMIT = "73815e49371c5fb5f49eec59d364e4d9d1b57b10"
EXPECTED_CLASS = "EXPERIMENTAL_STEP_AXIS_ORIGINAL_OPEN_LOOP_NOT_HARDWARE_VALIDATED"
OUT_FORMAT = "mini-wheelbot-mujoco-physical-replay-v1"
COLUMNS = [
    "time_s", "root_xyz_m_3", "root_quat_wxyz_4",
    "drive_joint_rad", "reaction_joint_rad",
    "reaction_omega_rad_s", "reaction_command_Nm"
]


def need(cond: bool, explanation: str) -> None:
    if not cond:
        raise ValueError(explanation)


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def finite(v) -> bool:
    return type(v) in (int, float) and math.isfinite(v)


def export(trace_path: Path, summary_path: Path, output_path: Path) -> dict:
    """Fail closed unless each exported angle is a recorded physical coordinate."""
    source_bytes = trace_path.read_bytes()
    summary_bytes = summary_path.read_bytes()
    source = json.loads(source_bytes)
    summary = json.loads(summary_bytes)
    need(summary.get("classification") == EXPECTED_CLASS and
         summary.get("upstream_commit") == COMMIT,
         "Unpinned or unverified physical experiment summary")
    receipts = [r for r in summary.get("runs", [])
                if r.get("frames_file") == trace_path.name]
    need(len(receipts) == 1 and receipts[0].get("frames_sha256") == sha(source_bytes),
         "Source trace bytes do not match physics run receipt")
    info, rows = source.get("info"), source.get("frames")
    need(isinstance(info, dict) and isinstance(rows, list) and len(rows) >= 3,
         "Missing original physical samples")
    need(info.get("mode") == "upstream" and info.get("start_roll_deg") == 90 and
         info.get("speed_cap_assumed") is True and
         info.get("numerical_warning") is None,
         "The selected Wheelbot physical run is not the capped 90-degree upstream trial")
    need(info.get("meets_provisional_up_criterion") is False and
         info.get("motor_limits_hardware_validated") is False and
         info.get("source_joint_label_parity_validated") is False,
         "This experimental replay must not claim calibrated successful self-righting")
    need(info.get("status") == "PROVISIONAL_GEOMETRY_MOTOR_AND_CONTROLLER_MAPPING",
         "Unexpected result classification")
    need(info.get("simulated_duration_s") and finite(info["simulated_duration_s"]),
         "Missing measured duration")
    result = []
    prev = None
    for row in rows:
        # [time, qpos xyz(3) quat(4) drive_joint(1) reaction_joint(1),
        #  actual_tilt, actual_reaction_omega, actual_torque, contact_count]
        need(isinstance(row, list) and len(row) == 14 and
             all(finite(v) for v in row), "Invalid full physical state frame")
        t = float(row[0])
        need(prev is None or t > prev, "Nonmonotonic raw frame clock")
        prev = t
        qw, qx, qy, qz = row[4:8]
        need(abs(qw*qw + qx*qx + qy*qy + qz*qz - 1) < 0.005,
             "Invalid recorded root quaternion")
        need(row[13] >= 0 and type(row[13]) is int, "Invalid recorded contact count")
        # Exactly 12 fields consumed by the existing Wheelbot V1 renderer.
        # Both joint angles come from the actual MuJoCo qpos vector, not estimates.
        new_row = row[:10] + [row[11], row[12]]
        need(len(new_row) == 12, "Unexpected Wheelbot renderer frame shape")
        result.append(new_row)
    measured_duration = round(float(rows[-1][0]), 3)
    need(abs(measured_duration - float(info["simulated_duration_s"])) < 0.06,
         "Incomplete or truncated physical simulation")
    duration = float(info["simulated_duration_s"])
    output = {
        "format": OUT_FORMAT,
        "title": "Mini Wheelbot V2 · source torque pulse · experimental failure",
        "robot": "Mini Wheelbot V2 — preliminary STEP-axes MuJoCo reconstruction",
        "physics_engine": "MuJoCo",
        "source_repo": "wheelbot/Mini-Wheelbot",
        "model_xml_sha256": summary.get("source_model_xml_sha256"),
        "description": "Replay of real recorded MuJoCo free-base qpos and two wheel joints, not a hardware-proven digital twin.",
        "limitations": [
            "Original upstream controller open-loop pulse only; source closed-loop AMPC was not ported",
            "Collision and COM geometry assumptions are not hardware calibrated",
            "Motor torque and speed limits are assumptions",
            "Self-righting did not pass the sustained-upright physical criterion",
            "Original commercial robot performance cannot be inferred from this negative model"
        ],
        "sample_stride": 12,
        "replay_hz": 20,
        "duration_s": duration,
        "frame_columns": COLUMNS,
        "frame_count": len(result),
        "frames": result,
        "status": "failed",
        "source": {
            "upstream_commit": COMMIT,
            "trace_sha256": sha(source_bytes),
            "research_summary_sha256": sha(summary_bytes),
            "trace_name": trace_path.name,
            "physics_steps_reported_s": duration,
            "experimental_status": info["status"],
            "min_tilt_deg": info.get("min_tilt_deg"),
            "final_tilt_deg": info.get("final_tilt_deg"),
            "longest_upright_s": info.get("longest_upright_s")
        }
    }
    payload = (json.dumps(output, ensure_ascii=False, separators=(",", ":"),
                          allow_nan=False) + "\n").encode("utf-8")
    if output_path.exists():
        need(output_path.read_bytes() == payload,
             "Immutable Android replay already exists with different content")
    else:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_bytes(payload)
    return {"frames": len(result), "duration_s": duration, "status": "failed",
            "trace_sha256": sha(source_bytes), "output_sha256": sha(payload)}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--trace", type=Path, required=True)
    ap.add_argument("--summary", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()
    print(json.dumps(export(args.trace, args.summary, args.out), indent=2))


if __name__ == "__main__":
    main()
