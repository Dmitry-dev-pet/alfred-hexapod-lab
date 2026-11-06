#!/usr/bin/env python3
"""Cross-machine CPU performance benchmark of the original 60 Lite3 S2 episodes.

Runs the existing S2 robustness code unchanged, including physical per-step
foot-contact audit. Does NOT train, actuate hardware, or invoke GitHub Actions.
Run in a pinned Python environment with the owner's private original_assets.
"""
from __future__ import annotations

import argparse
import json
import os
import platform
import resource
import statistics
import sys
import time
from pathlib import Path

import mujoco
import numpy as np
import onnxruntime as ort

import lite3_onnx_joint_audit as base
import lite3_s2_robustness as s2

POLICY_SHA = "773a1bbd0fee45ec38604d979d5fd6fb339ae15d13bfb5d6e2056a171d2a3422"
SCENE_SHA = "f054f2ef66eb53af08120040e41e3aded4826a00f794669f556430086c0e52d7"


def run(*, seed_start: int = 20261106, seeds: int = 10, baseline_only: bool = False, output: Path):
    if not 1 <= seeds <= 50:
        raise ValueError("seeds must be 1..50")
    if mujoco.__version__ != "3.14.0" or ort.__version__ != "1.23.2":
        raise RuntimeError("Benchmark requires MuJoCo 3.14.0 and ONNX Runtime 1.23.2")
    if base.sha256(base.ONNX) != POLICY_SHA or base.sha256(base.SCENE) != SCENE_SHA:
        raise RuntimeError("Private assets do not match the pinned benchmark model/policy")

    variations = s2.VARIATIONS[:1] if baseline_only else list(s2.VARIATIONS)
    host = {
        "system": platform.system(),
        "machine": platform.machine(),
        "processor": platform.processor(),
        "logical_cpus": os.cpu_count(),
        "cgroup_cpu_max": (
            Path("/sys/fs/cgroup/cpu.max").read_text().strip()
            if Path("/sys/fs/cgroup/cpu.max").exists() else None
        ),
        "cgroup_memory_max": (
            Path("/sys/fs/cgroup/memory.max").read_text().strip()
            if Path("/sys/fs/cgroup/memory.max").exists() else None
        ),
    }
    t = time.perf_counter()
    policy, joints, _ = base.make_policy()
    session_creation_seconds = time.perf_counter() - t

    # Warm up the exact code path, exclude startup and warmup from throughput.
    t = time.perf_counter()
    s2.run_perturbed(policy, joints, seed_start, variations[0])
    warmup_seconds = time.perf_counter() - t

    rows = []
    start = time.perf_counter()
    for variation in variations:
        for seed in range(seed_start, seed_start + seeds):
            t = time.perf_counter()
            data, _trace = s2.run_perturbed(policy, joints, seed, variation)
            duration = time.perf_counter() - t
            assert data["physics_contact_audit"]["physics_steps"] == 2375
            rows.append({
                "variation": variation.name,
                "seed": seed,
                "wall_seconds": duration,
                "torque_saturation_ticks": data["torque_saturation_ticks"],
                "max_tilt_deg": data["max_tilt_deg"],
                "fall": data["first_fall"] is not None,
                "net_root_m": data["net_root_m"],
                "fraction_slip_gt_005_m_s": data["physics_contact_audit"][
                    "fraction_loaded_contact_samples_speed_gt_005_m_s"
                ],
            })
            print(f"{variation.name:24s} {seed} {duration:.3f}s "
                  f"sat={data['torque_saturation_ticks']}", flush=True)
    wall_seconds = time.perf_counter() - start

    # Linux getrusage.ru_maxrss reports KiB; macOS reports bytes.
    rss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    peak_rss_bytes = int(rss if sys.platform == "darwin" else rss * 1024)
    simulated_seconds = 9.5 * len(rows)
    aggregate = {
        "n": len(rows),
        "sim_seconds": simulated_seconds,
        "physics_steps": 2375 * len(rows),
        "wall_seconds": wall_seconds,
        "sim_seconds_per_wall_second": simulated_seconds / wall_seconds,
        "physics_steps_per_wall_second": 2375 * len(rows) / wall_seconds,
        "per_episode_wall_median": statistics.median(r["wall_seconds"] for r in rows),
        "per_episode_wall_p95": sorted(r["wall_seconds"] for r in rows)[
            max(0, int(len(rows) * 0.95) - 1)
        ],
        "peak_rss_bytes": peak_rss_bytes,
    }
    result = {
        "benchmark": "XGO-Lite3 S2 6x10 CPU episodes, serial, full contact audit",
        "original_policy_sha256": POLICY_SHA,
        "original_scene_sha256": SCENE_SHA,
        "environment": {
            "host": host,
            "versions": {
                "python": sys.version.split()[0],
                "mujoco": mujoco.__version__,
                "onnxruntime": ort.__version__,
                "numpy": np.__version__,
            },
        },
        "control": {
            "physics_hz": 250, "policy_hz": 50, "episode_sim_s": 9.5,
            "seeds": [seed_start, seed_start + seeds - 1],
            "variations": [v.__dict__ for v in variations],
        },
        "policy_session_creation_seconds": session_creation_seconds,
        "warmup_seconds_excluded": warmup_seconds,
        "runs": rows,
        "aggregate": aggregate,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print("AGGREGATE " + json.dumps(aggregate, sort_keys=True), flush=True)
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--seeds", type=int, default=10)
    p.add_argument("--start-seed", type=int, default=20261106)
    p.add_argument("--only-baseline", action="store_true")
    p.add_argument("--output", type=Path, default=Path("perf_summary.json"))
    opts = p.parse_args()
    run(seed_start=opts.start_seed, seeds=opts.seeds,
        baseline_only=opts.only_baseline, output=opts.output)
