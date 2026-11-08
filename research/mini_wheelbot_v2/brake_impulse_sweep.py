#!/usr/bin/env python3
"""Sensitivity to reverse (often called 'braking') pulse in Mini Wheelbot V2.

Runs actual MuJoCo mj_step in the preliminary STEP-axis scene. Only reverse
pulse duration and an EARLY, NOT-SOURCE-VALIDATED source-gain LQR handoff are
varied. The source C++ default is 1.5 s reverse pulse. For publication and
cross-check we compare 0.3 s vs original and a gain .5 handoff at 5.0 s.
Do not mistake this as an optimized or hardware-safe robot controller.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path

from audit_original_cad import acquire
from audit_step_axes import audit as audit_step
from simulate_step_source_roll import run_one

CASES = [
    # Reverse pulse duration, mode, feedback start s, experimental gain.
    (0.0, "upstream", 6.1, 1.0),
    (0.15, "upstream", 6.1, 1.0),
    (0.20, "upstream", 6.1, 1.0),
    (0.25, "upstream", 6.1, 1.0),
    (0.30, "upstream", 6.1, 1.0),
    (0.35, "upstream", 6.1, 1.0),
    (0.40, "upstream", 6.1, 1.0),
    (0.45, "upstream", 6.1, 1.0),
    (0.60, "upstream", 6.1, 1.0),
    (0.75, "upstream", 6.1, 1.0),
    (0.90, "upstream", 6.1, 1.0),
    (1.05, "upstream", 6.1, 1.0),
    (1.20, "upstream", 6.1, 1.0),
    (1.50, "upstream", 6.1, 1.0),
]
for start in (4.9, 5.0, 5.1, 5.2):
    for gain in (0.25, 0.50, 1.0):
        CASES.append((0.30, "upstream_lqr_inverted", start, gain))


def run(cache: Path, steps: Path, out: Path):
    verified = acquire(cache)
    step = audit_step(steps)
    out.mkdir(parents=True, exist_ok=True)
    entries = []
    for pulse, mode, start, gain in CASES:
        r = run_one(
            cache, verified["params"], step, start_roll=90, mode=mode,
            speed_cap=True, duration=8.0, contact_profile="stiff",
            reverse_pulse_s=pulse, lqr_start_s=start, feedback_gain=gain,
        )
        frames = r.pop("frames")
        name = f"pulse_{pulse:.2f}_{mode}_start_{start:.2f}_gain_{gain:.2f}.json"
        path = out / name
        path.write_text(json.dumps({"info": r, "frames": frames},
                                   separators=(",", ":")) + "\n")
        entries.append({**r, "trace_file":name,
                        "trace_sha256":hashlib.sha256(path.read_bytes()).hexdigest()})
        print(name, "min", r["min_tilt_deg"], "end", r["final_tilt_deg"],
              "hold", r["longest_upright_s"], flush=True)
    result = {
        "status":"MODEL_SENSITIVITY_ONLY_NOT_HARDWARE_VALIDATED",
        "engine":"MuJoCo 3.14.0",
        "original_cad_commit":verified["upstream_commit"],
        "source_step_shas": {key: value["sha256"]
                             for key, value in step["files"].items()},
        "assumptions": [
            "C++ source default pulse is 1.5s; shorter pulse trials are our experiments",
            "22.2V × 380KV ideal speed cap is assumed, not motor envelope",
            "STEP motor centers checked but collision geometry/contact stiffness approximated",
            "The LQR sign mapping was inferred and inverted, not original controller parity",
            "Source chassis inertias require an explicit physically feasible projection",
            "No physical robot conclusion, no validated success"
        ],
        "runs": entries,
    }
    (out/"summary.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--cache",type=Path,default=Path.home()/"mini-wheelbot-original-cad")
    p.add_argument("--step-cache",type=Path,default=Path.home()/"mini-wheelbot-step")
    p.add_argument("--out",type=Path,default=Path.home()/"mini-wheelbot-brake-sweep")
    a = p.parse_args()
    r = run(a.cache,a.step_cache,a.out)
    print("RUNS",len(r["runs"]),"HARDWARE_VALIDATED",False)


if __name__ == "__main__":
    main()
