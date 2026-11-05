#!/usr/bin/env python3
"""Offline, dependency-free audit of recorded MuJoCo per-leg Fz contact traces.

Reads the *observed* 2-ms JSON/GZ trace created by replay_six_ml_mr.py.
No new physics is run and no state is interpolated or relabeled as passing.
The strict instantaneous support criterion intentionally remains unchanged.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import math
import statistics
from pathlib import Path
from typing import Any

WALK = ("ml", "mr", "bl", "br")
EXPECTED_SUPPORT = ("ml", "bl", "br")  # MR is the swinging leg
PHASE_SWING = "swing_mr"
PHASE_HOLD = "final_hold"
FZ_THRESHOLD_N = 0.2


def load_trace(path: Path) -> list[dict[str, Any]]:
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt", encoding="utf-8") as handle:
        data = json.load(handle)
    frames = data["frames"]
    if not isinstance(frames, list) or len(frames) < 2:
        raise ValueError("Expected at least two recorded physics frames")
    prev = -float("inf")
    for i, f in enumerate(frames):
        t = float(f["time_s"])
        if not math.isfinite(t) or t <= prev:
            raise ValueError(f"Non-monotonic/nonfinite time at frame {i}")
        prev = t
        for leg in WALK:
            z = float(f["Fz_N"][leg])
            if not math.isfinite(z):
                raise ValueError(f"Invalid Fz for {leg} at frame {i}")
    return frames


def index_runs(indices: list[int]) -> list[list[int]]:
    """Group *consecutive recorded frame indices*, not a rolling average."""
    out: list[list[int]] = []
    for index in indices:
        if not out or index != out[-1][-1] + 1:
            out.append([index])
        else:
            out[-1].append(index)
    return out


def audit(frames: list[dict[str, Any]], source_sha256: str | None = None) -> dict[str, Any]:
    step_durations = [float(b["time_s"]) - float(a["time_s"]) for a, b in zip(frames, frames[1:])]
    dt = statistics.median(step_durations)
    if max(abs(s - dt) for s in step_durations) > max(1e-6, dt * 0.05):
        raise ValueError("Non-uniform time samples: cannot report per-frame contact duration")

    swings = [(i, f) for i, f in enumerate(frames) if f["phase"] == PHASE_SWING]
    holds = [(i, f) for i, f in enumerate(frames) if f["phase"] == PHASE_HOLD]
    if not swings or not holds:
        raise ValueError("Missing MR swing or final hold in trace")
    swing_missing = [(i, f) for i, f in swings
                     if any(float(f["Fz_N"][leg]) < FZ_THRESHOLD_N for leg in EXPECTED_SUPPORT)]
    hold_missing = [(i, f) for i, f in holds
                    if any(float(f["Fz_N"][leg]) < FZ_THRESHOLD_N for leg in WALK)]
    swing_bad = [i for i, _ in swing_missing]
    hold_bad = [i for i, _ in hold_missing]
    runs = index_runs(swing_bad)
    swing_events = []
    for i, f in swing_missing:
        absent = [leg for leg in EXPECTED_SUPPORT if float(f["Fz_N"][leg]) < FZ_THRESHOLD_N]
        prev_fz = {leg: float(frames[i - 1]["Fz_N"][leg]) for leg in absent} if i > 0 else None
        next_fz = {leg: float(frames[i + 1]["Fz_N"][leg]) for leg in absent} if i + 1 < len(frames) else None
        swing_events.append({
            "trace_index": i,
            "time_s": round(float(f["time_s"]), 6),
            "missing_support": absent,
            "observed_fz_N": {leg: round(float(f["Fz_N"][leg]), 6) for leg in EXPECTED_SUPPORT},
            "prev_fz_for_missing_N": prev_fz,
            "next_fz_for_missing_N": next_fz,
        })
    gaps = [float(b["time_s"]) - float(a["time_s"])
            for (_, a), (_, b) in zip(swing_missing, swing_missing[1:])]
    hold_last_index = holds[-1][0]
    # Last recorded violating frame is not part of the stable tail.
    hold_tail_ticks = (hold_last_index - hold_bad[-1]) if hold_bad else len(holds)
    hold_last_bad = frames[hold_bad[-1]]["time_s"] if hold_bad else None
    return {
        "schema": "cad30-contact-chatter-audit-v1",
        "source_sha256": source_sha256,
        "analysis": "post-hoc only; NO simulation and NO control-loop test",
        "dt_s": round(dt, 9),
        "recorded_frames": len(frames),
        "swing": {
            "phase": PHASE_SWING,
            "required_support": list(EXPECTED_SUPPORT),
            "threshold_N": FZ_THRESHOLD_N,
            "frames": len(swings),
            "failed_frames": len(swing_missing),
            "failure_duration_s": round(len(swing_missing) * dt, 6),
            "max_consecutive_failed_frames": max((len(run) for run in runs), default=0),
            "failure_events": swing_events,
            "gap_between_failures_s": [round(x, 6) for x in gaps],
            "mean_gap_s": round(statistics.mean(gaps), 6) if gaps else None,
            "std_gap_s": round(statistics.pstdev(gaps), 6) if gaps else None,
            "missing_by_leg": {leg: sum(float(f["Fz_N"][leg]) < FZ_THRESHOLD_N
                                       for _, f in swings) for leg in EXPECTED_SUPPORT},
            "strict_pass": len(swing_missing) == 0,
        },
        "final_hold": {
            "phase": PHASE_HOLD,
            "required_support": list(WALK),
            "frames": len(holds),
            "failed_frames": len(hold_missing),
            "fraction_four_loaded": round(1 - len(hold_missing) / len(holds), 9),
            "last_failed_time_s": round(float(hold_last_bad), 6) if hold_last_bad is not None else None,
            "continuous_good_tail_s": round(hold_tail_ticks * dt, 6),
            "strict_pass": len(hold_missing) == 0,
        },
        "overall_strict_contact_pass": len(swing_missing) == 0 and len(hold_missing) == 0,
        "interpretation_guardrails": [
            "Fz=0 is an observed MuJoCo force sample; the trace has no per-contact geom pair list or mesh Z at this frame.",
            "Quasi-periodic isolated force losses are not proven to be numerical artifacts.",
            "Do not replace an instantaneous >=0.2N gate with smoothing/averaging without an explicit new protocol.",
            "This trace alone cannot test dt, solver, solref, friction or true robot hardware.",
        ],
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("trace", type=Path, help="Raw 2-ms JSON or JSON.GZ from PR #47")
    ap.add_argument("--out", type=Path, help="Optional compact JSON report")
    args = ap.parse_args()
    result = audit(load_trace(args.trace), hashlib.sha256(args.trace.read_bytes()).hexdigest())
    encoded = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(encoded, encoding="utf-8")
    else:
        print(encoded, end="")


if __name__ == "__main__":
    main()