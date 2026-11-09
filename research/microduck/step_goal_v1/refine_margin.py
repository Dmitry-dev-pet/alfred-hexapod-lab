"""Bounded follow-up: address rejected 2 cm joint-limit overshoot.

The original gate stays fixed. Only motor target margin is enlarged.
"""
from pathlib import Path
import argparse
import hashlib
import json
import tune_recovery as sim

HERE = Path(__file__).resolve().parent


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workspace", type=Path, required=True)
    a = ap.parse_args()
    sim.initialize(a.workspace)
    path = HERE / "traces/summary.json"
    summary = json.loads(path.read_text())
    assert "margin_refinement" not in summary
    (HERE / "traces/initial-search-summary.json").write_text(path.read_text())
    base = summary["selected"]["params"]
    extra = []
    for margin in (.105, .14, .21):
        params = {**base, "height_m": .02, "motor_target_margin_rad": margin}
        extra.append(sim.save(params, f"margin-2cm-{round(margin*1000)}", HERE / "traces"))
    accepted = [e for e in extra if e["status"] == "PASS"]
    best = min(accepted, key=lambda e: e["params"]["motor_target_margin_rad"]) if accepted else None
    if best:
        for height in (.01, .03, .05):
            extra.append(sim.save({**best["params"], "height_m": height}, f"margin-height-{round(height*100)}cm", HERE / "traces"))
        for seed in (20261109, 20261110, 20261111):
            extra.append(sim.save({**best["params"], "seed": seed}, f"margin-perturb-{seed}", HERE / "traces"))
        trace, _ = sim.run(best["params"])
        raw = (json.dumps(trace, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode()
        assert hashlib.sha256(raw).hexdigest() == best["trace_sha256"]
    summary["episodes"].extend(extra)
    summary["margin_refinement"] = {"reason": "2 cm reaches the goal but exceeds the unchanged 1 degree overshoot gate",
                                    "tested_margins_rad": [.105, .14, .21], "selected_2cm": best,
                                    "selected_byte_identical_repeat": bool(best)}
    path.write_text(json.dumps(summary, indent=2) + "\n")


if __name__ == "__main__":
    main()
