"""Issue #50: controlled audit of CAD30 starting-stance variants.

Reuses PR #43 static IK/wrench planner and PR #44 real 2ms MuJoCo
physics replay. The latter is NOT a walking controller; a result is a
negative or partial diagnostic unless all cycle gates actually pass.

Private original CAD and raw physical traces stay outside GitHub.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import importlib.util
import json
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class Variant:
    name: str
    middle_inward_mm: float
    rear_forward_mm: float
    rear_out_mm: float
    final_prep_back_mm: float


VARIANTS = (
    Variant("pr43_baseline", 50, 80, 20, 30),
    Variant("no_final_back", 50, 80, 20, 0),
    Variant("middle_40", 40, 80, 20, 0),
    Variant("middle_30", 30, 80, 20, 0),
    Variant("outward_10", 50, 80, 10, 0),
)


def candidate_targets(feet: dict[str, Any], variant: Variant) -> dict[str, Any]:
    """Copy original 3D foot vectors; use the original PR43 sign convention."""
    goals = {name: vec.copy() for name, vec in feet.items()}
    for name in ("ml", "mr", "bl", "br"):
        goals[name][2] = 0.0
    goals["ml"][0] += variant.middle_inward_mm / 1000
    goals["mr"][0] -= variant.middle_inward_mm / 1000
    goals["bl"][0] -= variant.rear_out_mm / 1000
    goals["br"][0] += variant.rear_out_mm / 1000
    goals["bl"][1] += variant.rear_forward_mm / 1000
    goals["br"][1] += variant.rear_forward_mm / 1000
    return goals


def load_module(path: Path, name: str):
    if not path.is_file():
        raise FileNotFoundError(f"Expected stacked research code: {path}")
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def summarize_math(result: dict[str, Any], nodes: list[dict[str, Any]]) -> dict[str, Any]:
    planned_y = [float(node["root_xy_m"][1]) for node in nodes]
    return {
        "status": result["status"],
        "sampled_errors": len(result.get("errors", [])),
        "sampled_nodes": len(nodes),
        "min_sampled_support_margin_mm": result["min_sampled_support_margin_mm"],
        "max_sampled_joint_abs_rad": result["max_sampled_leg_joint_abs_rad"],
        "min_required_peak_static_tau_Nm": result["max_sampled_required_static_tau_Nm"],
        "nominal_net_root_Y_from_saved_six_mm": result["max_body_net_Y_from_six_prep_mm"],
        "minimum_planned_root_Y_relative_to_saved_six_mm": 1000 * (min(planned_y) - planned_y[0]),
        "is_physical_gait": False,
    }


def summarize_physics(result: dict[str, Any]) -> dict[str, Any]:
    fail = result.get("failure") or {}
    return {
        "status": result["status"],
        "physics_steps": result["physics_steps"],
        "nodes_attempted": result["nodes_attempted"],
        "nodes_total": result["nodes_total"],
        "first_failure_phase": fail.get("phase"),
        "first_failure_reason": fail.get("reason"),
        "first_failure_margin_mm": fail.get("COM_margin_mm"),
        "minimum_loaded_leg_count": result["min_loaded_leg_count"],
        "net_COM_Y_from_saved_six_mm": result["COM_Y_delta_from_saved_six_start_mm"],
        "net_root_Y_from_saved_six_mm": result["root_Y_delta_from_saved_six_start_mm"],
        "has_full_verified_cycle": False,
    }


def run_one(repo: Path, cad_root: Path, six: Path, front: Path,
            private_output: Path, variant: Variant, physics: bool,
            node_seconds: float) -> dict[str, Any]:
    mod = load_module(repo / "experiments/cad30/pupper_baseline/math_first_four_crawl_v1.py", "m0_math")
    old_target = mod.candidate_targets
    old_back = mod.BODY_PREP_BACK_M
    try:
        mod.candidate_targets = lambda feet: candidate_targets(feet, variant)
        mod.BODY_PREP_BACK_M = variant.final_prep_back_mm / 1000
        result, nodes = mod.run(cad_root, six, front)
    finally:
        mod.candidate_targets = old_target
        mod.BODY_PREP_BACK_M = old_back
    private_output.mkdir(parents=True, exist_ok=True)
    trace_path = private_output / (variant.name + "_math_trace.json")
    trace_path.write_text(json.dumps({"schema": "cad30-math-first-four-crawl-v1", "nodes": nodes}))
    summary = {"variant": asdict(variant), "math": summarize_math(result, nodes),
               "private_math_trace_sha256": hashlib.sha256(trace_path.read_bytes()).hexdigest()}
    if physics:
        dynamic = load_module(repo / "experiments/cad30/math_first_dynamic/dynamic_replay_math_first.py", "m0_physics")
        dst = private_output / (variant.name + "_dynamic")
        physical = dynamic.run(cad_root, trace_path, six, dst, mu=1.1,
                               node_s=node_seconds, support_policy="three")
        summary["physics"] = summarize_physics(physical)
        full = dst.with_name(dst.stem + "_trace_2ms.json.gz")
        if full.exists():
            summary["private_physical_trace_sha256"] = hashlib.sha256(full.read_bytes()).hexdigest()
    return summary


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--repo-root", type=Path, required=True)
    p.add_argument("--cad-root", type=Path, required=True)
    p.add_argument("--six-state", type=Path, required=True)
    p.add_argument("--front-reference", type=Path, required=True)
    p.add_argument("--private-output", type=Path, required=True)
    p.add_argument("--report", type=Path, required=True)
    p.add_argument("--variants", nargs="+", default=[v.name for v in VARIANTS])
    p.add_argument("--physics", action="store_true")
    p.add_argument("--node-seconds", type=float, default=0.3)
    args = p.parse_args()
    lookup = {v.name: v for v in VARIANTS}
    if args.node_seconds <= 0:
        p.error("--node-seconds must be positive")
    if any(name not in lookup for name in args.variants):
        p.error(f"unknown variant; choose from {', '.join(lookup)}")
    cases = [run_one(args.repo_root, args.cad_root, args.six_state,
                     args.front_reference, args.private_output, lookup[name],
                     args.physics, args.node_seconds)
             for name in args.variants]
    report = {"schema": "cad30-M0-initial-stance-audit-v1", "issue": 50,
              "model": "original private Freenove CAD30 (no CAD assets included)",
              "saved_six": "physical saved six-support state AFTER earlier Pupper prebalance",
              "physics_timestep_s": 0.002, "node_seconds": args.node_seconds,
              "limits_are_unverified_model_assumptions": True,
              "goal": "full continuous 4-foot cycle and positive net motion including preparation",
              "cases": cases, "full_gait_proven": False}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()