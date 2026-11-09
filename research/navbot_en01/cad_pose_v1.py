#!/usr/bin/env python3
"""Create a CAD-pose-anchored EN01 *surrogate* MJCF from a pinned STEP receipt.

Only the static locations of two servo horns and wheel-tire axes are source
measured. Linkage kinematics, COM/inertias, servo zeros and dynamics remain
assumptions. The source v0 model is never overwritten.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
EVIDENCE = HERE / "results/cad_nominal_pose_from_pinned_step_20261109.json"
V0 = HERE / "model_v0.xml"
V1 = HERE / "model_v1_cad_pose.xml"
PINNED_BLOB = "9156683a0bb694c84dc541a3e26b5f6b08db1a41"
# Unknown from the STEP mechanical placement; retained diagnostic assumption.
TORSO_COM_ABOVE_HIP_M = 0.015


def dimensions(evidence: dict) -> dict[str, float]:
    if (evidence.get("schema") != "navbot-en01-cad-nominal-pose-evidence/v1"
            or evidence.get("source", {}).get("git_blob_sha1") != PINNED_BLOB
            or evidence.get("source", {}).get("raw_bytes") != 12358579):
        raise ValueError("Unexpected original STEP evidence; refusing model")
    placements = evidence["entities"]
    if set(placements) != {"428", "429", "471", "472"}:
        raise ValueError("Expected exactly four source assembly occurrences")
    for n, label in (("428", "舵盘"), ("429", "舵盘_MIR1"),
                     ("471", "2208电机轮胎"), ("472", "2208电机轮胎")):
        if placements[n]["name"] != label or placements[n]["occurrence_step_id"] != int(n):
            raise ValueError("Source part identity mismatch: " + n)
        axis = placements[n]["local_y_axis_in_cad"]
        if len(axis) != 3 or abs(abs(axis[2]) - 1) > 1e-6:
            raise ValueError("Wheel/hip bore not aligned with CAD lateral axis")

    def vec(n: str) -> list[float]:
        xyz = [float(x) for x in placements[n]["origin_mm"]]
        if len(xyz) != 3 or not all(math.isfinite(x) for x in xyz):
            raise ValueError("Invalid CAD coordinate")
        return xyz

    hp, hm, wp, wm = vec("428"), vec("429"), vec("472"), vec("471")
    dp = [b - a for a, b in zip(hp, wp)]
    dm = [b - a for a, b in zip(hm, wm)]
    if (abs(hp[0] - hm[0]) > 0.001 or abs(hp[1] - hm[1]) > 0.001
            or abs(wp[0] - wm[0]) > 0.001 or abs(wp[1] - wm[1]) > 0.001
            or max(abs(dp[i] - dm[i]) for i in (0, 1)) > 0.001
            or abs(dp[2] + dm[2]) > 0.001):
        raise ValueError("CAD placement symmetry gate failed")
    radii = evidence["measurements"]["tire_cylinder_radius_candidates_mm"]
    if [float(x) for x in radii] != [18.3, 18.5]:
        raise ValueError("Unexpected source tire surfaces")
    r = 18.5 / 1000.0  # candidate, not validated rolling radius
    hip_half = abs(hp[2] - hm[2]) / 2000.0
    tire_half = abs(wp[2] - wm[2]) / 2000.0
    dx, dz = dp[0] / 1000.0, dp[1] / 1000.0  # CAD Y maps to MuJoCo Z
    outer = tire_half - hip_half
    if not (0.02 < -dz < 0.08 and 0.02 < hip_half < 0.06 and
            0 < outer < 0.02 and abs(dx) < 0.02):
        raise ValueError("Implausible CAD nominal pose")
    return {"wheel_radius_m": r, "hip_half_span_m": hip_half,
            "tire_half_span_m": tire_half, "wheel_dx_m": dx,
            "wheel_dz_m": dz, "wheel_outer_offset_m": outer,
            "torso_com_above_hip_m": TORSO_COM_ABOVE_HIP_M,
            "root_z_m": r - dz + TORSO_COM_ABOVE_HIP_M}


def build(source: Path = EVIDENCE, baseline: Path = V0, output: Path = V1) -> dict:
    evidence = json.loads(source.read_text(encoding="utf-8"))
    v = dimensions(evidence)
    tree = ET.parse(baseline)
    root = tree.getroot()
    root.set("model", "NavBot_EN01_CAD_POSE_v1_SURROGATE_NOT_LINKAGE")
    torso = root.find("./worldbody/body[@name='torso']")
    if torso is None:
        raise ValueError("Baseline torso missing")
    torso.set("pos", f"0 0 {v['root_z_m']:.9f}")
    for side, sign in (("left", 1), ("right", -1)):
        leg = torso.find(f"./body[@name='{side}_leg']")
        if leg is None:
            raise ValueError("Baseline leg missing: " + side)
        leg.set("pos", f"0 {sign * v['hip_half_span_m']:.9f} {-TORSO_COM_ABOVE_HIP_M:.9f}")
        geom = leg.find(f"./geom[@name='{side}_leg_geom']")
        wheel = leg.find(f"./body[@name='{side}_wheel']")
        wheel_geom = wheel.find(f"./geom[@name='{side}_wheel_geom']")
        if geom is None or wheel is None or wheel_geom is None:
            raise ValueError("Baseline geometry incomplete: " + side)
        offset = sign * v["wheel_outer_offset_m"]
        end = f"{v['wheel_dx_m']:.9f} {offset:.9f} {v['wheel_dz_m']:.9f}"
        geom.set("fromto", "0 0 0 " + end)
        wheel.set("pos", end)
        size = wheel_geom.get("size", "").split()
        if len(size) != 2:
            raise ValueError("Expected cylinder radius and half-width")
        wheel_geom.set("size", f"{v['wheel_radius_m']:.9f} {size[1]}")
    root.insert(0, ET.Comment("SOURCE: pinned STEP nominal placement only; four-bar linkage, masses, torque-speed and servo zero NOT CAD VALIDATED"))
    ET.indent(tree, space="  ")
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="unicode", xml_declaration=False)
    with output.open("a", encoding="utf-8") as f:
        f.write("\n")
    return {"schema": "navbot-en01-cad-pose-surrogate/v1", "dimensions": v,
            "model_sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
            "evidence_git_blob_sha1": PINNED_BLOB,
            "validation": "static source pose only; four-bar and dynamics NOT validated"}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--source", type=Path, default=EVIDENCE)
    ap.add_argument("--baseline", type=Path, default=V0)
    ap.add_argument("--out", type=Path, default=V1)
    args = ap.parse_args()
    print(json.dumps(build(args.source, args.baseline, args.out), indent=2))


if __name__ == "__main__":
    main()
