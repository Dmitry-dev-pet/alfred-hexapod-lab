#!/usr/bin/env python3
"""Read-only static audit of upstream LuwuDynamics/luwu_mjlab XGO Lite3.

This tool does NOT simulate dynamics, load untrusted Python, or actuate hardware.
Usage: python audit_lite3_model.py --xml /path/to/lite3.xml --env /path/to/env_cfgs.py
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
from pathlib import Path
from xml.etree import ElementTree as ET

UPSTREAM_REPO = "https://github.com/LuwuDynamics/luwu_mjlab"
UPSTREAM_COMMIT = "447945fd40fb5585271dce4ea8cc82567916032a"
UPSTREAM_MODEL_BLOB = "9c9699c5fc3c5f4ed4c69ebf516c346ee6a57026"
HARDWARE_MASS_KG = 0.610  # Luwu product spec, not an experimentally weighed unit.


def git_blob_hash(data: bytes) -> str:
    header = f"blob {len(data)}\0".encode("ascii")
    return hashlib.sha1(header + data).hexdigest()


def audit_model(xml_bytes: bytes) -> dict:
    root = ET.fromstring(xml_bytes)
    if root.tag != "mujoco":
        raise ValueError("Expected MuJoCo MJCF root")
    bodies = root.findall(".//worldbody//body")
    masses = [float(e.attrib["mass"]) for b in bodies for e in b.findall("inertial")]
    joints = [e.attrib["name"] for b in bodies for e in b.findall("joint")]
    position_actuators = [e.attrib["joint"] for e in root.findall("./actuator/position")]
    legs = ("FL", "FR", "RL", "RR")
    leg_joint_names = [f"{leg}_{axis}_joint" for leg in legs for axis in ("hip", "thigh", "shank")]
    arm_joint_names = [n for n in joints if n not in leg_joint_names]
    foot_geoms = [e.attrib["name"] for e in root.findall(".//geom")
                  if e.attrib.get("name", "").endswith("_shank_collision")]
    servo = next((e for e in root.findall(".//default/position")
                  if "forcerange" in e.attrib), None)
    return {
        "model_name": root.attrib.get("model"),
        "git_blob_sha1": git_blob_hash(xml_bytes),
        "body_count": len(bodies),
        "joint_count": len(joints),
        "actuator_count": len(position_actuators),
        "leg_joint_count": sum(n in joints for n in leg_joint_names),
        "arm_joints": arm_joint_names,
        "model_mass_kg": round(sum(masses), 6),
        "hardware_nominal_mass_kg": HARDWARE_MASS_KG,
        "mass_ratio_hw_over_model": round(HARDWARE_MASS_KG / sum(masses), 5) if masses else None,
        "shank_collision_geoms": sorted(foot_geoms),
        "position_default_force_range": servo.attrib.get("forcerange") if servo is not None else None,
        "verified_upstream_xml_blob": git_blob_hash(xml_bytes) == UPSTREAM_MODEL_BLOB,
        "notes": [
            "Static MJCF structure only; not an mjspec compilation or dynamics simulation.",
            "Model mass sum and actuator count are not a calibrated hardware parity check.",
            "Factory arm includes a claw DOF not present as a joint in this model.",
        ],
    }


def audit_env(source: str) -> dict:
    def num(pattern: str) -> float | None:
        m = re.search(pattern, source)
        return float(m.group(1)) if m else None
    dt = num(r"cfg\.sim\.mujoco\.timestep\s*=\s*([\d.]+)")
    decimation = num(r"cfg\.decimation\s*=\s*([\d.]+)")
    return {
        "mujoco_timestep_seconds": dt,
        "policy_decimation": int(decimation) if decimation is not None else None,
        "nominal_policy_hz": round(1 / (dt * decimation), 6) if dt and decimation else None,
        "action_scale": num(r"joint_pos_action\.scale\s*=\s*([\d.]+)"),
        "foot_phase_period_s": num(r'cfg\.observations\["actor"\].terms\["phase"\].params\["period"\]\s*=\s*([\d.]+)'),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--xml", type=Path, required=True)
    ap.add_argument("--env", type=Path, help="Upstream lite3_walk/env_cfgs.py")
    ap.add_argument("--strict-upstream-sha", action="store_true", help="Reject an XML not matching the pinned original blob")
    ap.add_argument("--output", type=Path, help="Write JSON report here")
    args = ap.parse_args()
    result = {"source_repo": UPSTREAM_REPO, "source_commit": UPSTREAM_COMMIT,
              "model": audit_model(args.xml.read_bytes())}
    if args.strict_upstream_sha and not result["model"]["verified_upstream_xml_blob"]:
        raise SystemExit("FAIL: XML is not byte-for-byte the audited upstream revision")
    if args.env:
        result["environment"] = audit_env(args.env.read_text(encoding="utf-8"))
    report = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(report, encoding="utf-8")
    print(report, end="")


if __name__ == "__main__":
    main()
