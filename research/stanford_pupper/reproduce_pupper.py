#!/usr/bin/env python3
"""Pupper v3 original-model CPU physics preflight, with NO RL training.

This is not a replay of a neural policy. It loads the exact pinned Stanford MJCF,
removes only non-colliding render meshes, adds a ground plane and identical
position-servo parameters from Stanford's PUPPER_ACTUATOR, and integrates
MuJoCo mj_step while commanding the original neutral pose.

The model provenance is checked against Git's SHA-1 blob format. The built-in
joint/geometry/mass contract is checked before simulation. Do not use this as a
claim that a shipped policy was evaluated or sim-to-real succeeded.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

SOURCE_REPO = "cs123-stanford/pupper-mjlab"
SOURCE_REVISION = "aa475e9a393a209b29fd5de7e0ee10f3de643230"
SOURCE_PATH = "src/mjlab/asset_zoo/robots/pupper_v3/xmls/pupper_v3.xml"
SOURCE_BLOB_SHA1 = "7771e176c42b3aed1d48cc62434771b0fceb7c8c"
MODEL_MASS_KG = 3.170
PUPPER_JOINTS = tuple(f"leg_{position}_{side}_{n}" for position, side in
                      (("front", "r"), ("front", "l"), ("back", "r"), ("back", "l"))
                      for n in (1, 2, 3))
DEFAULT_POSE = (0.26, 0.0, -0.52, -0.26, 0.0, 0.52,
                0.26, 0.0, -0.52, -0.26, 0.0, 0.52)
FOOT_GEOMS = tuple(f"leg_{pos}_{side}_3_collision" for pos, side in
                   (("front", "r"), ("front", "l"), ("back", "r"), ("back", "l")))
FOOT_SITES = tuple(f"leg_{pos}_{side}_3_foot_site" for pos, side in
                   (("front", "r"), ("front", "l"), ("back", "r"), ("back", "l")))
TIMESTEP = 0.004
DECIMATION = 5
KP = 5.5
KD = 0.25
TORQUE_LIMIT = 3.0


def git_blob_sha(raw: bytes) -> str:
    return hashlib.sha1(b"blob " + str(len(raw)).encode() + b"\0" + raw).hexdigest()


def read_pinned_xml(path: Path) -> bytes:
    raw = path.read_bytes()
    digest = git_blob_sha(raw)
    if digest != SOURCE_BLOB_SHA1:
        raise ValueError(f"Source MJCF blob SHA mismatch: {digest}; expected {SOURCE_BLOB_SHA1}")
    return raw


def build_cpu_mjcf(raw: bytes) -> str:
    """Remove non-colliding visual meshes. Inertials and collisions stay intact."""
    root = ET.fromstring(raw)
    assert root.tag == "mujoco" and root.get("model") == "pupper_v3"
    removed_visuals = 0
    for body in root.findall(".//body"):
        for geom in list(body.findall("geom")):
            if geom.get("type") == "mesh":
                # Source default mesh geoms have no contact or inertia.
                assert geom.get("density") == "0" and geom.get("class") is None
                body.remove(geom)
                removed_visuals += 1
    assert removed_visuals == 13, f"Unexpected number of visual mesh geoms: {removed_visuals}"
    asset = root.find("asset")
    assert asset is not None and len(asset.findall("mesh")) == 13
    root.remove(asset)

    compiler = root.find("compiler")
    assert compiler is not None
    compiler.attrib.pop("meshdir", None)

    opts = ET.Element("option", {
        "timestep": str(TIMESTEP), "integrator": "Euler",
        "cone": "pyramidal", "impratio": "10",
        "iterations": "2", "ls_iterations": "5",
    })
    ET.SubElement(opts, "flag", {"eulerdamp": "disable"})
    root.insert(1, opts)

    world = root.find("worldbody")
    assert world is not None
    ET.SubElement(world, "geom", {
        "name": "ground_cpu_preflight", "type": "plane", "size": "5 5 0.1",
        "friction": "0.8 0.02 0.01", "contype": "1", "conaffinity": "1",
        "condim": "3", "group": "0", "rgba": "0.1 0.15 0.19 1",
    })

    # These are Stanford's own PUPPER_ACTUATOR values in pupper_constants.py.
    # Do not claim this is a full mjlab env; e.g. sensor/actuator delays,
    # randomized observations and the PPO policy are NOT installed here.
    actuator = ET.SubElement(root, "actuator")
    for joint in PUPPER_JOINTS:
        ET.SubElement(actuator, "position", {
            "name": f"servo_{joint}", "joint": joint,
            "kp": str(KP), "kv": str(KD),
            "forcelimited": "true", "forcerange": f"{-TORQUE_LIMIT} {TORQUE_LIMIT}",
        })
    return ET.tostring(root, encoding="unicode")


def load_model(raw: bytes) -> mujoco.MjModel:
    model = mujoco.MjModel.from_xml_string(build_cpu_mjcf(raw))
    if model.nq != 19 or model.nv != 18 or model.nu != 12:
        raise AssertionError(f"Unexpected model nq/nv/nu: {model.nq}/{model.nv}/{model.nu}")
    if not math.isclose(sum(model.body_mass), MODEL_MASS_KG, abs_tol=0.00001):
        raise AssertionError(f"Unexpected model mass {sum(model.body_mass)} kg")
    for j in PUPPER_JOINTS:
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j) < 0:
            raise AssertionError(f"Missing original joint {j}")
    for f in FOOT_GEOMS:
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f) < 0:
            raise AssertionError(f"Missing original collision {f}")
    for s in FOOT_SITES:
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, s) < 0:
            raise AssertionError(f"Missing original foot site {s}")
    return model


def contact_snapshot(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, float]:
    loads = {g: 0.0 for g in FOOT_GEOMS}
    plane_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground_cpu_preflight")
    for i in range(data.ncon):
        c = data.contact[i]
        if plane_id not in (c.geom1, c.geom2):
            continue
        other_id = c.geom2 if c.geom1 == plane_id else c.geom1
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, other_id)
        if name in loads:
            cf = np.zeros(6, dtype=np.float64)
            mujoco.mj_contactForce(model, data, i, cf)
            loads[name] += max(0.0, cf[0])
    return loads


def run_free_stance(model: mujoco.MjModel, *, duration_s: float) -> dict:
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    data.qpos[0:3] = [0.0, 0.0, 0.18]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    data.qpos[7:19] = DEFAULT_POSE
    data.ctrl[:] = DEFAULT_POSE
    mujoco.mj_forward(model, data)

    count = round(duration_s / TIMESTEP)
    stride = DECIMATION
    rows = []
    peak_force = 0.0
    saturations = 0
    contact_samples = 0
    for n in range(count):
        mujoco.mj_step(model, data)
        peak_force = max(peak_force, max(abs(float(x)) for x in data.actuator_force))
        saturations += int(any(abs(float(x)) >= TORQUE_LIMIT - 1e-6 for x in data.actuator_force))
        if (n + 1) % stride == 0 or n == count - 1:
            loads = contact_snapshot(model, data)
            feet_on_floor = int(sum(x > 0.2 for x in loads.values()))
            contact_samples += int(feet_on_floor > 0)
            rows.append({
                "t": round(data.time, 6),
                "base_xyz": [round(float(x), 8) for x in data.qpos[:3]],
                "base_quat_wxyz": [round(float(x), 8) for x in data.qpos[3:7]],
                "joint_q": [round(float(x), 8) for x in data.qpos[7:19]],
                "foot_normal_N": {k: round(v, 4) for k, v in loads.items()},
                "feet_loaded": feet_on_floor,
                "actuator_force_Nm": [round(float(x), 6) for x in data.actuator_force],
            })
    if not rows:
        raise AssertionError("No recorded frames")
    return {
        "mj_step_count": count,
        "stored_frames": len(rows),
        "initial_base_height_m": 0.18,
        "final_base_height_m": rows[-1]["base_xyz"][2],
        "final_loaded_feet": rows[-1]["feet_loaded"],
        "any_contact_sample_count": contact_samples,
        "peak_abs_actuator_force_Nm": round(peak_force, 6),
        "saturated_step_fraction": round(saturations / count, 6),
        "frames": rows,
    }


def cli() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-xml", type=Path, required=True,
                        help="Exact original Stanford Pupper v3 MJCF file (Git blob SHA checked).")
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument("--output", type=Path, default=Path("pupper_cpu_stance.json"))
    args = parser.parse_args()
    if not (0 < args.seconds <= 120):
        parser.error("--seconds must be in (0,120]")
    raw = read_pinned_xml(args.source_xml)
    model = load_model(raw)
    result = run_free_stance(model, duration_s=args.seconds)
    report = {
        "schema": "alfred-pupper-physics-only-v1",
        "policy_inference": "NOT_RUN",
        "rl_training": "NOT_RUN",
        "source": {"repo": SOURCE_REPO, "commit": SOURCE_REVISION,
                   "path": SOURCE_PATH, "git_blob_sha1": SOURCE_BLOB_SHA1},
        "approximations": ["render-only meshes omitted; original inertials and collision spheres kept",
                           "flat floor added", "position servos from upstream constants (no latency)"],
        "physics": {"engine": "mujoco", "version": mujoco.__version__, "timestep_s": TIMESTEP,
                    "policy_interval_s": TIMESTEP * DECIMATION, "mass_kg": MODEL_MASS_KG},
        "experiment": result,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"status": "CPU_STANCE_DONE", "output": str(args.output),
                      "stored_frames": result["stored_frames"],
                      "final_height_m": result["final_base_height_m"],
                      "contact": result["final_loaded_feet"],
                      "policy_inference": "NOT_RUN"}, ensure_ascii=False))


if __name__ == "__main__":
    cli()
