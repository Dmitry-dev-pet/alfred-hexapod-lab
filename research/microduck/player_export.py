#!/usr/bin/env python3
"""Export authentic original-STL MicroDuck replay poses from saved MuJoCo states.

No physics regenerated. Every frame comes from an existing verified 50 Hz
qpos trace. MuJoCo forward kinematics computes the visual mesh world poses.
No third-party mesh bytes are copied into the destination Git repository.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct

import mujoco
import numpy as np
from scipy.spatial import cKDTree

SOURCE_COMMIT = "bae39f5900b318f7e87fc485c30a31213cfa14eb"
POLICY_SHA256 = "2fc7f8ee6acd7d0d3f66a99927b5d5a56589aeed02ec714de329b252d5ccf492"
FORMAT = "microduck-original-stl-mujoco-replay-v1"
ROUTE = "microduck-standup-v1"
CASES = ("stand", "sit", "fold", "roll180", "pitch180", "side90", "push")


def check(condition, message):
    if not condition:
        raise ValueError(message)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def numerics(row):
    check(all(math.isfinite(float(v)) for v in row), "Non-finite MuJoCo transform")
    return [round(float(v), 7) for v in row]


def quat_xyzw_source(q):
    """Input MuJoCo wxyz; output MuJoCo wxyz numerics."""
    return numerics(q)


def matrix_to_quat(mat):
    out = np.zeros(4)
    mujoco.mju_mat2Quat(out, np.asarray(mat, dtype=np.float64).reshape(9))
    return quat_xyzw_source(out)


def binary_stl_vertices(path):
    payload = path.read_bytes()
    check(len(payload) >= 84, f"Bad STL: {path}")
    count = struct.unpack_from("<I", payload, 80)[0]
    check(count > 0 and len(payload) == 84 + 50 * count,
          f"Unsupported ASCII/invalid binary STL: {path}")
    records = np.frombuffer(payload, dtype=np.dtype([
        ("normal", "<f4", (3,)),
        ("v", "<f4", (3, 3)),
        ("attr", "<u2"),
    ]), offset=84, count=count)
    return np.unique(records["v"].reshape(-1, 3).astype(np.float64), axis=0)


def verified_mesh_transform(model, mid, assets, name):
    """Find and VERIFY exact raw STL->MuJoCo normalized mesh transform.

    MuJoCo preprocesses imported meshes. We independently align original STL
    vertices with compiled mesh_vert (nearest-point error) and fail closed
    if a clean rigid/similarity mapping cannot be recovered.
    """
    filename = name + ".stl"
    path = assets / filename
    check(path.is_file(), f"Missing original upstream STL: {filename}")
    raw = binary_stl_vertices(path)
    start, total = int(model.mesh_vertadr[mid]), int(model.mesh_vertnum[mid])
    target = np.asarray(model.mesh_vert[start:start + total], dtype=np.float64)
    check(len(target) > 0, "Empty compiled mesh")
    scale = np.asarray(model.mesh_scale[mid], dtype=np.float64)
    pos = np.asarray(model.mesh_pos[mid], dtype=np.float64)
    quat = np.asarray(model.mesh_quat[mid], dtype=np.float64)
    m = np.zeros(9)
    mujoco.mju_quat2Mat(m, quat)
    rotation = m.reshape(3, 3)
    tree = cKDTree(target)
    # MuJoCo's mesh_pos/mesh_quat transform imported source vertices; test
    # the common interpretations, including inverted offset and quaternion.
    candidates = []
    for label, R in (("q", rotation), ("q-inverse", rotation.T)):
        for suffix, tr in (
            ("plus", pos),
            ("minus", -pos),
            ("centered", -(R @ pos)),
            ("inverse-centered", R @ pos),
            ("zero", np.zeros(3)),
        ):
            candidates.append((label + "-" + suffix, R, tr))
    sample = raw[::max(1, len(raw) // 2000)]
    scored = []
    for label, R, tr in candidates:
        points = (sample * scale) @ R.T + tr
        d, _ = tree.query(points)
        scored.append((float(np.quantile(d, .99)), float(np.median(d)), label, R, tr))
    scored.sort(key=lambda t: (t[0], t[1]))
    p99, median, label, R, tr = scored[0]
    # The match must be essentially exact up to STL float32 precision.
    check(p99 <= 5e-5, f"Cannot verify original STL orientation {name}: p99={p99:.7f}, best={label}")
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, R.reshape(9))
    b = path.read_bytes()
    return {
        "file": filename,
        "sha256": digest(b),
        "size": len(b),
        "scale": numerics(scale),
        "rotation_wxyz": numerics(q),
        "translation_xyz": numerics(tr),
        "fit_p99_m": round(p99, 8),
        "fit_median_m": round(median, 8),
        "fit_method": label,
    }


def build(args):
    root = args.upstream.resolve()
    scene = root / "src/mjlab_microduck/robot/microduck/scene.xml"
    assets = root / "src/mjlab_microduck/robot/microduck/assets"
    check(scene.is_file() and assets.is_dir(), "Pinned upstream checkout missing robot MJCF/STL")
    model = mujoco.MjModel.from_xml_path(str(scene))
    raw = args.trace.read_bytes()
    trace = json.loads(raw)
    check(trace["schema"] == "microduck-cpu-benchmark-v1", "Wrong trace format")
    check(trace["model_commit"] == SOURCE_COMMIT, "Wrong pinned robot commit")
    check(trace["policy_sha256"] == POLICY_SHA256, "Wrong ONNX policy")
    check(trace["scene_sha256"] == digest(scene.read_bytes()), "MJCF scene changed")
    check(len(trace["cases"]) == len(CASES), "Missing physical cases")
    check(model.nq == len(trace["cases"][0]["frames"][0]["qpos"]), "Model/trace qpos mismatch")
    check(model.nu == 14, "Unexpected joint actuation count")

    visible = []
    unique_meshes = {}
    for gid in range(model.ngeom):
        if int(model.geom_type[gid]) != int(mujoco.mjtGeom.mjGEOM_MESH):
            continue
        if int(model.geom_group[gid]) != 2:  # Official "visual" class, never collision geoms.
            continue
        mid = int(model.geom_dataid[gid])
        check(mid >= 0, "Unresolved visual mesh")
        mesh_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mid)
        check(mesh_name and mesh_name.replace("_", "").isalnum(), "Unsafe mesh reference")
        if mesh_name not in unique_meshes:
            unique_meshes[mesh_name] = verified_mesh_transform(model, mid, assets, mesh_name)
        matid = int(model.geom_matid[gid])
        rgba = (model.mat_rgba[matid] if matid >= 0 else model.geom_rgba[gid])
        visible.append({"gid": gid, "mesh": mesh_name,
                        "rgba": numerics(rgba)})
    check(15 <= len(visible) <= 180, f"Implausible geometry part count: {len(visible)}")
    check(8 <= len(unique_meshes) <= 80, "Implausible mesh asset count")

    manifest = {
        "schema": "microduck-original-stl-assets-v1",
        "repo": "pollen-robotics/microduck_rl",
        "revision": SOURCE_COMMIT,
        "root": "src/mjlab_microduck/robot/microduck/assets/",
        "model_scene_sha256": digest(scene.read_bytes()),
        "source_trace_sha256": digest(raw),
        "licence": "CC BY-NC-SA 4.0; upstream attribution required; personal research only",
        "meshes": unique_meshes,
        "visible_geoms": visible,
    }
    target = args.output.resolve()
    target.mkdir(parents=True, exist_ok=True)
    manifestpath = target / "asset-manifest.json"
    manifestpath.write_text(json.dumps(manifest, separators=(",", ":")) + "\n")

    data = mujoco.MjData(model)
    for case in trace["cases"]:
        name = case["case"]
        check(name in CASES, f"Unexpected case {name}")
        out = []
        prior = -1.0
        for rec in case["frames"]:
            t = float(rec["t"])
            check(t > prior, "Time must strictly increase")
            prior = t
            check(len(rec["qpos"]) == model.nq and len(rec["qvel"]) == model.nv,
                  "Truncated physical state")
            data.qpos[:] = rec["qpos"]
            data.qvel[:] = rec["qvel"]
            data.ctrl[:] = rec["ctrl"]
            mujoco.mj_forward(model, data)
            row = [round(t, 5)] + numerics(data.qpos[:7])
            for g in visible:
                gid = g["gid"]
                row.extend(numerics(data.geom_xpos[gid]))
                row.extend(matrix_to_quat(data.geom_xmat[gid]))
            check(all(math.isfinite(x) for x in row), "Nonfinite FK sample")
            out.append(row)
        check(len(out) == 300 and abs(prior - 6.0) < .01, "Unexpected sample length")
        result = {
            "format": FORMAT,
            "source": {
                "model_commit": SOURCE_COMMIT,
                "trace_sha256": digest(raw),
                "policy_sha256": POLICY_SHA256,
                "scene_sha256": digest(scene.read_bytes()),
                "physics": trace["physics"],
            },
            "case": name,
            "status": case["status"],
            "duration_s": prior,
            "frame_count": len(out),
            "gate_stable_at_end": case["stable_at_end_1s"],
            "first_one_second_stable_s": case["time_to_first_1s_stable_s"],
            "warnings": case["warnings"],
            "layout": {
                "root_pose_start": 1,
                "visible_mesh_pose_start": 8,
                "visible_mesh_stride": 7,
                "visible_mesh_count": len(visible),
                "frame_stride": len(out[0])
            },
            "frames": out,
        }
        check(all(len(row) == result["layout"]["frame_stride"] for row in out),
              "Variable-length playback frame")
        pathname = target / (name + ".json")
        pathname.write_text(json.dumps(result, separators=(",", ":")) + "\n")
        check(pathname.stat().st_size <= 8_000_000, f"Replay too big for guarded catalog: {name}")
        print(f"{name}: {len(out)} genuine pose frames, {len(visible)} visual geoms, {pathname.stat().st_size} bytes")
    print(f"original STL references: {len(unique_meshes)}, visible instanced geoms: {len(visible)}")
    print(f"manifest: {manifestpath.stat().st_size} bytes")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--upstream", required=True, type=Path)
    ap.add_argument("--trace", required=True, type=Path)
    ap.add_argument("--output", required=True, type=Path)
    build(ap.parse_args())


if __name__ == "__main__":
    main()
