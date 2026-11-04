"""Read the Freenove CAD kinematic chains and derive an objective tip landmark.

The tip is the tibia-mesh vertex farthest from its knee hinge center,
computed in the zero-angle pose. This is a geometric *candidate*, not a
verified physical rubber foot contact point.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from hexlab.cad_mujoco_v04 import DEFAULT_CACHE, build

LEG_CODES = ("ml", "bl", "br", "mr", "fr", "fl")


def _joint_on_body(model, mujoco, body_name: str) -> int:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0 or int(model.body_jntnum[body_id]) != 1:
        raise ValueError(f"Expected one hinge on {body_name}")
    jid = int(model.body_jntadr[body_id])
    if int(model.jnt_type[jid]) != int(mujoco.mjtJoint.mjJNT_HINGE):
        raise ValueError(f"Not a hinge: {body_name}")
    return jid


def geom_mesh_world_vertices(model, data, gid: int) -> np.ndarray:
    meshid = int(model.geom_dataid[gid])
    if meshid < 0:
        raise ValueError("Expected tibia mesh")
    start, n = int(model.mesh_vertadr[meshid]), int(model.mesh_vertnum[meshid])
    geom_rot = np.asarray(data.geom_xmat[gid]).reshape(3, 3)
    return np.asarray(model.mesh_vert[start:start+n]) @ geom_rot.T + np.asarray(data.geom_xpos[gid])


def load_legs(model, data, mujoco) -> list[dict]:
    mujoco.mj_forward(model, data)
    root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link_2")
    root_pos = np.asarray(data.xpos[root])
    result = []
    for suffix in LEG_CODES:
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"tibia{suffix}")
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"tibia{suffix}_collision_0")
        if gid < 0 or body < 0:
            raise ValueError(f"Missing CAD tibia mesh {suffix}")
        joints = [_joint_on_body(model, mujoco, f"{segment}{suffix}")
                  for segment in ("coxa", "femur", "tibia")]
        mesh_world = geom_mesh_world_vertices(model, data, gid)
        knee = np.asarray(data.xanchor[joints[2]])
        lengths = np.linalg.norm(mesh_world - knee, axis=1)
        tip_world = mesh_world[int(np.argmax(lengths))]
        body_rot = np.asarray(data.xmat[body]).reshape(3,3)
        tip_local = body_rot.T @ (tip_world - np.asarray(data.xpos[body]))
        hip = np.asarray(data.xanchor[joints[0]])
        radial = hip[:2] - root_pos[:2]
        if np.linalg.norm(radial) <= 0.03:
            raise ValueError(f"No radial hip direction: {suffix}")
        result.append({
            "suffix": suffix,
            "body": body,
            "geom": gid,
            "jids": joints,
            "qadrs": [int(model.jnt_qposadr[j]) for j in joints],
            "dadrs": [int(model.jnt_dofadr[j]) for j in joints],
            "hip_xyz": hip.copy(),
            "tip_local": tip_local,
            "tip_at_zero_world": tip_world.copy(),
            "radial_unit_xy": radial / np.linalg.norm(radial),
            "knee_to_tip_m": float(lengths.max()),
            "mesh_vertex_count": int(mesh_world.shape[0]),
        })
    return result


def tip_world(model, data, leg: dict) -> np.ndarray:
    body = leg["body"]
    return (np.asarray(data.xpos[body]) +
            np.asarray(data.xmat[body]).reshape(3,3) @ leg["tip_local"])


def report_geometry(model, data, mujoco, legs: list[dict]) -> dict:
    root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link_2")
    return {
        "source": "CAD derived from MattiaGrigoli Freenove FNK0052, pinned external meshes",
        "root_world_xyz_m": [float(x) for x in data.xpos[root]],
        "legs": [{
            "name": l["suffix"],
            "joint_names": [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in l["jids"]],
            "hip_world_xyz_m": [round(float(x), 6) for x in l["hip_xyz"]],
            "tip_at_zero_world_xyz_m": [round(float(x), 6) for x in l["tip_at_zero_world"]],
            "knee_to_tip_distance_m": round(l["knee_to_tip_m"], 6),
            "radial_xy": [round(float(x), 6) for x in l["radial_unit_xy"]],
            "mesh_vertices": l["mesh_vertex_count"],
        } for l in legs],
        "unverified_kinematic_state": True
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=("nominal", "lower", "upper"), default="nominal")
    parser.add_argument("--cache", type=Path, default=DEFAULT_CACHE)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    import mujoco
    path, _ = build(args.scenario, cache=args.cache, offline=True)
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    legs = load_legs(model, data, mujoco)
    report = report_geometry(model, data, mujoco, legs)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
