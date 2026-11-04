"""Compare an upstream differential-IK solver on FNK0052 without making new robot control code.

This tool is a *benchmark adapter*, not a controller. It marks the four grounded
contact points as MuJoCo sites, optionally asks upstream mink for one tiny
kinematic adjustment, and independently checks pose/foot displacement.

Source model and recorded qpos are restored from user-owned Google Drive ZIPs.
No Freenove STL meshes, measurements, or binaries are checked into GitHub.
"""
from __future__ import annotations

import argparse
import importlib.metadata
import json
from pathlib import Path
import sys
import zipfile

import mujoco
import numpy as np

STANCE = ("ml", "mr", "bl", "br")
FRONT = ("fl", "fr")
REPLAY_PATH = "results/claw_tuck_compare_2026-11-04.json"
FRAME_NAME = "base_link_2"
MARKER_PREFIX = "audit_foot_contact_"
DELTA_BODY = np.array([0.0, 0.00035, -0.00010], dtype=float)


def replay_pose(zip_path: Path) -> np.ndarray:
    with zipfile.ZipFile(zip_path) as archive:
        run = json.loads(archive.read(REPLAY_PATH))
    frames = run["semi_tucked"]["qpos_samples_10hz"]
    frame = min(frames, key=lambda row: abs(float(row["t"]) - 9.3))
    if abs(float(frame["t"]) - 9.3) > 1e-5:
        raise ValueError("Expected recorded pose B at t=9.3 seconds")
    return np.asarray(frame["qpos"], dtype=float)


def contact_sites(model_path: Path, zip_path: Path):
    """Add one nonphysical marker at each mean grounded tibia contact point.

    A site is massless and non-colliding, therefore adding it must not alter
    joint and actuator ordering, mass or inertia. No contact points are faked:
    require four actual tibia-floor contacts from the archived MuJoCo state.
    """
    model = mujoco.MjModel.from_xml_path(str(model_path))
    state = mujoco.MjData(model)
    qpos = replay_pose(zip_path)
    if qpos.shape != (model.nq,):
        raise ValueError("Archived qpos incompatible with MuJoCo scene")
    state.qpos[:] = qpos
    mujoco.mj_forward(model, state)
    floor = int(model.geom("floor").id)
    tibia_geom = {
        int(model.geom(f"tibia{leg}_collision_0").id): leg
        for leg in (*STANCE, *FRONT)
    }
    contacts = {leg: [] for leg in (*STANCE, *FRONT)}
    for i in range(state.ncon):
        c = state.contact[i]
        gid = (
            int(c.geom2) if int(c.geom1) == floor
            else int(c.geom1) if int(c.geom2) == floor
            else -1
        )
        if gid in tibia_geom:
            contacts[tibia_geom[gid]].append(np.asarray(c.pos).copy())
    actual = {name for name, pts in contacts.items() if pts}
    if actual != set(STANCE):
        raise RuntimeError(f"Expected exactly the four stance legs, got {sorted(actual)}")

    spec = mujoco.MjSpec.from_file(str(model_path))
    original_markers = {}
    for leg in STANCE:
        body_id = int(model.body(f"tibia{leg}").id)
        p_world = np.mean(contacts[leg], axis=0)
        rot = np.asarray(state.xmat[body_id]).reshape(3, 3)
        p_local = rot.T @ (p_world - np.asarray(state.xpos[body_id]))
        spec.body(f"tibia{leg}").add_site(
            name=MARKER_PREFIX + leg, pos=p_local,
            size=(0.0005,), type=mujoco.mjtGeom.mjGEOM_SPHERE,
            rgba=(0.4, 0.8, 0.3, 0.0),
        )
        original_markers[leg] = p_world
    marked = spec.compile()
    if (model.nq, model.nv, model.nu, model.nmesh) != (
        marked.nq, marked.nv, marked.nu, marked.nmesh
    ):
        raise AssertionError("Adding sites changed the model layout")
    if not np.allclose(model.body_mass, marked.body_mass):
        raise AssertionError("Adding sites changed body masses")
    if not np.allclose(model.body_inertia, marked.body_inertia):
        raise AssertionError("Adding sites changed body inertias")

    after = mujoco.MjData(marked)
    after.qpos[:] = qpos
    mujoco.mj_forward(marked, after)
    reconstructed = {
        leg: np.asarray(after.site_xpos[marked.site(MARKER_PREFIX + leg).id]).copy()
        for leg in STANCE
    }
    reconstruction_error = max(
        np.linalg.norm(reconstructed[leg] - original_markers[leg]) for leg in STANCE
    )
    if reconstruction_error > 1e-8:
        raise AssertionError(f"Site positions do not match actual contacts: {reconstruction_error}")
    return marked, qpos, original_markers, reconstruction_error


def _read_positions(model, data):
    return {
        leg: np.asarray(data.site_xpos[model.site(MARKER_PREFIX + leg).id]).copy()
        for leg in STANCE
    }


def actual_violations(model, qpos, research_joint_limit=1.7):
    bad = []
    for j in range(model.njnt):
        if model.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        if name is None or not name.startswith("motor"):
            continue
        angle = float(qpos[model.jnt_qposadr[j]])
        if abs(angle) > research_joint_limit:
            bad.append({"joint": name, "position_rad": angle})
    return bad


def try_mink(model, qpos, reference_markers, max_drift_m=0.00002):
    """One local pose-IK probe, returning honest UNAVAILABLE if mink absent.

    Hard hardware approval is ALWAYS false: no servo force, velocity or
    calibration data here. Do not use this output as motor commands.
    """
    try:
        import mink
        import qpsolvers
    except ImportError as exc:
        return {
            "status": "UNAVAILABLE", "reason": str(exc),
            "mink_not_run": True,
            "kinematic_gate_pass": False, "hardware_approved": False,
        }

    configuration = mink.Configuration(model)
    configuration.update(qpos.copy())
    tasks = []
    for leg in STANCE:
        task = mink.FrameTask(
            frame_name=MARKER_PREFIX + leg, frame_type="site",
            position_cost=10.0, orientation_cost=0.0,
        )
        task.set_target_from_configuration(configuration)
        tasks.append(task)
    base_task = mink.FrameTask(
        frame_name=FRAME_NAME, frame_type="body",
        position_cost=1.0, orientation_cost=1.0,
    )
    tform = configuration.get_transform_frame_to_world(FRAME_NAME, "body").as_matrix()
    tform[:3, 3] += DELTA_BODY
    base_task.set_target(mink.SE3.from_matrix(tform))
    tasks.append(base_task)

    posture = mink.PostureTask(model, cost=1e-4)
    posture.set_target_from_configuration(configuration)
    tasks.append(posture)

    # Match our previous benchmark by keeping the two lifted front legs and
    # both head joints fixed. The four stance legs and floating base remain free.
    frozen = []
    # Identify actual leg subtrees by names, not Freenove servo channel IDs.
    # Front legs and non-leg (head) hinges stay fixed for this one-step trial.
    leg_bases = {f"coxa{leg}" for leg in (*STANCE, *FRONT)}
    for j in range(model.njnt):
        if model.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        ancestor = int(model.jnt_bodyid[j])
        parent_leg = None
        while ancestor != 0:
            body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, ancestor)
            if body_name in leg_bases:
                parent_leg = body_name.removeprefix("coxa")
                break
            ancestor = int(model.body_parentid[ancestor])
        if parent_leg is None or parent_leg in FRONT:
            frozen.append(int(model.jnt_dofadr[j]))
    frozen = sorted(set(frozen))
    constraints = [mink.DofFreezingTask(model, dof_indices=frozen)] if frozen else []
    limits = [mink.ConfigurationLimit(model)]
    solve = mink.solve_ik(
        configuration=configuration, tasks=tasks, dt=1.0,
        solver="daqp", damping=1e-9, limits=limits, constraints=constraints,
    )
    candidate = configuration.integrate(solve, dt=1.0)
    after = mujoco.MjData(model)
    after.qpos[:] = candidate
    mujoco.mj_forward(model, after)
    corrected = _read_positions(model, after)
    drift = {leg: float(np.linalg.norm(corrected[leg] - reference_markers[leg])) for leg in STANCE}
    initial_base = mujoco.MjData(model)
    initial_base.qpos[:] = qpos
    mujoco.mj_forward(model, initial_base)
    bid = int(model.body(FRAME_NAME).id)
    displacement = np.asarray(after.xpos[bid]) - np.asarray(initial_base.xpos[bid])
    max_step = max(abs(float(candidate[model.jnt_qposadr[j]] - qpos[model.jnt_qposadr[j]]))
                   for j in range(model.njnt) if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE)
    success = (
        max(drift.values()) < max_drift_m
        and abs(float(displacement[1])-float(DELTA_BODY[1])) < 0.0001
        and max_step < 0.02
    )
    return {
        "status": "RUN", "mink_version": importlib.metadata.version("mink"),
        "solver": "daqp", "max_foot_drift_mm": max(drift.values()) * 1000,
        "achieved_body_xyz_delta_mm": (displacement * 1000).tolist(),
        "body_target_xyz_delta_mm": (DELTA_BODY * 1000).tolist(),
        "max_hinge_delta_rad": max_step,
        "joint_bound": "XML ±pi (NOT hardware)",
        "kinematic_gate_pass": bool(success), "hardware_approved": False,
    }


def benchmark(model_path, replay_zip):
    model, qpos, markers, reconstruct_error = contact_sites(model_path, replay_zip)
    out = {
        "study": "mink-vs-local-kinematic-feasibility-probe-v1",
        "model": "Original FNK0052 CAD30 nominal, 30 meshes + four massless added sites",
        "mujoco_version": mujoco.__version__,
        "source_pose": "saved dynamic semi_tucked B at t=9.3s",
        "qpos_len": int(model.nq), "nv": int(model.nv), "nu": int(model.nu),
        "reconstructed_contact_site_error_m": float(reconstruct_error),
        "research_1p7rad_violations": actual_violations(model, qpos),
        "mink": try_mink(model, qpos, markers),
        "baseline": {
            "source": "Existing PR #29 local WBC on pose B, same contact points",
            "reported_previous_max_drift_mm": 0.000865,
            "reported_previous_max_hinge_delta_rad": 0.0027669522,
            "not_timed_this_run": True,
            "not_same_task_weights": True,
        },
        "warning": "This does not establish dynamic stability, walking, contact planning, hardware torque limits, or comparable real-time performance.",
    }
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--model", type=Path, required=True)
    p.add_argument("--pose-zip", type=Path, required=True)
    p.add_argument("--output", type=Path)
    args = p.parse_args()
    result = benchmark(args.model, args.pose_zip)
    text = json.dumps(result, indent=2, ensure_ascii=False)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    print(text)
    if result["mink"]["status"] == "RUN" and not result["mink"]["kinematic_gate_pass"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()