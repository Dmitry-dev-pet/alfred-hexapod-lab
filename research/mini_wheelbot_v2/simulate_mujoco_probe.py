#!/usr/bin/env python3
"""Mini Wheelbot V2: preliminary honest MuJoCo *contact* probe.

This script uses three verified source STL meshes for visuals, identified masses
and principal inertias, and distinct joints for driving and reaction wheels.
Collision shapes, wheel placements, motor voltage and torque limits are
EXPERIMENTAL ASSUMPTIONS, not original hardware specs or validated CAD transforms.
The results cannot certify self-righting of the physical Mini Wheelbot.
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
import trimesh

from audit_original_cad import acquire

ROOT = Path(__file__).resolve().parent
UPSTREAM = "wheelbot/Mini-Wheelbot"
CAD_SUBDIR = "hardware/mechanical/meshes"
VOLTAGE_ASSUMED = 22.2
MOTOR_KV_RPM_PER_VOLT = 380  # T-Motor MN4006 KV380; no-load idealization
NO_LOAD_SPEED_RAD_S = VOLTAGE_ASSUMED * MOTOR_KV_RPM_PER_VOLT * 2 * math.pi / 60
TORQUE_LEVELS_NM = (0.0, 0.03, 0.10, 0.20)
DT = 0.001
DURATION_S = 8.0
REPLAY_INTERVAL = 0.05


def source_inertia_for_mujoco(params: dict) -> dict:
    """Project identified principal inertias minimally onto triangle inequality.

    Official sysid I_Bx + I_Bz < I_By, impossible for a rigid body's
    principal moments. Explicit Euclidean correction is an experiment,
    not a statement about the real hardware.
    """
    original = [params["I_Bx"], params["I_By"], params["I_Bz"]]
    ix, iy, iz = original
    violation = max(0., iy - ix - iz)
    if violation:
        delta = violation / 3.
        ix, iy, iz = ix + delta, iy - delta, iz + delta
    if any(v <= 0 for v in (ix, iy, iz)) or max(ix, iy, iz) > sum((ix, iy, iz)) - max(ix, iy, iz) + 1e-12:
        raise ValueError("Inertia projection failed")
    return {
        "original_kg_m2": original,
        "projected_kg_m2": [ix, iy, iz],
        "triangle_violation_kg_m2": violation
    }


def compiled_visual_body(cache: Path) -> Path:
    """Convert verified original binary STL to OBJ without decimation."""
    cad = cache / CAD_SUBDIR
    source = cad / "body.stl"
    derived = cad / "body-source-unaltered-faces.obj"
    if not derived.is_file():
        src = trimesh.load(source, force="mesh")
        if len(src.faces) != 255098:
            raise ValueError("Unexpected original triangle count")
        src.export(derived)
    return derived


def build_xml(cache: Path, params: dict, torque_limit: float) -> str:
    cad = cache / CAD_SUBDIR
    body, rotor = compiled_visual_body(cache), (cad / "actuator_rotor.stl")
    for path in (body, rotor):
        if not path.is_file():
            raise RuntimeError("Missing verified original source mesh: " + str(path))
    mb, mw = params["m_B"], params["m_WR"]
    ix, iy, iz = source_inertia_for_mujoco(params)["projected_kg_m2"]
    iside, iaxial = params["I_Wxz_Ryz"], params["I_Wy_Rx"]
    wheel_r = params["r_W"]
    # Axis convention inferred from sysid names: drive hinge Y / reaction X.
    # Physical collision dimensions and ±0.043 m joint centers are hypotheses.
    return f"""<mujoco model="Mini_Wheelbot_CAD_V2_preliminary_probe">
  <compiler angle="radian"/>
  <option timestep="{DT}" gravity="0 0 -9.81" integrator="implicitfast" iterations="80"/>
  <asset>
    <mesh name="original_body" file="{body}" scale=".001 .001 .001"/>
    <mesh name="original_rotor" file="{rotor}" scale=".001 .001 .001"/>
  </asset>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 .05" friction="1.0 .01 .001"/>
    <body name="chassis" pos="0 0 .075">
      <freejoint name="base"/>
      <inertial pos="0 0 0" mass="{mb}" diaginertia="{ix} {iy} {iz}"/>
      <geom name="authentic_body_visual" type="mesh" mesh="original_body"
            contype="0" conaffinity="0" group="1" rgba=".6 .75 .85 1"/>
      <geom name="assumed_body_collision" type="box"
            size=".042 .040 .049" friction=".85 .01 .001" rgba=".45 .65 .75 .2"/>
      <body name="drive_wheel" pos="0 0 -.043">
        <joint name="drive_axis" type="hinge" axis="0 1 0" damping=".0002"/>
        <inertial pos="0 0 0" mass="{mw}"
                  diaginertia="{iside} {iaxial} {iside}"/>
        <geom name="drive_collision" type="cylinder" size="{wheel_r} .014"
              quat=".707106781 .707106781 0 0"
              friction=".9 .01 .001" rgba=".3 .3 .35 .4"/>
        <geom name="original_drive_rotor_visual" type="mesh" mesh="original_rotor"
              quat=".707106781 0 0 .707106781" contype="0" conaffinity="0" group="1"
              rgba=".75 .7 .55 1"/>
      </body>
      <body name="reaction_wheel" pos="0 0 .043">
        <joint name="reaction_axis" type="hinge" axis="1 0 0" damping=".0002"/>
        <inertial pos="0 0 0" mass="{mw}"
                  diaginertia="{iaxial} {iside} {iside}"/>
        <geom name="reaction_collision" type="cylinder" size="{wheel_r} .014"
              quat=".707106781 0 .707106781 0"
              friction=".9 .01 .001" rgba=".4 .35 .3 .3"/>
        <geom name="original_reaction_rotor_visual" type="mesh" mesh="original_rotor"
              contype="0" conaffinity="0" group="1" rgba=".7 .63 .45 1"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="drive" joint="drive_axis" gear="1"
       ctrllimited="true" ctrlrange="-{torque_limit} {torque_limit}"/>
    <motor name="reaction" joint="reaction_axis" gear="1"
       ctrllimited="true" ctrlrange="-{torque_limit} {torque_limit}"/>
  </actuator>
</mujoco>"""


def signed_roll_and_tilt(data: mujoco.MjData, body_id: int) -> tuple[float, float]:
    axis_z = data.xmat[body_id].reshape(3, 3)[:, 2]
    roll = math.atan2(-axis_z[1], axis_z[2])
    tilt = math.degrees(math.acos(float(np.clip(axis_z[2], -1, 1))))
    return roll, tilt


def run_one(cache: Path, params: dict, limit: float, initial_roll: float,
            seconds: float = DURATION_S) -> dict:
    xml = build_xml(cache, params, limit)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "reaction_axis")
    motor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "reaction")
    dof = model.jnt_dofadr[joint_id]
    # This is a drop from a side-oriented configuration; no teleport during sim.
    # Start 1 cm above approximate collision envelope to avoid geometric overlap.
    data.qpos[:3] = (0.0, 0.0, 0.055)
    tilt = math.radians(initial_roll)
    data.qpos[3:7] = (math.cos(tilt / 2), math.sin(tilt / 2), 0, 0)
    mujoco.mj_forward(model, data)
    traj = []
    max_penetration = 0.0
    max_flywheel_speed = 0.0
    max_applied_torque = 0.0
    total_contact_steps = 0
    fall_below_floor = False
    limit_hits = 0
    last_upright_start = None
    longest_upright = 0.
    min_tilt = 180.
    n = int(seconds / DT)
    sample_stride = int(round(REPLAY_INTERVAL / DT))
    for i in range(n):
        roll, ang = signed_roll_and_tilt(data, body_id)
        min_tilt = min(min_tilt, ang)
        angular_rate = float(data.qvel[3])
        flywheel_speed = float(data.qvel[dof])
        if limit <= 0.:
            torque = 0.
        else:
            # Sign: positive torque on reaction wheel yields negative body roll.
            torque = float(np.clip(0.12 * roll + 0.015 * angular_rate, -limit, limit))
            if abs(flywheel_speed) >= NO_LOAD_SPEED_RAD_S and torque * flywheel_speed > 0:
                torque = 0.
                limit_hits += 1
        data.ctrl[0] = 0.
        data.ctrl[motor_id] = torque
        max_applied_torque = max(max_applied_torque, abs(torque))
        max_flywheel_speed = max(max_flywheel_speed, abs(flywheel_speed))
        if data.ncon:
            total_contact_steps += 1
            max_penetration = max(max_penetration,
                                  max(0., -min(float(c.dist) for c in data.contact)))
        if ang < 10:
            if last_upright_start is None:
                last_upright_start = data.time
            longest_upright = max(longest_upright, data.time - last_upright_start)
        else:
            last_upright_start = None
        if data.qpos[2] < -.02:
            fall_below_floor = True
        if i % sample_stride == 0 or i == n - 1:
            traj.append([
                round(float(data.time), 4),
                round(float(data.qpos[0]), 5),
                round(float(data.qpos[1]), 5),
                round(float(data.qpos[2]), 5),
                *[round(float(q), 6) for q in data.qpos[3:7]],
                round(ang, 3), round(math.degrees(roll), 3),
                round(flywheel_speed, 3), round(torque, 5),
                int(data.ncon)
            ])
        mujoco.mj_step(model, data)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise RuntimeError("Numerical divergence; cannot infer success")
    _, final_tilt = signed_roll_and_tilt(data, body_id)
    geometry_assumptions_validated = False
    motor_limits_from_hardware = False
    criterion_observed = (final_tilt < 10 and longest_upright >= 1.
                          and not fall_below_floor and limit_hits == 0)
    # Regardless of numeric outcome, this model is preliminary.
    return {
        "status": "PRELIMINARY_UNVALIDATED_GEOMETRY_AND_MOTOR_LIMITS",
        "observed_criterion_met_in_model": bool(criterion_observed),
        "geometry_assumptions_validated": geometry_assumptions_validated,
        "motor_limits_from_hardware": motor_limits_from_hardware,
        "physics_engine": "MuJoCo",
        "mujoco_version": mujoco.__version__,
        "dt_s": DT, "duration_s": seconds, "initial_roll_deg": initial_roll,
        "torque_cap_assumed_nm": limit, "voltage_assumed_v": VOLTAGE_ASSUMED,
        "max_speed_assumed_rad_s": NO_LOAD_SPEED_RAD_S,
        "final_tilt_deg": final_tilt, "minimum_tilt_deg": min_tilt,
        "longest_upright_s": longest_upright,
        "peak_reaction_speed_rad_s": max_flywheel_speed,
        "peak_torque_nm": max_applied_torque,
        "reaction_speed_limit_hits": limit_hits,
        "maximum_contact_penetration_m": max_penetration,
        "steps_with_contact": total_contact_steps,
        "floor_breach": fall_below_floor,
        "frames": traj,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cache", type=Path, default=Path.home() / "mini-wheelbot-original-cad")
    ap.add_argument("--out", type=Path, default=Path("/tmp/wheelbot-v2-probe"))
    ap.add_argument("--seconds", type=float, default=DURATION_S)
    args = ap.parse_args()
    if args.seconds < 2 or args.seconds > 30:
        ap.error("duration must be between 2 and 30 seconds")
    verified = acquire(args.cache)
    params = verified["params"]
    args.out.mkdir(parents=True, exist_ok=True)
    runs = []
    for init in (90, 60):
        for cap in TORQUE_LEVELS_NM:
            run = run_one(args.cache, params, cap, init, args.seconds)
            samples = run.pop("frames")
            key = f"side_{init:02d}_torque_{cap:.2f}"
            out = args.out / f"{key}.json"
            out.write_text(json.dumps({"model": "Mini Wheelbot CAD V2 preliminary",
                                       "run": run, "frame_columns":
                                       ["t", "x", "y", "z", "qw", "qx", "qy", "qz",
                                        "tilt_deg", "signed_roll_deg",
                                        "flywheel_omega_rad_s", "torque_nm", "contacts"],
                                       "frames": samples}, separators=(",", ":")) + "\n")
            run["trajectory_file"] = out.name
            run["trajectory_sha256"] = hashlib.sha256(out.read_bytes()).hexdigest()
            runs.append(run)
            print(key, "final", round(run["final_tilt_deg"], 2),
                  "min", round(run["minimum_tilt_deg"], 2),
                  "speed", round(run["peak_reaction_speed_rad_s"], 1),
                  "criterion", run["observed_criterion_met_in_model"], flush=True)
    summary = {
        "classification": "PRELIMINARY_PHYSICS_PROBE_NOT_HARDWARE_VALIDATED",
        "upstream": UPSTREAM,
        "upstream_commit": verified["upstream_commit"],
        "original_meshes_verification": {x["path"]: x["sha256"]
                                        for x in verified["source_files"]},
        "inertia_correction": source_inertia_for_mujoco(params),
        "visual_mesh_conversion": {
            "method": "trimesh STL-to-OBJ, all 255098 triangles retained",
            "derived_obj_sha256": hashlib.sha256(
                compiled_visual_body(args.cache).read_bytes()).hexdigest()
        },
        "model_xml_sha256_example": hashlib.sha256(
            build_xml(args.cache, params, .1).encode()).hexdigest(),
        "assumptions": [
            "Original source meshes are used visually, collision shapes are boxes/cylinders",
            "Joint axis assignment derived from original sysid notation, not verified CAD assembly",
            "±0.043m wheel-center placements are assumed, not measured from STEP",
            "22.2V assumed from 6 cells; 380 KV is no-load speed estimate, not real RPM cap",
            "Torque test set is hypothetical 0 to 0.20 Nm, not verified drive/battery limit",
            "Original sysid inertias violate rigid-body triangle inequality; minimum Euclidean correction used",
            "MuJoCo STL decoder triangle limit: converted original body STL to OBJ with all triangles retained",
            "The model is not a physical digital twin and cannot verify self-righting"
        ],
        "runs": runs
    }
    report = args.out / "summary.json"
    report.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    print("RESULT:", report, flush=True)


if __name__ == "__main__":
    main()
