"""Exploratory physics probe for an APPROXIMATE EN01 model. Not source firmware or a trained policy."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

MODEL = Path(__file__).with_name("model_v0.xml")
UPSTREAM_SHA = "3139f2fb32b38d10611feb905e16f8aa012f2f2e"
SERVO_RATED_NM = 1.5 * 0.0980665
SERVO_STALL_NM = 4.5 * 0.0980665
SERVO_NOLOAD_RAD_S = (math.pi / 3) / 0.09  # 0.09 s per 60 deg (6 V)
WHEEL_ASSUMED_NM = 0.030
CROUCH_RAD = (12 + 8.4 * (80 - 32)) * math.tau / 4096
EXTEND_RAD = (12 + 8.4 * (40 - 32)) * math.tau / 4096


def clamp(value: float, maximum: float) -> float:
    return max(-maximum, min(maximum, value))


def setpoint(t: float, mode: str) -> tuple[float, bool]:
    """Probe targets. Timings are OUR assumptions, not original firmware timing."""
    if mode == "stand":
        return EXTEND_RAD, False
    if mode in ("crouch", "jump"):
        # Limited-speed preparation tests if normal motion destabilises the robot.
        if t < 1.5:
            return EXTEND_RAD, False
        if t < 3.35:
            return EXTEND_RAD + (CROUCH_RAD - EXTEND_RAD) * (t - 1.5) / 1.85, False
        if mode == "crouch" or t < 3.60:
            return CROUCH_RAD, False
        return EXTEND_RAD, (3.60 <= t < 3.75)
    if mode == "jump_fast":
        if t < 1.5:
            return EXTEND_RAD, False
        if t < 2.1:
            return CROUCH_RAD, False
        return EXTEND_RAD, (2.1 <= t < 2.25)
    raise ValueError(mode)


def run(mode: str, duration: float, output: Path, model_path: Path = MODEL) -> dict:
    try:
        import mujoco
        import numpy as np
    except ImportError as exc:
        raise SystemExit("A local MuJoCo 3.x runtime + numpy is required; NO physics was run.") from exc

    m = mujoco.MjModel.from_xml_path(str(model_path))
    d = mujoco.MjData(m)
    torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso")
    left_wheel_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "left_wheel")
    right_wheel_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "right_wheel")
    floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    torso_geom = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "torso_geom")
    joint = {name: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name) for name in
             ("hip_left", "hip_right", "wheel_left", "wheel_right")}
    actuator = {name: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "a_" + name)
                for name in joint}
    # Only the measured source contains mirrored servo position counts;
    # model-space hip axes are aligned, so both targets use the same sign.
    wheel_geom = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "left_wheel_geom")
    wheel_radius_m = float(m.geom_size[wheel_geom, 0])
    frames = []
    launch_t = {"jump": 3.6, "jump_fast": 2.1}.get(mode)
    flight_start = None
    flight_takeoff_upright = False
    flight_takeoff_vz = 0.0
    flight_durations = []
    accepted_flights = []
    contacted_after_launch = False
    qualified_landing_started = False
    stable_after_touchdown_s = 0.0
    max_stable_landing_s = 0.0
    max_min_wheel_clearance = -1.0
    floor_penetration_max = 0.0
    prelaunch_samples = 0
    prelaunch_valid_samples = 0
    max_abs_pitch = 0.0
    torso_ground_steps = 0
    max_abs_x = 0.0
    mujoco.mj_forward(m, d)
    prev_com_z = float(d.subtree_com[torso, 2])

    steps = math.ceil(duration / m.opt.timestep)
    for step in range(steps):
        t = float(d.time)
        target, peak_trial = setpoint(t, mode)
        for name in ("hip_left", "hip_right"):
            j = joint[name]
            pos = float(d.qpos[m.jnt_qposadr[j]])
            speed = float(d.qvel[m.jnt_dofadr[j]])
            raw = 3.0 * (target - pos) - 0.045 * speed
            # Assumed linear speed/torque envelope; actual Feetech curve unmeasured.
            torque_limit = (SERVO_STALL_NM if peak_trial else SERVO_RATED_NM)
            torque_limit *= max(0.0, 1.0 - abs(speed) / SERVO_NOLOAD_RAD_S)
            if speed * raw > 0 and abs(speed) >= SERVO_NOLOAD_RAD_S:
                raw = 0
            d.ctrl[actuator[name]] = clamp(raw, torque_limit)

        R = d.xmat[torso].reshape(3, 3)
        pitch = math.atan2(-float(R[2, 0]), float(R[2, 2]))
        gyro_y = float(d.sensor("imu_gyro").data[1])
        mean_wheel_rate = sum(float(d.qvel[m.jnt_dofadr[joint[n]]])
                              for n in ("wheel_left", "wheel_right")) / 2
        # Diagnostic wheel stabilizer, NOT original firmware and NOT tuned to EN01.
        wheel_ctrl = clamp(0.15 * pitch + 0.050 * gyro_y + 0.008 * mean_wheel_rate,
                           WHEEL_ASSUMED_NM)
        d.ctrl[actuator["wheel_left"]] = wheel_ctrl
        d.ctrl[actuator["wheel_right"]] = wheel_ctrl

        mujoco.mj_step(m, d)
        contact_total_n = 0.0
        torso_ground = False
        floor_ground = False
        penetration = 0.0
        for c in range(d.ncon):
            contact = d.contact[c]
            if floor not in (int(contact.geom1), int(contact.geom2)):
                continue
            floor_ground = True
            torso_ground |= torso_geom in (int(contact.geom1), int(contact.geom2))
            wrench = np.zeros(6, dtype=float)
            mujoco.mj_contactForce(m, d, c, wrench)
            contact_total_n += max(0.0, float(wrench[0]))
            penetration = max(penetration, max(0.0, -float(contact.dist)))
        floor_penetration_max = max(floor_penetration_max, penetration)
        clearance = min(float(d.xpos[b, 2]) - wheel_radius_m
                        for b in (left_wheel_body, right_wheel_body))
        # Record physical contact on every physics step; never infer flight from video frames.
        post_R = d.xmat[torso].reshape(3, 3)
        post_pitch = math.atan2(-float(post_R[2, 0]), float(post_R[2, 2]))
        max_abs_pitch = max(max_abs_pitch, abs(post_pitch))
        max_abs_x = max(max_abs_x, abs(float(d.qpos[0])))
        torso_ground_steps += int(torso_ground)
        com_z = float(d.subtree_com[torso, 2])
        com_vz = (com_z - prev_com_z) / m.opt.timestep
        prev_com_z = com_z
        if launch_t is not None:
            if launch_t - 0.25 <= float(d.time) < launch_t:
                prelaunch_samples += 1
                prelaunch_valid_samples += int(floor_ground and not torso_ground and abs(post_pitch) < 0.35)
            if float(d.time) >= launch_t:
                max_min_wheel_clearance = max(max_min_wheel_clearance, clearance)
                if not floor_ground:
                    if flight_start is None and contacted_after_launch:
                        flight_start = float(d.time)
                        flight_takeoff_upright = abs(post_pitch) < 0.35 and not torso_ground
                        flight_takeoff_vz = com_vz
                else:
                    contacted_after_launch = True
                    if flight_start is not None:
                        duration_flight = float(d.time) - flight_start
                        flight_durations.append(duration_flight)
                        if (duration_flight >= 0.020 and flight_takeoff_upright
                                and flight_takeoff_vz > 0.05):
                            accepted_flights.append(duration_flight)
                            qualified_landing_started = True
                        flight_start = None
                    if qualified_landing_started and abs(post_pitch) < 0.35 and not torso_ground:
                        stable_after_touchdown_s += m.opt.timestep
                        max_stable_landing_s = max(max_stable_landing_s, stable_after_touchdown_s)
                    else:
                        stable_after_touchdown_s = 0.0

        if step % 10 == 0:  # every 20 ms at default 0.002 s physics dt
            frames.append({
                "t": round(float(d.time), 6),
                "qpos": d.qpos.tolist(), "qvel": d.qvel.tolist(),
                "ctrl": d.ctrl.tolist(), "pitch_rad": pitch,
                "gyro_y_rad_s": gyro_y, "ground_contact": floor_ground,
                "torso_ground_contact": torso_ground,
                "ground_normal_force_n": contact_total_n,
                "min_wheel_clearance_m": clearance, "penetration_m": penetration,
                "short_peak_trial": peak_trial,
            })
    if flight_start is not None:
        flight_durations.append(float(d.time) - flight_start)

    prelaunch_pass = (prelaunch_samples >= 100 and
                      prelaunch_valid_samples / prelaunch_samples >= 0.95)
    qualified_jump = bool(accepted_flights) and prelaunch_pass
    state_pass = torso_ground_steps == 0 and max_abs_pitch < 0.35 and max_abs_x < 0.5
    stats = {
        "mode": mode, "duration_s": duration, "physics_steps": steps,
        "mujoco_version": mujoco.__version__, "source_sha": UPSTREAM_SHA,
        "model": ("source CAD nominal pose surrogate; four-bar NOT VALIDATED"
                  if model_path.name == "model_v1_cad_pose.xml" else
                  "hand-built approximate EN01, NOT upstream CAD"),
        "model_file": model_path.name,
        "model_sha256": hashlib.sha256(model_path.read_bytes()).hexdigest(),
        "controller": "exploratory PD, NOT upstream firmware, NOT trained policy",
        "rated_servo_nm": SERVO_RATED_NM, "stall_servo_nm": SERVO_STALL_NM,
        "wheel_torque_nm_assumed": WHEEL_ASSUMED_NM,
        "raw_full_body_no_contact_intervals_s": flight_durations,
        "candidate_upright_upward_flights_s": accepted_flights,
        "prelaunch_upright_supported": prelaunch_pass,
        "prelaunch_valid_fraction": (prelaunch_valid_samples / prelaunch_samples
                                      if prelaunch_samples else None),
        "has_flight_ge_20ms": qualified_jump,
        "torso_ground_steps": torso_ground_steps,
        "max_abs_pitch_rad": max_abs_pitch,
        "max_abs_x_m": max_abs_x,
        "stable_stand_or_crouch": state_pass if mode in ("stand", "crouch") else None,
        "max_min_wheel_clearance_m": max_min_wheel_clearance if mode == "jump" else None,
        "stable_landing_2s_observed": qualified_jump and max_stable_landing_s >= 2.0,
        "max_floor_penetration_m": floor_penetration_max,
        "result_classification": "EXPLORATORY_ONLY_NOT_SIM_TO_REAL_VALIDATED",
        "frames": frames,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(stats, indent=2, ensure_ascii=False), encoding="utf-8")
    return {k: v for k, v in stats.items() if k != "frames"}


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--mode", choices=["stand", "crouch", "jump", "jump_fast"], default="stand")
    p.add_argument("--duration", type=float, default=6)
    p.add_argument("--model", type=Path, default=MODEL,
                   help="Explicit model variant; default remains original v0")
    p.add_argument("--out", type=Path, default=Path(".cache/navbot_en01/probe.json"))
    args = p.parse_args()
    if args.duration <= 0:
        p.error("duration must be positive")
    print(json.dumps(run(args.mode, args.duration, args.out, args.model), indent=2))


if __name__ == "__main__":
    main()
