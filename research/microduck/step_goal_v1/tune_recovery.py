#!/usr/bin/env python3
"""Bounded step-goal search around fixed MicroDuck v8 ONNX policies.

Only policy commands and session-switch timing vary. No runtime qpos/qvel
assignment, external force, geometry change, or neural-weight update occurs.
"""
from __future__ import annotations

import argparse
import contextlib
import hashlib
import io
import itertools
import json
import math
import os
from pathlib import Path
import sys
import time

import mujoco as mj
import numpy as np

HERE = Path(__file__).resolve().parent
DT, CDT = .0005, .02
ROLL_SHA = "3d60da08fc13f29c1b57f41977aa898132c0d60042100149d8e775affcbca32b"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def initialize(workspace):
    os.environ["MICRODUCK_WORKSPACE"] = str(workspace)
    sys.path.insert(0, str(HERE / "source"))
    # Preload pinned local dependencies before legacy path adapters run.
    import bam_backlash_cpu
    import step_cpu
    import fall_cpu
    import measure_clearance
    import wall_cpu
    import collision_model
    global E, BUILD, IP, ROLL, WORKSPACE
    WORKSPACE = workspace
    E, BUILD = collision_model.terrain, collision_model.build
    manifest = json.loads((workspace / "microduck-work/source-manifest.json").read_text())
    manifest["_path"] = str(workspace / "microduck-work/source-manifest.json")
    gate = E.bench.source_gate(E.RL, E.BAM, manifest)
    assert sha(E.POLICY) == E.bench.POLICY_SHA256
    ROLL = workspace / "microduck-roll/roulade.onnx"
    assert sha(ROLL) == ROLL_SHA
    IP = E.bench.upstream_inference(E.RL)
    IP.ort.InferenceSession = E.bench.cpu_session
    return gate


def sole_inside(bounds, edge):
    return all(b["min"][0] > edge + .001 and b["max"][0] < edge + 1.2 - .001
               and b["min"][1] > -.599 and b["max"][1] < .599
               for b in bounds.values())


def quiet(f, height):
    return (f["tilt_deg"] < 20 and .09 < f["trunk_z_m"] < .18 + height
            and f["linear_speed_mps"] < .06 and f["angular_speed_rps"] < .7
            and all(v > .05 for v in f["foot_loads_n"].values()))


def run(params):
    height, edge = params["height_m"], params["edge_x_m"]
    E.EDGE = edge
    with contextlib.redirect_stdout(io.StringIO()):
        m, d, ctrl, _ = BUILD(IP, height)
        mj.mj_resetDataKeyframe(m, d, mj.mj_name2id(m, mj.mjtObj.mjOBJ_KEY, "STAND"))
        joints = m.actuator_trnid[:, 0]
        sq = m.jnt_qposadr[joints]
        # Initial perturbations only; never applied after the first mj_step.
        seed = params.get("seed", 0)
        if seed:
            rng = np.random.default_rng(seed)
            d.qpos[sq] += rng.uniform(-.005, .005, len(sq))
        ctrl.reset(d.qpos)
        ctrl.last_ts = float(d.time)
        mj.mj_forward(m, d)
        p = IP.PolicyInference(m, d, walking_onnx_path=str(E.POLICY),
                               roulade_onnx_path=str(ROLL),
                               roulade_duration=params["roll_duration_s"],
                               bam_ctrl=ctrl, use_projected_gravity=True, new_cmd_obs=True)
        E.bench.install_encoder_view(p, ctrl, m, d)
        bq, _ = E.bench.backlash_indices(m)
        ctrl.q_target[:] = d.qpos[sq] + d.qpos[bq]
        p.set_vel_cmd(0, 0, 0)
    assert not p.use_delay and m.opt.timestep == DT
    margin = params.get("motor_target_margin_rad", .035)
    lo, hi = m.jnt_range[joints, 0] + margin, m.jnt_range[joints, 1] - margin
    floor = mj.mj_name2id(m, mj.mjtObj.mjOBJ_GEOM, "floor")
    step = mj.mj_name2id(m, mj.mjtObj.mjOBJ_GEOM, "step_top")
    feet = {s: mj.mj_name2id(m, mj.mjtObj.mjOBJ_GEOM, s + "_foot_collision") for s in ("left", "right")}
    root = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, "trunk_base")
    if root < 0:
        root = 1
    initial = {"qpos": d.qpos.tolist(), "qvel": d.qvel.tolist()}
    trigger = handback = first_top = None
    trigger_state = None
    frames, approach_velocity = [], []
    max_env = max_body_step = max_self = max_torque = max_limit = peak_force = 0.
    resets = False
    angle = frontier = 0.
    command = 0.
    goal_seen = False
    phase = "settle"
    # Every episode receives >= 4 s recovery, even with late triggers.
    end = 9.
    k = 0
    began = time.monotonic()
    while k * CDT < end - 1e-9:
        t = round(k * CDT, 8)
        mj.mj_forward(m, d)
        bounds = {s: E.foot_bounds(m, d, g) for s, g in feet.items()}
        gap = edge - max(b["max"][0] for b in bounds.values())
        if t >= 1 and phase == "settle":
            phase, command = "approach", params["approach_command_mps"]
        if phase == "approach":
            approach_velocity.append(float(d.qvel[0]))
            if gap <= params["trigger_gap_m"] or params.get("flat_control", False):
                trigger, phase, command = t, "roll", 0.
                trigger_state = {"qpos": d.qpos.tolist(), "qvel": d.qvel.tolist(), "toe_gap_m": gap}
                with contextlib.redirect_stdout(io.StringIO()):
                    p.trigger_behavior("roulade")
                end = max(end, t + params["roll_duration_s"] + 4.)
            elif t >= 6.:
                phase, command = "no_trigger", 0.
        with contextlib.redirect_stdout(io.StringIO()):
            old_behavior = p.behavior_mode
            p.update_behavior(CDT)
            if old_behavior and not p.behavior_mode:
                handback, phase = t, "recovery"
            if phase == "recovery":
                # Forward assistance is an input to the same frozen policy.
                # Stop when full support is detected, or after a bounded 2 s.
                command = params["recovery_command_mps"] if t < handback + 2 and not goal_seen else 0.
            p.set_vel_cmd(command, 0, 0)
        action = p.infer()
        target = np.clip(p.default_pose + action * p.action_scale, lo, hi)
        ctrl.q_target[:] = target
        p.last_action = ((target - p.default_pose) / p.action_scale).astype(np.float32)
        for _ in range(round(CDT / DT)):
            before = float(d.time)
            ctrl.update()
            mj.mj_step(m, d)
            resets |= abs(float(d.time) - before - DT) > 1e-8
            if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all():
                raise FloatingPointError("Nonfinite physics")
            max_torque = max(max_torque, float(np.abs(d.actuator_force).max()))
            max_limit = max(max_limit, float(np.maximum(m.jnt_range[joints, 0] - d.qpos[sq],
                                                        d.qpos[sq] - m.jnt_range[joints, 1]).max()))
            if trigger is not None:
                vel = np.empty(6)
                mj.mj_objectVelocity(m, d, mj.mjtObj.mjOBJ_XBODY, root, vel, 1)
                angle += float(vel[1]) * DT
                frontier = max(frontier, angle)
            for ct in d.contact:
                pair = (int(ct.geom1), int(ct.geom2))
                env = floor in pair or step >= 0 and step in pair
                if env:
                    max_env = max(max_env, -float(ct.dist))
                    if step >= 0 and step in pair:
                        other = pair[1] if pair[0] == step else pair[0]
                        if m.body_weldid[m.geom_bodyid[other]] == root:
                            max_body_step = max(max_body_step, -float(ct.dist))
                else:
                    max_self = max(max_self, -float(ct.dist))
        mj.mj_forward(m, d)
        loads, top = {s: 0. for s in feet}, {s: 0. for s in feet}
        contacts = []
        for i, ct in enumerate(d.contact):
            pair = (int(ct.geom1), int(ct.geom2))
            wrench = np.zeros(6)
            mj.mj_contactForce(m, d, i, wrench)
            contacts.append({"geoms": list(pair), "force_n": float(wrench[0]),
                             "distance_m": float(ct.dist), "pos": ct.pos.tolist(),
                             "normal": ct.frame[:3].tolist()})
            if floor in pair or step >= 0 and step in pair:
                peak_force = max(peak_force, float(np.linalg.norm(wrench[:3])))
            if abs(ct.frame[2]) > .85:
                for s, g in feet.items():
                    if g in pair and (floor in pair or step >= 0 and step in pair):
                        loads[s] += max(0., float(wrench[0]))
                    if g in pair and step >= 0 and step in pair and ct.pos[2] >= height - .001:
                        top[s] += max(0., float(wrench[0]))
        bounds = {s: E.foot_bounds(m, d, g) for s, g in feet.items()}
        complete_top = bool(height and sole_inside(bounds, edge) and all(v > .05 for v in top.values()))
        goal_seen |= complete_top
        if complete_top and first_top is None:
            first_top = float(d.time)
        frames.append({"t": round(float(d.time), 6), "phase": phase,
                       **E.bench.sample(m, d, 0, 0), "command_mps": command,
                       "motor_target_rad": target.tolist(), "foot_bounds": bounds,
                       "foot_loads_n": loads, "step_loads_n": top, "contacts": contacts,
                       "complete_top_support": complete_top,
                       "forward_rotation_deg": math.degrees(angle)})
        k += 1
    last = frames[-50:]
    standing = all(quiet(f, height) for f in last)
    full_top = bool(height and standing and all(f["complete_top_support"] for f in last))
    warnings = {str(i): int(w.number) for i, w in enumerate(d.warning) if w.number}
    quality = not resets and not warnings and max_env < .001 and max_self < .001 and max_limit < math.radians(1)
    # Diagnostic ranking never changes the binary goal or quality thresholds.
    longest = current = 0
    for f in frames:
        current = current + 1 if f["complete_top_support"] else 0
        longest = max(longest, current)
    summary = {"params": params, "trigger_s": trigger, "actual_trigger": trigger_state,
               "handback_s": handback, "standing_final_1s": standing,
               "step_goal_reached": full_top, "quality_gate": quality,
               "status": "PASS" if quality and (full_top if height else standing) else "FAIL",
               "max_forward_rotation_deg": math.degrees(frontier),
               "final_forward_rotation_deg": math.degrees(angle),
               "first_complete_top_support_s": first_top,
               "longest_complete_top_support_s": longest * CDT,
               "max_trunk_x_m": max(f["qpos"][0] for f in frames),
               "final_position_m": frames[-1]["qpos"][:3],
               "max_environment_penetration_m": max_env, "max_body_step_penetration_m": max_body_step,
               "max_self_contact_penetration_m": max_self,
               "max_motor_limit_overshoot_rad": max(0., max_limit),
               "peak_torque_nm": max_torque, "peak_sampled_environment_force_n": peak_force,
               "max_actual_approach_vx_mps": max(approach_velocity, default=0.),
               "warnings": warnings, "physics_reset_detected": resets,
               "duration_s": frames[-1]["t"], "frames": len(frames)}
    return {"schema": "microduck-step-goal-tuning/v1", "physics_dt_s": DT, "policy_dt_s": CDT,
            "stand_policy_sha256": sha(E.POLICY), "roll_policy_sha256": sha(ROLL),
            "terrain": {"height_m": height, "edge_x_m": edge, "half_size_xy_m": [.6, .6],
                        "center_xy_m": [edge + .6, 0]},
            "initial_state": initial, "summary": summary, "frames": frames}, time.monotonic() - began


def save(params, name, out):
    path = out / (name + ".json")
    if path.exists():
        trace = json.loads(path.read_text())
        assert trace["summary"]["params"] == params
        elapsed = 0.
    else:
        trace, elapsed = run(params)
        path.write_text(json.dumps(trace, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n")
    s = {**trace["summary"], "trace_file": path.name, "trace_sha256": sha(path)}
    print(json.dumps({"name": name, "status": s["status"], "top_s": s["longest_complete_top_support_s"],
                      "stand": s["standing_final_1s"], "rotation": s["max_forward_rotation_deg"],
                      "quality": s["quality_gate"], "elapsed_s": round(elapsed, 2)}), flush=True)
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workspace", type=Path, required=True)
    ap.add_argument("--smoke", action="store_true")
    ap.add_argument("--out", type=Path, default=HERE / "traces")
    a = ap.parse_args()
    gate = initialize(a.workspace.resolve())
    a.out.mkdir(parents=True, exist_ok=True)
    base = {"height_m": .01, "edge_x_m": .18, "approach_command_mps": .30,
            "trigger_gap_m": .05, "roll_duration_s": 2., "recovery_command_mps": 0.,
            "motor_target_margin_rad": .07, "seed": 0}
    entries = [save({**base, "height_m": 0., "flat_control": True, "approach_command_mps": 0.}, "flat-control", a.out),
               save(base, "step-1cm-baseline", a.out)]
    if not a.smoke:
        for v, gap, duration in itertools.product((.18, .30, .45), (.03, .07), (1., 2., 3.)):
            params = {**base, "approach_command_mps": v, "trigger_gap_m": gap, "roll_duration_s": duration}
            entries.append(save(params, f"search-v{round(v*100):02d}-g{round(gap*100):02d}-d{round(duration*10):02d}", a.out))
        candidates = [e for e in entries if e["params"]["height_m"] and e["quality_gate"] and e["trigger_s"] is not None]
        candidates.sort(key=lambda e: (e["step_goal_reached"], e["longest_complete_top_support_s"],
                                       e["max_forward_rotation_deg"]), reverse=True)
        for rank, e in enumerate(candidates[:2], 1):
            for command in (.12, .24):
                entries.append(save({**e["params"], "recovery_command_mps": command}, f"assist-r{rank}-v{round(command*100)}", a.out))
        pool = [e for e in entries if e["params"]["height_m"] and e["quality_gate"] and e["trigger_s"] is not None]
        pool.sort(key=lambda e: (e["step_goal_reached"], e["longest_complete_top_support_s"], e["max_forward_rotation_deg"]), reverse=True)
        selected = pool[0]
        for height in (.02, .03, .05):
            entries.append(save({**selected["params"], "height_m": height}, f"height-{round(height*100)}cm", a.out))
        for seed in (20261109, 20261110, 20261111):
            entries.append(save({**selected["params"], "seed": seed}, f"perturb-{seed}", a.out))
        # Byte-exact reproducibility is distinct from perturbation robustness.
        repeat, _ = run(selected["params"])
        encoded = json.dumps(repeat, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n"
        assert hashlib.sha256(encoded.encode()).hexdigest() == selected["trace_sha256"]
        repeat_proof = {"trace_file": selected["trace_file"], "sha256": selected["trace_sha256"], "byte_identical": True}
    else:
        selected = repeat_proof = None
    report = {"schema": "microduck-step-goal-tuning-summary/v1", "source_gate": gate,
              "fixed_neural_weights": True, "mujoco_version": mj.__version__,
              "selected": selected, "repeat": repeat_proof, "episodes": entries}
    (a.out / ("smoke-summary.json" if a.smoke else "summary.json")).write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
