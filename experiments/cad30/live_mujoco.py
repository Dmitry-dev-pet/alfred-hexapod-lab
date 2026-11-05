"""Independent physical CAD30 live-velocity A/B smoke experiment (Issue #62).

Reuses existing local, owner-verified CAD30 scene, MuJoCo 3.14, stand IK,
CAD toe landmarks and torque-limited PD. No new physics engine, no fictional
root motion, no body teleport/reset after settling. Not a browser server.

Example: python experiments/cad30/live_mujoco.py --source-dir /owner/CAD30/source
The source directory is private and must never be committed to GitHub.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

from hexlab.live_gait import LEGS, LiveGait, Twist


# Commands deliberately change in mid-cycle, not at period boundaries.
COMMANDS = (
    (0.0, Twist(vx=.012)),
    (3.10, Twist(vy=.010)),
    (6.20, Twist(wz=.15)),
    (9.30, Twist()),
)


def stage_at(t: float, commands=COMMANDS) -> int:
    return max(i for i, (start, _) in enumerate(commands) if t >= start)


def contact_loads(model, data, mujoco, geometry_ids: dict[str, int]) -> dict[str, float]:
    """Measured MuJoCo normal contact forces (N), not scheduled contacts."""
    ground = model.geom('floor').id
    result = {name: 0.0 for name in LEGS}
    reverse = {v: k for k, v in geometry_ids.items()}
    force = np.zeros(6)
    for ci in range(data.ncon):
        c = data.contact[ci]
        if c.geom1 == ground:
            other = int(c.geom2)
        elif c.geom2 == ground:
            other = int(c.geom1)
        else:
            continue
        if other in reverse:
            mujoco.mj_contactForce(model, data, ci, force)
            result[reverse[other]] += max(0.0, float(force[0]))
    return result


def run(source_dir: Path, *, scenario: str = 'nominal', gait: str = 'wave',
        seconds: float = 12.4, control_dt: float = .04,
        foot_lift_m: float = .007, max_stride_m: float = .022,
        speed_scale: float = 0.5) -> dict:
    import mujoco
    from hexlab.stand_v04 import static_pose, hold_under_gravity
    from hexlab.stand_geometry_v04 import load_legs, tip_world
    from hexlab.step_one_leg_v04 import forward_from_geometry, solve_waypoint

    if not (0 < speed_scale <= 1 and .004 <= foot_lift_m <= .030 and
            .010 <= max_stride_m <= .050):
        raise ValueError('Invalid experiment limits')
    commands = tuple((start, Twist(c.vx * speed_scale, c.vy * speed_scale,
                                    c.wz * speed_scale)) for start, c in COMMANDS)
    scene = source_dir / 'mjcf' / f'scene_local30_{scenario}.xml'
    if not scene.is_file():
        raise FileNotFoundError(f'Private verified CAD30 scene not available: {scene}')
    model = mujoco.MjModel.from_xml_path(str(scene))
    if model.nu != 20:
        raise ValueError('Expected CAD30 model with 20 actuators')
    timestep = float(model.opt.timestep)
    if abs(timestep - .002) > 1e-8:
        raise ValueError(f'Expected 2ms physics dt; got {timestep}')
    if abs(round(control_dt / timestep) * timestep - control_dt) > 1e-9:
        raise ValueError('Controller dt must be integer multiple of physical timestep')
    floor = model.geom('floor').id
    model.geom_friction[floor, 0] = 1.1
    for name in LEGS:
        model.geom_friction[model.geom(f'tibia{name}_collision_0').id, 0] = 1.1

    data = mujoco.MjData(model)
    legs = load_legs(model, data, mujoco)
    by_name = {leg['suffix']: leg for leg in legs}
    if set(by_name) != set(LEGS):
        raise ValueError('CAD names do not match planned six legs')
    _fit = static_pose(model, data, mujoco, legs, .145, .150)
    settle = hold_under_gravity(model, data, mujoco, data.qpos.copy(), 1.0)
    if not settle['stable_by_proxy_thresholds']:
        raise RuntimeError('Saved initial six-leg pose did not stand under gravity')
    mujoco.mj_forward(model, data)

    root = model.body('base_link_2').id
    start_root = np.asarray(data.xpos[root]).copy()
    R0 = np.asarray(data.xmat[root]).reshape(3, 3).copy()
    # A CAD-derived world forward axis projected into the settled body frame.
    front_xy = np.asarray(forward_from_geometry(legs), dtype=float)
    forward_world = np.array([*front_xy, 0.])
    forward_local = R0.T @ forward_world
    forward_local /= np.linalg.norm(forward_local)
    left_local = np.array([-forward_local[1], forward_local[0], 0.0])
    left_local /= np.linalg.norm(left_local)
    left_world = R0 @ left_local
    anchors_body = {}
    anchors_xy = {}
    for leg in legs:
        p = R0.T @ (tip_world(model, data, leg) - start_root)
        anchors_body[leg['suffix']] = p
        anchors_xy[leg['suffix']] = (float(p @ forward_local), float(p @ left_local))

    ctl = LiveGait(anchors_xy, gait=gait, period_s=2.4,
                   foot_lift_m=foot_lift_m, max_stride_m=max_stride_m)
    joint_ids = np.asarray(model.actuator_trnid[:, 0], dtype=int)
    qa = np.asarray(model.jnt_qposadr[joint_ids], dtype=int)
    da = np.asarray(model.jnt_dofadr[joint_ids], dtype=int)
    actuator_for_qa = {int(a): i for i, a in enumerate(qa)}
    if len(actuator_for_qa) != model.nu:
        raise ValueError('Actuators are not mapped one-to-one to joints')
    qtarget = data.qpos[qa].copy()
    bounds = np.asarray(model.actuator_ctrlrange).copy()
    reference = mujoco.MjData(model)
    geometry_ids = {n: model.geom(f'tibia{n}_collision_0').id for n in LEGS}
    frame = None
    trace = []
    sample_interval = round(.10 / timestep)
    control_interval = round(control_dt / timestep)
    command_interval = round(.08 / timestep)
    stage_number = -1
    max_ik_error_m = 0.0
    ik_rejections = 0
    saturation_steps = 0
    min_loaded_legs = 6
    min_height = float('inf')
    max_tilt_deg = 0.
    max_abs_torque = 0.
    elapsed_wall_start = time.perf_counter()
    physics_steps = 0
    for k in range(round(seconds / timestep)):
        sim_t = k * timestep
        stage = stage_at(sim_t, commands)
        if stage != stage_number:
            stage_number = stage
        if k % command_interval == 0 or k == 0 or abs(sim_t - commands[stage][0]) < timestep / 2:
            ctl.set_command(commands[stage][1])
        if k % control_interval == 0:
            frame = ctl.tick(control_dt)
            qtarget = data.qpos[qa].copy()
            reference.qpos[:] = data.qpos
            reference.qvel[:] = 0.0
            mujoco.mj_forward(model, reference)
            root_pos = np.asarray(data.xpos[root]).copy()
            root_rot = np.asarray(data.xmat[root]).reshape(3, 3).copy()
            for name in LEGS:
                dx, dy, dz = frame.foot_offsets[name]
                delta = dx * forward_local + dy * left_local + np.array([0., 0., dz])
                target = root_pos + root_rot @ (anchors_body[name] + delta)
                leg = by_name[name]
                seed = np.asarray(data.qpos[leg['qadrs']]).copy()
                try:
                    solved, err = solve_waypoint(model, reference, mujoco, leg, target, seed)
                except (ValueError, RuntimeError, FloatingPointError):
                    err = float('inf')
                    solved = seed
                max_ik_error_m = max(max_ik_error_m, float(err))
                if not np.isfinite(err) or err > .006:
                    ik_rejections += 1
                    continue
                for qindex, value in zip(leg['qadrs'], solved):
                    qtarget[actuator_for_qa[int(qindex)]] = value
        tau_unc = 7 * (qtarget - data.qpos[qa]) - .32 * data.qvel[da]
        if np.any(tau_unc > bounds[:, 1]) or np.any(tau_unc < bounds[:, 0]):
            saturation_steps += 1
        tau = np.clip(tau_unc, bounds[:, 0], bounds[:, 1])
        max_abs_torque = max(max_abs_torque, float(np.max(np.abs(tau))))
        data.ctrl[:] = tau
        mujoco.mj_step(model, data)
        physics_steps += 1
        if k % sample_interval == 0 or k == round(seconds / timestep) - 1:
            R = np.asarray(data.xmat[root]).reshape(3, 3)
            root_pos = np.asarray(data.xpos[root])
            forward_now = R @ forward_local
            yaw = math.atan2(float(np.cross(forward_world, forward_now)[2]),
                             float(forward_world @ forward_now))
            cos_tilt = float(np.clip((R @ (R0.T @ np.array([0., 0., 1.]))) @
                                     np.array([0., 0., 1.]), -1., 1.))
            tilt = math.degrees(math.acos(cos_tilt))
            forces = contact_loads(model, data, mujoco, geometry_ids)
            loaded = sum(fz >= .2 for fz in forces.values())
            min_loaded_legs = min(min_loaded_legs, loaded)
            min_height = min(min_height, float(root_pos[2]))
            max_tilt_deg = max(max_tilt_deg, tilt)
            displacement = root_pos - start_root
            trace.append({
                'time_s': round(float(sim_t + timestep), 3),
                'stage': stage, 'mode': frame.mode,
                'root_forward_m': round(float(displacement @ forward_world), 6),
                'root_left_m': round(float(displacement @ left_world), 6),
                'yaw_rad': round(yaw, 5), 'height_m': round(float(root_pos[2]), 5),
                'tilt_deg': round(tilt, 3), 'loaded_legs': loaded,
                'Fz_N': {n: round(fz, 3) for n, fz in forces.items()},
                'command': list(commands[stage][1].__dict__.values()),
            })
            if not np.all(np.isfinite(data.qpos)) or root_pos[2] < .065 or tilt > 50:
                break
    wall_s = time.perf_counter() - elapsed_wall_start
    checkpoints = {}
    for i, (start, command) in enumerate(commands):
        seg = [p for p in trace if p['stage'] == i]
        if seg:
            first = seg[0]
            last = seg[-1]
            checkpoints[str(i)] = {
                'command': command.__dict__,
                'observed_forward_m': round(last['root_forward_m'] - first['root_forward_m'], 6),
                'observed_left_m': round(last['root_left_m'] - first['root_left_m'], 6),
                'observed_yaw_rad': round(last['yaw_rad'] - first['yaw_rad'], 6),
                'min_loaded_legs': min(p['loaded_legs'] for p in seg),
            }
    # Predefined acceptance thresholds, not retrofitted to the measurements.
    gates = {
        'forward_gt_5mm': checkpoints.get('0', {}).get('observed_forward_m', 0) > .005,
        'left_gt_3mm': checkpoints.get('1', {}).get('observed_left_m', 0) > .003,
        'yaw_ccw_gt_0p03rad': checkpoints.get('2', {}).get('observed_yaw_rad', 0) > .03,
        'no_fall_height_gt_8cm': min_height > .08,
        'tilt_lt_18deg': max_tilt_deg < 18,
        'at_least_3_real_loaded_legs': min_loaded_legs >= 3,
        'no_ik_rejections': ik_rejections == 0,
        'torque_saturation_lt_1pct': saturation_steps / max(1, physics_steps) < .01,
        'full_duration_completed': physics_steps == round(seconds / timestep),
    }
    return {
        'status': 'PHYSICAL_PASS' if all(gates.values()) else 'PHYSICAL_FAIL',
        'note': 'Uncalibrated servo/mass/friction model; simulation is not hardware verification',
        'source': str(scene), 'physics': mujoco.__version__,
        'gait': gait, 'scenario': scenario, 'physics_dt_s': timestep,
        'speed_scale': speed_scale, 'foot_lift_m': foot_lift_m,
        'max_stride_m': max_stride_m,
        'controller_dt_s': control_dt, 'elapsed_wall_s': round(wall_s, 3),
        'duration_s': round(physics_steps * timestep, 3),
        'physics_steps': physics_steps,
        'max_ik_error_m': None if not np.isfinite(max_ik_error_m) else round(max_ik_error_m, 6),
        'ik_rejections': ik_rejections,
        'min_real_loaded_legs': min_loaded_legs,
        'max_tilt_deg': round(max_tilt_deg, 3),
        'min_height_m': round(min_height, 5),
        'max_abs_motor_torque_Nm': round(max_abs_torque, 5),
        'motor_saturation_steps': saturation_steps,
        'gates': gates, 'stages': checkpoints, 'trace': trace,
    }


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument('--source-dir', type=Path, default=Path('experiments/cad30/source'))
    p.add_argument('--scenario', choices=('nominal','lower','upper'), default='nominal')
    p.add_argument('--gait', choices=('wave','tripod'), default='wave')
    p.add_argument('--seconds', type=float, default=12.4)
    p.add_argument('--foot-lift', type=float, default=.007)
    p.add_argument('--max-stride', type=float, default=.022)
    p.add_argument('--speed-scale', type=float, default=0.5)
    p.add_argument('--output', type=Path, default=Path('results/cad30_live_v1.json'))
    args = p.parse_args()
    report = run(args.source_dir, scenario=args.scenario, gait=args.gait,
                 seconds=args.seconds, foot_lift_m=args.foot_lift,
                 max_stride_m=args.max_stride, speed_scale=args.speed_scale)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k:v for k,v in report.items() if k != 'trace'}, indent=2))


if __name__ == '__main__':
    main()
