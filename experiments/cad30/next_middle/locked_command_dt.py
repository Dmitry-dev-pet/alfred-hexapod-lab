#!/usr/bin/env python3
"""MuJoCo CAD30: compare integration substeps under *identical recorded* 2ms motor goals.

Inputs are private CAD30 meshes and PR49's baseline/feedback raw MR 2ms traces.
No hardware commands; never edit qpos/qvel after saved checkpoint initialization.
This is an OPEN-LOOP recorded-command ABLATION, not a live feedback-controller test.
"""
from __future__ import annotations
import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path

CONTROL_DT = 0.002
LEGS = ('ml', 'mr', 'bl', 'br', 'fl', 'fr')
STANCE = ('ml', 'bl', 'br')
WALK = ('ml', 'mr', 'bl', 'br')


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def read_frames(path):
    path = Path(path)
    opener = gzip.open if path.suffix == '.gz' else open
    with opener(path, 'rt', encoding='utf-8') as stream:
        frames = json.load(stream)['frames']
    if not frames:
        raise ValueError('Empty 2ms source trace')
    for i, frame in enumerate(frames):
        if len(frame.get('motor_goal', ())) != 20:
            raise ValueError(f'Frame {i}: expected 20 motor goals')
        if i and not math.isclose(frame['time_s'] - frames[i - 1]['time_s'],
                                  CONTROL_DT, rel_tol=0, abs_tol=1e-7):
            raise ValueError(f'Frame {i}: not a uniform 2ms physical trace')
    return frames


def triangle_margin(contacts, com):
    """Signed min planar distance to each side of three actual stance contacts (mm)."""
    import numpy as np
    def cross2(a, b):
        return float(a[0] * b[1] - a[1] * b[0])
    if any(leg not in contacts for leg in STANCE):
        return float('-inf')
    pts = np.asarray([contacts[leg] for leg in STANCE], float)
    center = np.asarray(com[:2], float)
    area2 = cross2(pts[1] - pts[0], pts[2] - pts[0])
    if abs(area2) < 1e-10:
        return float('-inf')
    sign = 1 if area2 > 0 else -1
    signed = [sign * cross2(pts[(i + 1) % 3] - pts[i],
                                    center - pts[i]) /
              float(np.linalg.norm(pts[(i + 1) % 3] - pts[i]))
              for i in range(3)]
    return min(signed) * 1000


def physical_contacts(mj, model, data, floor, by_geom):
    """Independently recompute real Fz via MuJoCo contact wrench, never interpolate."""
    import numpy as np
    fz = {leg: 0.0 for leg in LEGS}
    points = {leg: [] for leg in LEGS}
    pair_counts = {leg: 0 for leg in LEGS}
    force = np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        if int(c.geom1) == floor:
            other = int(c.geom2)
        elif int(c.geom2) == floor:
            other = int(c.geom1)
        else:
            continue
        leg = by_geom.get(other)
        if leg is None:
            continue
        pair_counts[leg] += 1
        mj.mj_contactForce(model, data, i, force)
        world = c.frame.reshape(3, 3).T @ force[:3]
        fz[leg] += max(0.0, float(world[2]))
        points[leg].append(np.asarray(c.pos[:2], dtype=float).copy())
    xy = {leg: np.mean(value, axis=0) for leg, value in points.items() if value}
    return fz, xy, pair_counts


def run(cad_root, trace, checkpoint, substeps, mu, out):
    import numpy as np
    import mujoco as mj
    import sys
    if substeps not in (1, 2, 4):
        raise ValueError('Substeps must be 1, 2, or 4')
    dt = CONTROL_DT / substeps
    trace = Path(trace); checkpoint = Path(checkpoint)
    cad_root = Path(cad_root)
    frames = read_frames(trace)
    state = json.loads(checkpoint.read_text())
    xml = cad_root / 'source/mjcf/scene_local30_nominal.xml'
    sys.path.insert(0, str(cad_root))
    from hexlab.stand_geometry_v04 import geom_mesh_world_vertices
    model = mj.MjModel.from_xml_path(str(xml))
    model.opt.timestep = dt
    data = mj.MjData(model)
    data.qpos[:] = state['qpos']; data.qvel[:] = state['qvel']
    floor = int(model.geom('floor').id)
    by_geom = {int(model.geom(f'tibia{leg}_collision_0').id): leg for leg in LEGS}
    for geom in (*by_geom, floor):
        model.geom_friction[geom, 0] = mu
    mj.mj_forward(model, data)
    actuator_joint = np.asarray(model.actuator_trnid[:, 0], int)
    qa = np.asarray(model.jnt_qposadr[actuator_joint], int)
    va = np.asarray(model.jnt_dofadr[actuator_joint], int)
    if len(qa) != 20:
        raise RuntimeError('Unexpected CAD30 actuator layout')
    limits = model.actuator_ctrlrange
    mass = model.body_mass
    total_mass = mass.sum()
    error_max = 0.0; error_rms_sum = 0.0
    fz_error_max = 0.0; bad_substep = 0; bad_tick = 0; bad_streak = 0; streak_max = 0
    low_by_leg = {leg: 0 for leg in STANCE}
    min_fz = float('inf'); min_margin = float('inf')
    missing_pairs = 0; first_bad = None; failures = []
    max_raw_tau = 0.; max_q = 0.; end_four_count = 0; end_four_total = 0
    mesh_min_z_on_bad = []
    trace_rows = []
    for index, frame in enumerate(frames):
        goal = np.asarray(frame['motor_goal'], float)
        # 500Hz position PD is IDENTICAL for all comparisons. Hold this torque
        # across 1/2/4 actual MuJoCo steps; compare numerical integration only.
        raw = 7 * (goal - data.qpos[qa]) - .32 * data.qvel[va]
        data.ctrl[:] = np.clip(raw, limits[:, 0], limits[:, 1])
        max_raw_tau = max(max_raw_tau, float(np.max(np.abs(raw[:18]))))
        low_this_tick = False
        for sub in range(substeps):
            mj.mj_step(model, data)
            fz, xy, pairs = physical_contacts(mj, model, data, floor, by_geom)
            phase = frame['phase']
            if phase.startswith('swing_mr'):
                margin = triangle_margin(xy, (np.asarray(data.xipos) * mass[:, None]).sum(axis=0) / total_mass)
                min_fz = min(min_fz, *(fz[leg] for leg in STANCE))
                min_margin = min(min_margin, margin)
                low = [leg for leg in STANCE if fz[leg] < .2]
                bad = bool(low) or margin < 10
                if low:
                    low_this_tick = True
                    bad_substep += 1
                    bad_streak += 1; streak_max = max(streak_max, bad_streak)
                    for leg in low:
                        low_by_leg[leg] += 1
                    if any(pairs[leg] == 0 for leg in low):
                        missing_pairs += 1
                    if first_bad is None:
                        first_bad = {'control_frame': index, 'substep': sub,
                                     'phase': phase, 'Fz_N': fz, 'margin_mm': margin if math.isfinite(margin) else None}
                    if len(mesh_min_z_on_bad) < 80:
                        g = next(g for g, l in by_geom.items() if l == 'ml')
                        mesh_min_z_on_bad.append(float(geom_mesh_world_vertices(model, data, g)[:, 2].min()))
                else:
                    bad_streak = 0
                if bad and len(failures) < 10:
                    failures.append({'frame': index, 'substep': sub, 'reason': 'low_contact_or_margin',
                                     'low_stance_legs': low, 'margin_mm': margin if math.isfinite(margin) else None})
            if phase == 'final_hold':
                end_four_total += 1
                end_four_count += int(all(fz[leg] >= .2 for leg in WALK))
            max_q = max(max_q, float(np.max(np.abs(data.qpos[qa[:18]]))))
            if not np.isfinite(data.qpos).all():
                raise RuntimeError(f'Nonfinite qpos at frame {index} substep {sub}')
        if low_this_tick:
            bad_tick += 1
        err = float(np.max(np.abs(data.qpos - np.asarray(frame['qpos'], float))))
        error_max = max(error_max, err)
        error_rms_sum += err * err
        fz_error_max = max(fz_error_max, *(abs(fz[leg] - frame['Fz_N'][leg]) for leg in WALK))
        # Retain 2ms-aligned state even when simulation dt is faster.
        if index % 10 == 0 or low_this_tick:
            trace_rows.append({'frame': index, 'phase': frame['phase'],
                               't_relative_s': (index + 1) * CONTROL_DT,
                               'Fz_N': {leg: fz[leg] for leg in WALK},
                               'qpos_max_error_to_reference': err,
                               'min_stance_Fz_N': min(fz[leg] for leg in STANCE)})
    baseline_reproduced = substeps != 1 or (error_max < 1e-4 and fz_error_max < 0.25)
    result = {
        'status': ('BASELINE_REPRODUCED' if baseline_reproduced else 'BASELINE_REPRODUCTION_FAILED')
                  if substeps == 1 else 'PHYSICS_RUN_REQUIRES_BASELINE_FIDELITY_CHECK',
        'claim': 'locked 500Hz command physical replay; NOT actual 0.5ms online feedback',
        'input_trace_sha256': sha256(trace), 'checkpoint_sha256': sha256(checkpoint),
        'model': 'original private CAD30 source/mjcf/scene_local30_nominal.xml',
        'mu': mu, 'controller_tick_s': CONTROL_DT, 'integrator_dt_s': dt,
        'substeps': substeps, 'control_frames': len(frames),
        'physics_steps': len(frames) * substeps,
        'qpos_max_error_to_original_2ms': error_max,
        'qpos_frame_max_error_rms_to_original_2ms': math.sqrt(error_rms_sum / len(frames)),
        'Fz_max_error_to_original_2ms_N': fz_error_max,
        'stance_low_Fz_substeps': bad_substep, 'stance_low_Fz_2ms_bins': bad_tick,
        'stance_low_Fz_by_leg': low_by_leg, 'longest_consecutive_low_Fz_steps': streak_max,
        'minimum_stance_Fz_N': min_fz if math.isfinite(min_fz) else None,
        'minimum_stance_triangle_margin_mm': min_margin if math.isfinite(min_margin) else None,
        'low_Fz_frames_with_absent_floor_pair': missing_pairs,
        'first_low_Fz': first_bad, 'ML_mesh_min_z_on_low_Fz_m': mesh_min_z_on_bad,
        'terminal_four_loaded_fraction': end_four_count / max(1, end_four_total),
        'max_raw_leg_torque_Nm': max_raw_tau, 'max_leg_joint_rad': max_q,
        'strict_support_pass': bad_substep == 0 and min_margin >= 10,
        'hardware_validated': False, 'full_gait': False, 'net_forward_gait_proven': False,
        'limitations': [
            'Replay uses recorded effective motor goals and DOES NOT recalculate Fz feedback at different dt.',
            'A 2ms fidelity check must PASS before treating 1/0.5ms results as integration ablations.',
            'The checkpoint contains qpos/qvel/motor goals, not MuJoCo qacc_warmstart; numerical drift may result.',
            'Substep guard examines every physical step and never filters short contact losses.'
        ]
    }
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    with gzip.open(out.with_suffix('.trace.json.gz'), 'wt', encoding='utf-8') as stream:
        json.dump({'schema': 'cad30-locked-command-dt/1', 'frames': trace_rows}, stream)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cad-root', required=True, type=Path)
    parser.add_argument('--trace', required=True, type=Path)
    parser.add_argument('--checkpoint', required=True, type=Path)
    parser.add_argument('--substeps', type=int, choices=(1, 2, 4), required=True)
    parser.add_argument('--mu', type=float, default=1.1)
    parser.add_argument('--out', required=True, type=Path)
    args = parser.parse_args()
    result = run(args.cad_root, args.trace, args.checkpoint, args.substeps, args.mu, args.out)
    print(json.dumps({k: v for k, v in result.items() if k not in ('ML_mesh_min_z_on_low_Fz_m',)},
                     ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()