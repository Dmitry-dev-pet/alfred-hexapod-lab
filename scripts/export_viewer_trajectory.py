"""Export a *recorded* CAD30 MuJoCo trajectory to a public-safe Three.js replay.

The output contains physical positions, leg joint-anchor locations, a geometric
foot-tip landmark and observed contact flags. It contains no STL/MJCF meshes.
MuJoCo runs offline on the local machine; the public viewer NEVER simulates it.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

CODES = ('ml', 'bl', 'br', 'mr', 'fr', 'fl')
FORMAT = 'fnk0052-physical-points-v1'


def export_trajectory(source: Path, model_xml: Path, output: Path) -> dict:
    import mujoco
    from hexlab.stand_geometry_v04 import load_legs, tip_world
    # Ensure the caller explicitly provides the model used for the simulation.
    report = json.loads(source.read_text(encoding='utf-8'))
    if not report.get('qpos_samples') or not report.get('trace'):
        raise ValueError('Input is missing recorded physical qpos/trace; do not fabricate frames')
    if report.get('gait') != 'tripod' or report.get('scenario') != 'nominal':
        raise ValueError('This public demo is locked to nominal CAD30 tripod gait')
    if not report.get('finished_full_duration') or report.get('fall_or_nonfinite'):
        raise ValueError('Only complete, unfailing physical runs qualify for the 60s demo')
    if float(report['duration_s']) < 59.9:
        raise ValueError('Not a full 60-second trajectory')
    model = mujoco.MjModel.from_xml_path(str(model_xml))
    data = mujoco.MjData(model)
    legs = load_legs(model, data, mujoco)
    assert [leg['suffix'] for leg in legs] == list(CODES)
    base_id = model.body('base_link_2').id
    snapshots = report['qpos_samples']
    trace = report['trace']
    if len(snapshots) < 1000:
        raise ValueError('Too few samples for an interactive long demonstration')
    previous_t = -float('inf')
    frames = []
    trace_i = 0
    for state in snapshots:
        t = float(state['t'])
        if t <= previous_t:
            raise ValueError('Nonmonotonic recorded timeline')
        previous_t = t
        qpos = np.asarray(state['qpos'], dtype=float)
        if qpos.size != model.nq or not np.all(np.isfinite(qpos)):
            raise ValueError('Invalid recorded model qpos')
        data.qpos[:] = qpos
        data.qvel[:] = 0
        mujoco.mj_forward(model, data)
        # Source 50 Hz physical trace is preferred to contacts recomputed with zero velocity.
        while trace_i + 1 < len(trace) and abs(float(trace[trace_i + 1]['t']) - t) <= abs(float(trace[trace_i]['t']) - t):
            trace_i += 1
        actual_contacts = set(trace[trace_i]['contacts'])
        mask = sum((1 << i) for i, code in enumerate(CODES) if code in actual_contacts)
        root = [*data.xpos[base_id].tolist(), *data.xquat[base_id].tolist()]
        joints = []
        for leg in legs:
            for jid in leg['jids']:
                joints.extend(data.xanchor[jid].tolist())
            joints.extend(tip_world(model, data, leg).tolist())
        # Flattened [root:7, joint-anchor+tip:72, contact bitmask:1].
        frames.append([round(float(v), 5) for v in (root + joints)] + [mask])
    interval = (float(snapshots[-1]['t']) - float(snapshots[0]['t'])) / (len(snapshots) - 1)
    forward = np.array([1., 0.])
    # The measured progress axis in run receipts is along the CAD front pair.
    hips = {leg['suffix']:leg['hip_xyz'] for leg in legs}
    d = (hips['fr'][:2] + hips['fl'][:2] - hips['br'][:2] - hips['bl'][:2]) / 2
    if np.linalg.norm(d) > 1e-8:
        forward = d / np.linalg.norm(d)
    payload = {
        'format': FORMAT,
        'source': 'Recorded 60-second free-body MuJoCo 3.14.0 CAD30 nominal tripod trial',
        'visualization': 'procedural linkage proxy — no original CAD mesh redistributed',
        'units': 'metres', 'coordinate_system': 'MuJoCo Z-up',
        'duration_s': round(float(report['duration_s']), 3),
        'sample_interval_s': round(interval, 6),
        'sample_count': len(frames),
        'stride': 80,
        'layout': 'base position xyz + quaternion wxyz (7), 6 x [hip/coxa, femur, tibia, tip] xyz (72), contact mask (1)',
        'leg_order': list(CODES),
        'forward_xy': [round(float(v), 6) for v in forward],
        'summary': {
            'advance_m': report['advance_m'], 'max_tilt_deg': report['max_tilt_deg'],
            'min_grounded_legs': report['min_grounded_legs'],
            'max_abs_torque_nm': report['max_abs_torque_nm'],
            'saturation_fraction': report['saturation_fraction'],
            'estimated_mass_kg': report['robot_mass_kg'],
            'fall_or_nonfinite': report['fall_or_nonfinite'],
        },
        'limitations': ['Physical parameters assumed, not measured on real FNK0052',
                        'Foot tip is a CAD mesh landmark, not an actual rubber tip',
                        'Public model is procedural geometry, not original STL',
                        'Contact flags are sampled from 50Hz recorded MuJoCo trace'],
        'frames': frames,
    }
    assert all(len(f) == payload['stride'] for f in frames)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, ensure_ascii=False, separators=(',', ':')) + '\n', encoding='utf-8')
    return payload


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    result = export_trajectory(args.source, args.model, args.out)
    print(json.dumps({k:v for k,v in result.items() if k != 'frames'}, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
