"""Audit frozen EN01 traces; numerical flight is not mechanical validation.

This deliberately does not clip a replay or change the frozen v2-v4 model.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from .linkage_model import create


def audit_trace(record, model_xml=None):
    import mujoco
    import numpy as np

    xml = create() if model_xml is None else model_xml
    digest = hashlib.sha256(xml.encode()).hexdigest()
    if record['summary']['model_sha256'] != digest:
        raise ValueError('Trace model hash mismatch')
    model = mujoco.MjModel.from_xml_string(xml)
    frames = record['frames']
    if not frames:
        raise ValueError('Empty trace')
    q = np.asarray([f['qpos'] for f in frames], dtype=float)
    times = np.asarray([f['t'] for f in frames], dtype=float)
    if q.shape != (len(frames), model.nq) or not np.isfinite(q).all():
        raise ValueError('Invalid recorded joint positions')
    if not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError('Invalid recorded times')
    joints = []
    violations = []
    for jid in range(model.njnt):
        if model.jnt_type[jid] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
        values = q[:, model.jnt_qposadr[jid]]
        row = {'name': name, 'min_rad': float(values.min()),
               'max_rad': float(values.max()), 'limited': bool(model.jnt_limited[jid])}
        if row['limited']:
            lo, hi = model.jnt_range[jid]
            excess = np.maximum(np.maximum(lo-values, values-hi), 0.)
            idx = np.flatnonzero(excess > 1e-6)
            row.update(declared_range_rad=[float(lo), float(hi)],
                       max_limit_excursion_rad=float(excess.max()),
                       violating_saved_frames=len(idx),
                       first_violation_s=float(times[idx[0]]) if len(idx) else None)
            if len(idx):
                violations.append(name)
        joints.append(row)
    robot = [i for i in range(model.ngeom) if model.geom_bodyid[i] != 0]
    # This audits filter masks only. Adjacent-body filtering and geometry shape
    # quality are separate checks, so a mask-enabled pair is not validation.
    enabled = sum(bool((model.geom_contype[a] & model.geom_conaffinity[b]) or
                       (model.geom_contype[b] & model.geom_conaffinity[a]))
                  for ai, a in enumerate(robot) for b in robot[ai+1:])
    reasons = ['Mechanical stops and the absolute servo/CAD zero are uncalibrated',
               'Solid CAD interference and collision response are not validated']
    if enabled == 0:
        reasons.append('All robot/robot collision pairs are disabled by filter masks')
    if violations:
        reasons.append('Recorded hinge positions exceed the declared diagnostic limits')
    return {'schema': 'en01-model-validity-audit/v1',
            'model_sha256': digest,
            'status': 'INVALID_FOR_HARDWARE_INFERENCE',
            'physical_jump_claim_allowed': False,
            'reasons': reasons, 'joints': joints,
            'mask_enabled_robot_pairs': enabled,
            'scope': 'Saved-frame audit; unsaved steps may contain larger excursions. '
                     'This does not certify absence of self-intersection.'}


def require_physical_claim(report):
    if report.get('physical_jump_claim_allowed') is not True:
        raise ValueError('Physical jump claim blocked: model validity is not established')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    raw = args.trace.read_bytes()
    result = audit_trace(json.loads(raw))
    result['trace_sha256'] = hashlib.sha256(raw).hexdigest()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(result['status'])


if __name__ == '__main__':
    main()
