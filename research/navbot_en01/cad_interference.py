"""Exact pinned-STEP intersection witness, not a complete joint-limit calibration."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

from .linkage import PINNED_BLOB, PINNED_BYTES, solve
from .linkage_model import load_evidence

MOVING = (62, 65, 28)
FIXED = (40, 41, 42, 43, 53, 56, 72)
ANGLES = (0., -.04, -.08, -.12, -.18, -.22, .1, .3, .5, .65)


def pinned_parts(path):
    raw = Path(path).read_bytes()
    blob = hashlib.sha1(b'blob '+str(len(raw)).encode()+b'\0'+raw).hexdigest()
    if len(raw) != PINNED_BYTES or blob != PINNED_BLOB:
        raise ValueError('Pinned original STEP mismatch')
    from .cad_parts import read_parts
    parts = read_parts(path)
    if len(parts) != 73:
        raise ValueError('Unexpected assembly part count')
    return parts


def intersection_volume(a, b):
    import cadquery as cq
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
    aa, bb = a.BoundingBox(), b.BoundingBox()
    if any(getattr(aa, d+'max') < getattr(bb, d+'min') or
           getattr(bb, d+'max') < getattr(aa, d+'min') for d in 'xyz'):
        return 0.
    return cq.Shape._bool_op(a, (a,), (b,), BRepAlgoAPI_Common(), parallel=False).Volume()


def posed_shape(parts, pid, q):
    points = load_evidence()['sides']['cad_z_positive']['points_xy_mm']
    solution = solve(points, -q)
    if pid in (62, 65):
        pivot, delta = points['B'], solution['calf_delta_rad']
        shift = [solution['B'][i]-pivot[i] for i in (0, 1)]
    elif pid == 28:
        pivot, delta, shift = points['D'], solution['rod_delta_rad'], [0., 0.]
    else:
        raise ValueError('Part outside declared witness subset')
    return parts[pid]['shape'].rotate((*pivot, 0), (*pivot, 1), math.degrees(delta)).translate((*shift, 0))


def witnesses(parts, q):
    hits = []
    for moving in MOVING:
        shape = posed_shape(parts, moving, q)
        for fixed in FIXED:
            volume = intersection_volume(shape, parts[fixed]['shape'])
            if volume > .01:
                hits.append({'parts': [moving, fixed], 'volume_mm3': volume})
    return {'q_rad': q, 'hits': hits}


def audit(step):
    parts = pinned_parts(step)
    return {'schema': 'en01-cad-interference-witness/v1',
            'source_step_git_blob': PINNED_BLOB,
            'moving_parts': list(MOVING), 'fixed_parts': list(FIXED),
            'part_names': {str(i): parts[i]['reference_name'] for i in (*MOVING, *FIXED)},
            'intersection_reporting_threshold_mm3': .01,
            'method': 'OpenCascade exact BRep common, analytical closed four-bar branch',
            'rows': [witnesses(parts, q) for q in ANGLES],
            'scope': 'Right-side witness subset only. Discrete poses do not establish a '
                     'collision-free continuous interval, encoder zero, mechanical stop, '
                     'or collision response. No safe joint range is asserted.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--step', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.step)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'poses': len(result['rows']), 'interfering_poses': sum(bool(r['hits']) for r in result['rows'])}))


if __name__ == '__main__':
    main()
