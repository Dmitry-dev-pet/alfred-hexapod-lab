#!/usr/bin/env python3
"""Package eight previously measured EN01 traces, without executing physics.

Raw inputs stay outside Git. Geometry is read from each trace's source MJCF.
The owner authorised publication of existing diagnostic results on 2026-11-09.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

MODES = ('stand', 'crouch', 'jump', 'jump_fast')
COLUMNS = ['time_s', 'x_m', 'y_m', 'z_m', 'qw', 'qx', 'qy', 'qz', 'hip_left_rad', 'wheel_left_rad', 'hip_right_rad', 'wheel_right_rad', 'ground_contact_0_1', 'torso_contact_0_1', 'contact_normal_N', 'pitch_rad']
HASHES = {
    'v0': 'd73f680bb6384ca35b7d3b6cb684c5d2d7be26ac78c3858947c765c59ef38d4c',
    'v1': 'e9feb2b29572f522676f761685dacd732fab36c1145e9a58cb7faa1e761c1ca0',
}

def sha(raw):
    return hashlib.sha256(raw).hexdigest()

def vector(node, attr):
    return [float(x) for x in node.attrib[attr].split()]

def geometry(xml):
    root = ET.fromstring(xml)
    body = lambda name: root.find(f'.//body[@name="{name}"]')
    geom = lambda name: root.find(f'.//geom[@name="{name}"]')
    return {
        'torso_size': vector(geom('torso_geom'), 'size'),
        'left_hip': vector(body('left_leg'), 'pos'),
        'right_hip': vector(body('right_leg'), 'pos'),
        'left_wheel': vector(body('left_wheel'), 'pos'),
        'right_wheel': vector(body('right_wheel'), 'pos'),
        'wheel_radius': vector(geom('left_wheel_geom'), 'size')[0],
        'wheel_width': vector(geom('left_wheel_geom'), 'size')[1] * 2,
        'leg_radius': vector(geom('left_leg_geom'), 'size')[0],
    }

def export(v0, v1, out):
    checks = json.loads((v0 / 'CHECKSUMS_SHA256.json').read_text())
    for rel, digest in checks.items():
        if sha((v0 / rel).read_bytes()) != digest:
            raise ValueError('Original package checksum mismatch: ' + rel)
    receipts = []
    for version, base, model in [('v0', v0, 'model_v0.xml'), ('v1', v1, 'model_v1_cad_pose.xml')]:
        xml = (base / 'research/navbot_en01' / model).read_bytes()
        for mode in MODES:
            src = base / (f'results/navbot_en01/{mode}_verified.json' if version == 'v0' else f'results_v1_raw/{mode}_raw.json')
            raw = src.read_bytes()
            original = json.loads(raw)
            summary = original if version == 'v0' else original['summary']
            frames = original['frames']
            if summary['mode'] != mode or summary['mujoco_version'] != '3.14.0' or summary['physics_steps'] != 3000 or len(frames) != 300:
                raise ValueError('Unexpected trace provenance')
            if version == 'v1' and summary['model_sha256'] != sha(xml):
                raise ValueError('Model hash mismatch')
            replay = []
            for n, frame in enumerate(frames):
                for field, width in [('qpos', 11), ('qvel', 10), ('ctrl', 4)]:
                    seq = frame[field]
                    if len(seq) != width or not all(type(v) in (int, float) and math.isfinite(v) for v in seq):
                        raise ValueError('Invalid physical sample')
                if abs(frame['t'] - (.002 + n*.02)) > 1e-5:
                    raise ValueError('Invalid frame time')
                if abs(sum(v*v for v in frame['qpos'][3:7])-1) > .01:
                    raise ValueError('Invalid quaternion')
                for flag in ['ground_contact', 'torso_ground_contact']:
                    if type(frame[flag]) is not bool:
                        raise ValueError('Invalid contact')
                row = [frame['t'], *frame['qpos'], int(frame['ground_contact']), int(frame['torso_ground_contact']), frame['ground_normal_force_n'], frame['pitch_rad']]
                if not all(math.isfinite(v) for v in row):
                    raise ValueError('Nonfinite replay')
                replay.append(row)
            flight = summary.get('has_flight_ge_20ms') if version == 'v0' else summary['has_qualified_jump']
            landing = summary.get('stable_landing_2s_observed') if version == 'v0' else summary['stable_landing_2s']
            if flight is not False or landing is not False:
                raise ValueError('Unexpected positive jump claim; review evidence')
            status = 'failed' if mode.startswith('jump') else 'research'
            if (status == 'research') != (summary['torso_ground_steps'] == 0):
                raise ValueError('Verdict conflicts with physical contacts')
            doc = {
                'format': 'navbot-en01-mujoco-physical-replay-v1',
                'model_version': version,
                'model_label': 'v0: приближённая исходная модель' if version == 'v0' else 'v1: модель с исходной позой из CAD, механизм приближённый',
                'status': status, 'physics_engine': 'MuJoCo', 'physics_version': '3.14.0',
                'duration_s': 6, 'sample_stride': 16, 'frame_count': 300,
                'frame_columns': COLUMNS, 'frames': replay, 'geometry': geometry(xml),
                'source': {'trace_sha256': sha(raw), 'model_xml_sha256': sha(xml), 'physics_steps': 3000, 'upstream_commit': '3139f2fb32b38d10611feb905e16f8aa012f2f2e', 'research_commit': 'b492d4273a694b60f6d57136e6aa8295c35eb5e7', 'raw_trace_in_public_repo': False},
                'result': {'mode': mode, 'max_abs_x_m': summary['max_abs_x_m'], 'max_abs_pitch_deg': math.degrees(summary['max_abs_pitch_rad']) if version == 'v0' else summary['max_abs_pitch_deg'], 'torso_ground_steps': summary['torso_ground_steps'], 'qualified_flight': flight, 'stable_landing_2s': landing},
            }
            out.mkdir(parents=True, exist_ok=True)
            dest = out / f'{version}-{mode}.json'
            payload = (json.dumps(doc, ensure_ascii=False, allow_nan=False, separators=(',', ':')) + '\n').encode()
            if dest.exists() and dest.read_bytes() != payload:
                raise ValueError('Immutable replay already exists with different bytes')
            dest.write_bytes(payload)
            # Round-trip proof: every floating-base pose and all 4 joints are unchanged.
            restored = json.loads(dest.read_bytes())
            assert all(row[0] == frame['t'] and row[1:12] == frame['qpos'] for row, frame in zip(restored['frames'], frames))
            receipts.append({'file': dest.name, 'trace_sha256': sha(raw), 'replay_sha256': sha(payload), 'frames': 300, 'status': status})
    return receipts

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--v0', type=Path, required=True)
    p.add_argument('--v1', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    print(json.dumps(export(a.v0, a.v1, a.out), indent=2))
