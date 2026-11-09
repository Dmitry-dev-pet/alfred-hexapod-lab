#!/usr/bin/env python3
"""EN01 radius sensitivity using genuine MuJoCo steps, NOT CAD / hardware parity.

Only the two wheel cylinder radii and root starting z change. A candidate local
STEP cylindrical radius of 18.5mm is not proof of effective rolling radius.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent))
import probe

BASE_MODEL = Path(__file__).with_name("model_v0.xml")
RADII_MM = (18.5, 21.0, 24.0)
MODES = ("stand", "crouch", "jump", "jump_fast")
BASELINE_RAW_SHA256 = {
    "stand": "a5d0cad75d4c4cf1e78c245ebfeda4dc23e9431addcec0741ecf8a8196cdf273",
    "crouch": "ac2f053b44353f0f15066fa9908f54afb6a5f3e73f432c1f550cfd82b5ab43cc",
    "jump": "608cfa5778bf96d4578b36ec65408e64e54dd07d72366ff5faf8b058137174d3",
    "jump_fast": "98920e20e4506a2f644288cc4c4083c7b264d38207700bf42fca2d571220b62c",
}

def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()

def variant_xml(base: str, radius_mm: float) -> str:
    if radius_mm <= 0 or radius_mm > 100 or base.count('pos="0 0 0.090"') != 1 or base.count('size="0.024 0.009"') != 2:
        raise ValueError("Unexpected input geometry/radius; audit before changing")
    if radius_mm == 24.0:
        return base  # exact 2026-11-08 baseline, no float-formatting differences
    z = .066 + radius_mm / 1000
    return (base.replace('pos="0 0 0.090"', f'pos="0 0 {z:.6f}"')
                .replace('size="0.024 0.009"', f'size="{radius_mm/1000:.6f} 0.009"'))

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', type=Path, default=Path('results/navbot_en01/radius_sensitivity_20261109'))
    args = ap.parse_args()
    import mujoco
    if mujoco.__version__ != '3.14.0':
        raise RuntimeError('Reproducible parity requires MuJoCo 3.14.0; not run')
    args.out.mkdir(parents=True, exist_ok=True)
    base = BASE_MODEL.read_text(encoding='utf8')
    old_model = probe.MODEL
    rows = []
    try:
        for radius_mm in RADII_MM:
            model_path = args.out / f'model_r{radius_mm:04.1f}.xml'
            model_path.write_text(variant_xml(base, radius_mm), encoding='utf8')
            probe.MODEL = model_path
            for mode in MODES:
                trace = args.out / f'{mode}_r{radius_mm:04.1f}.json'
                result = probe.run(mode, 6.0, trace)
                trace_sha = sha(trace)
                if radius_mm == 24.0 and trace_sha != BASELINE_RAW_SHA256[mode]:
                    raise AssertionError(f'{mode} baseline trace differs from pinned SHA')
                keys = ('mode', 'duration_s', 'physics_steps', 'max_abs_x_m',
                        'max_abs_pitch_rad', 'torso_ground_steps', 'has_flight_ge_20ms',
                        'stable_landing_2s_observed', 'max_floor_penetration_m')
                row = {k: result[k] for k in keys}
                row.update(radius_mm=radius_mm, trace_sha256=trace_sha,
                           model_sha256=sha(model_path))
                rows.append(row)
                print(f'{radius_mm:.1f} mm {mode}: max_x={row["max_abs_x_m"]:.4f} m '
                      f'torso_ground={row["torso_ground_steps"]} '
                      f'qualified_flight={row["has_flight_ge_20ms"]}', flush=True)
    finally:
        probe.MODEL = old_model
    receipt = {
        'schema': 'navbot-en01-radius-sensitivity-v1',
        'classification': 'EXPLORATORY_APPROXIMATE_ONLY_NOT_CAD_OR_HARDWARE_VALIDATED',
        'upstream_commit': '3139f2fb32b38d10611feb905e16f8aa012f2f2e',
        'step_blob_sha1': '9156683a0bb694c84dc541a3e26b5f6b08db1a41',
        'mujoco_version': mujoco.__version__,
        'base_model_sha256': sha(BASE_MODEL),
        'note': ('18.3/18.5mm are local STEP tire surface radii, not verified assembled rolling radius; '
                 'no controller, torque, motor, mass, or inertia changes. Root z adjusted for ground contact.'),
        'results': rows,
    }
    (args.out / 'receipt.json').write_text(json.dumps(receipt, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print('RECEIPT', args.out / 'receipt.json', flush=True)

if __name__ == '__main__':
    main()
