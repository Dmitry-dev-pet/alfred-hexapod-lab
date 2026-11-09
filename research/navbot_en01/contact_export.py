"""Verify independent FK and original CAD surfaces before emitting a v5 replay."""
from __future__ import annotations
import argparse, hashlib, json, tempfile
from pathlib import Path
from .linkage_export import build
from .contact_mesh_audit import audit_record


def export(trace, out, *, model_path, mesh_path):
    raw = Path(trace).read_bytes()
    record = json.loads(raw)
    summary = record['summary']
    if summary.get('experiment') != 'cad_contact_v5':
        raise ValueError('Not a contact-repair trace')
    xml = Path(model_path).read_text()
    with tempfile.TemporaryDirectory() as td:
        temp = Path(td)/'replay.json'
        receipt = build(trace, temp, model_xml=xml)
        doc = json.loads(temp.read_bytes())
    audit = audit_record(record, mesh_path, xml)
    limits_ok = (min(summary['hip_min_all_steps_rad']) >= 0 and
                 max(summary['hip_max_all_steps_rad']) <= .65)
    if not limits_ok or audit['unexpected_intersection_frames']:
        raise ValueError('Contact-repair recording failed operating-range/CAD intersection validation')
    doc['contact_repair'] = {'version': 5, 'operating_range_rad': [0, .65],
                             'all_step_hip_limits_ok': limits_ok,
                             'cad_surface_audit': audit,
                             'hardware_jump_claim_allowed': False}
    doc['source_command_experiment'] = {
        'loop_period_ms_ASSUMED': summary['source_loop_period_ms_ASSUMED'],
        'cad_anchor_q_rad_ASSUMED': summary['cad_anchor_q_at_height38_rad_ASSUMED']}
    doc['status'] = 'research' if summary['mode'] != 'jump' or summary['jump_pass'] else 'failed'
    doc['limitations'] = [
        'Original CAD visualization; 28 convex outer envelopes provide conservative self contact',
        'Operating guard 0..0.65 rad with 0.01 rad margin is not a measured factory stop',
        'Servo zero, mass/inertia, motor response, floor compliance and springs remain uncalibrated',
        'Independent surface intersection audit covers saved poses; intended joint mates excluded',
        'Numerical simulation result only; no real-robot jump validation']
    payload = json.dumps(doc, allow_nan=False, separators=(',', ':'))+'\n'
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    if out.exists() and out.read_text() != payload:
        raise ValueError('Immutable replay conflict')
    out.write_text(payload)
    return {**receipt, 'status': doc['status'], 'replay_sha256': hashlib.sha256(payload.encode()).hexdigest(),
            'cad_surface_audit': audit, 'all_step_hip_limits_ok': limits_ok}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('trace', 'out', 'model', 'cad-mesh'):
        p.add_argument('--'+name, type=Path, required=True)
    a = p.parse_args()
    r = export(a.trace, a.out, model_path=a.model, mesh_path=a.cad_mesh)
    print(json.dumps({k: v for k, v in r.items() if k != 'cad_surface_audit'}, indent=2))


if __name__ == '__main__':
    main()
