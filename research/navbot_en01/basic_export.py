"""Preserve every measured basic-motion outcome, with independent FK/CAD audits."""
from __future__ import annotations
import argparse, hashlib, json, tempfile
from pathlib import Path
from .linkage_export import build
from .contact_mesh_audit import audit_record


def export(trace, out, *, model_path, mesh_path):
    raw = Path(trace).read_bytes()
    record = json.loads(raw)
    summary = record['summary']
    if summary.get('experiment') != 'basic_motion_v6':
        raise ValueError('Not a basic-motion trace')
    xml = Path(model_path).read_text()
    with tempfile.TemporaryDirectory() as td:
        temp = Path(td)/'replay.json'
        receipt = build(trace, temp, model_xml=xml)
        doc = json.loads(temp.read_bytes())
    audit = audit_record(record, mesh_path, xml)
    limits_ok = min(summary['hip_min_all_steps_rad']) >= 0 and max(summary['hip_max_all_steps_rad']) <= .65
    geometry_ok = limits_ok and audit['unexpected_intersection_frames'] == 0
    doc['basic_motion'] = {'version': 6, 'case': summary['case'], 'checks': summary['checks'],
                           'basic_pass': summary['basic_pass'], 'geometry_ok': geometry_ok,
                           'cad_surface_audit': audit, 'hardware_motion_claim_allowed': False}
    doc['status'] = 'research' if geometry_ok and summary['basic_pass'] else 'failed'
    doc['limitations'] = [
        'Frozen v5 CAD-contact model and actuator ceilings; mass/servo response remain assumed',
        'Balance plus differential yaw is a controller adaptation, not exact original firmware',
        'Operating guards are not calibrated factory stops',
        'Contact slip is force-weighted tangential velocity relative to the static floor',
        'Original surface audit covers saved body poses, excluding intended joint mates',
        'Only a surrogate simulation; no hardware movement or thermal-load validation']
    payload = json.dumps(doc, allow_nan=False, separators=(',', ':'))+'\n'
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    if out.exists() and out.read_text() != payload:
        raise ValueError('Immutable replay conflict')
    out.write_text(payload)
    return {**receipt, 'status': doc['status'], 'replay_sha256': hashlib.sha256(payload.encode()).hexdigest(),
            'geometry_ok': geometry_ok, 'cad_surface_audit': audit}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('trace', 'out', 'model', 'cad-mesh'):
        p.add_argument('--'+name, type=Path, required=True)
    a = p.parse_args()
    r = export(a.trace, a.out, model_path=a.model, mesh_path=a.cad_mesh)
    print(json.dumps({k:v for k,v in r.items() if k != 'cad_surface_audit'}, indent=2))
