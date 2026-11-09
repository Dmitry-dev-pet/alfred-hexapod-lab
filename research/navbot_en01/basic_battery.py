"""Run all twelve predefined baseline-motion cases on one unchanged v5 model."""
import argparse, hashlib, json
from pathlib import Path
from .contact_model import create
from .basic_commands import CASES
from .basic_probe import run
from .basic_export import export


def battery(out, mesh):
    out, mesh = Path(out).resolve(), Path(mesh).resolve()
    out.mkdir(parents=True, exist_ok=True)
    model = out/'model.xml'
    xml = create(mesh)
    # Exact same model bytes as the final v5 battery, not just similar parameters.
    if hashlib.sha256(xml.encode()).hexdigest() != '04f24a31a9c1f72e6f8599c266876ec6bdf17f2819ed79f510d1c145c82663f3':
        raise ValueError('Frozen v5 model changed')
    if model.exists() and model.read_text() != xml:
        raise ValueError('Immutable model conflict')
    model.write_text(xml)
    rows = []
    for case in CASES:
        trace, replay = out/'raw'/(case+'_raw.json'), out/'replay'/(case+'.json')
        summary = run(trace, case=case, model_path=model)
        receipt = export(trace, replay, model_path=model, mesh_path=mesh)
        rows.append({'case':case, 'summary':summary, 'export':receipt})
        print(case, 'PASS' if summary['basic_pass'] and receipt['geometry_ok'] else 'FAIL',
              [k for k,v in summary['checks'].items() if not v], flush=True)
    (out/'RESULTS.json').write_text(json.dumps(rows, indent=2)+'\n')
    manifest = {str(p.relative_to(out)):{'bytes':p.stat().st_size, 'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
                for p in sorted(out.rglob('*')) if p.is_file() and p.name != 'MANIFEST.json'}
    (out/'MANIFEST.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return rows


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--cad-mesh', type=Path, required=True)
    a = p.parse_args()
    battery(a.out, a.cad_mesh)
