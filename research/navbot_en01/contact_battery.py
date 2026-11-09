"""Freeze eight contact-repair cases plus one timestep-refinement comparison."""
import argparse, hashlib, json
from pathlib import Path
from .contact_model import create
from .contact_probe import run
from .contact_export import export

CASES = [(s, s, 5, .00025) for s in ('stand', 'crouch', 'forced-fold', 'forced-extend')]
CASES += [('jump-p'+str(ms), 'jump', ms, .00025) for ms in (1, 2, 5, 10)]
CASES += [('jump-p5-halfdt', 'jump', 5, .000125)]


def battery(out, mesh):
    out, mesh = Path(out).resolve(), Path(mesh).resolve()
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    for name, scenario, period, dt in CASES:
        model = out/('model-'+str(dt)+'.xml')
        xml = create(mesh, timestep=dt)
        if model.exists() and model.read_text() != xml:
            raise ValueError('Immutable model conflict')
        model.write_text(xml)
        trace, replay = out/'raw'/(name+'_raw.json'), out/'replay'/(name+'.json')
        summary = run(trace, loop_ms=period, jump=scenario == 'jump', scenario=scenario, model_path=model)
        receipt = export(trace, replay, model_path=model, mesh_path=mesh)
        rows.append({'name': name, 'summary': summary, 'export': receipt})
        print(name, 'CAD PASS', 'jump_pass', summary['jump_pass'], flush=True)
    (out/'RESULTS.json').write_text(json.dumps(rows, indent=2)+'\n')
    manifest = {str(p.relative_to(out)): {'bytes': p.stat().st_size,
                'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}
                for p in sorted(out.rglob('*')) if p.is_file() and p.name != 'MANIFEST.json'}
    (out/'MANIFEST.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return rows


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--cad-mesh', type=Path, required=True)
    a = p.parse_args()
    battery(a.out, a.cad_mesh)
