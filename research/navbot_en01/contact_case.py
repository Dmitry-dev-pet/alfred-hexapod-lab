"""Run one declared v5 recipe, generating its pinned CAD-contact model first."""
import argparse
from pathlib import Path
from .contact_battery import CASES
from .contact_model import create
from .contact_probe import run

if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--case', choices=[c[0] for c in CASES], required=True)
    p.add_argument('--cad-mesh', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    _, scenario, period, dt = next(c for c in CASES if c[0] == a.case)
    model = a.out.parent.parent/('model-'+str(dt)+'.xml')
    model.parent.mkdir(parents=True, exist_ok=True)
    xml = create(a.cad_mesh, timestep=dt)
    if model.exists() and model.read_text() != xml:
        raise ValueError('Immutable model conflict')
    model.write_text(xml)
    run(a.out, loop_ms=period, jump=scenario == 'jump', scenario=scenario, model_path=model)
