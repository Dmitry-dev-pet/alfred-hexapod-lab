"""Generate the frozen v5 model and run one predefined basic-motion recipe."""
import argparse
from pathlib import Path
from .basic_commands import CASES
from .basic_probe import run
from .contact_model import create

if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--case', choices=CASES, required=True)
    p.add_argument('--cad-mesh', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    model = a.out.parent.parent/'model.xml'
    model.parent.mkdir(parents=True, exist_ok=True)
    xml = create(a.cad_mesh)
    if model.exists() and model.read_text() != xml:
        raise ValueError('Immutable model conflict')
    model.write_text(xml)
    run(a.out, case=a.case, model_path=model)
