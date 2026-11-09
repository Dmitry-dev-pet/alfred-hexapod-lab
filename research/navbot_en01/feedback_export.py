"""Rebuild the declared sensitivity profile before exporting measured replay."""
import argparse
import json
from pathlib import Path
from .feedback_model import make_model
from .linkage_export import build


def export(trace,out):
    s=json.loads(Path(trace).read_text())['summary']
    return build(trace,out,model_xml=make_model(s['model_profile'],timestep=s['timestep_s']))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--trace',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();print(json.dumps(export(a.trace,a.out),indent=2))
