#!/usr/bin/env python3
"""Rerun the positive step case at half timestep as a numerical sensitivity check."""
import argparse
import json
from pathlib import Path

import baseline as b

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--assets',type=Path,required=True)
    ap.add_argument('--trial',type=Path,required=True);a=ap.parse_args()
    b.DT=.0005
    out=a.trial/'convergence';out.mkdir(exist_ok=True)
    manifest,model,path,_,_=b.build(a.assets,out,.01)
    summary=b.run_case(manifest,model,path,out,'walk-step-10mm-half-dt',True,24)
    receipt=json.loads((a.trial/'receipt.json').read_text())
    reference=next(c for c in receipt['cases'] if c['case']=='walk-step-10mm')
    difference=abs(summary['metrics']['forward_after_settle_m']-reference['metrics']['forward_after_settle_m'])
    proof={'schema':'hexarth-timestep-sensitivity/v1','coarse_dt_s':.001,'fine_dt_s':b.DT,
           'case':'walk-step-10mm','fine_result':summary,'forward_difference_m':difference,
           'same_whole_step_verdict':summary['whole_robot_on_step']==reference['whole_robot_on_step']}
    assert proof['same_whole_step_verdict'] and difference<.005,proof
    (out/'receipt.json').write_text(json.dumps(proof,indent=2)+'\n')
    print(json.dumps(proof))
