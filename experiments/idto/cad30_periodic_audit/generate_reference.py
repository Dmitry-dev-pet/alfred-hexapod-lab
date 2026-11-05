"""Build a whole four-swing, numerically optimized *soft reference* test for IDTO.

This generator does NOT prove or enforce a physical gait: it only constructs
an externally specified, periodic nominal trajectory and a pinned IDTO YAML.
Genuine hard contact/actuator constraints and MuJoCo verification are G1/G2
success gates, not outputs of this tool.
"""
from __future__ import annotations
import argparse
import csv
import json
import math
from pathlib import Path
import zipfile
import numpy as np

LEG_ORDER = ('ml','mr','bl','br')

def mujoco_to_drake(q):
    if len(q) != 27:
        raise ValueError(f'expected nq=27, got {len(q)}')
    q = np.asarray(q, dtype=float)
    # Verified G0 parser: MuJoCo [xyz, wxyz, 20 joints], Drake [wxyz, xyz, 20 joints].
    return np.r_[q[3:7], q[:3], q[7:]]

def sample_nodes(nodes, first=106, last=217, steps=28):
    if steps < 8:
        raise ValueError('at least 8 steps to represent 4 distinct swing phases')
    # Use deterministic monotone indices, keep both endpoints and at least one
    # sample in each actual 13-node airborne swing phase.
    indices = np.rint(np.linspace(first, last, steps+1)).astype(int)
    if len(set(indices)) != len(indices):
        raise ValueError('sample count exceeds trace resolution')
    picked = [nodes[int(i)] for i in indices]
    for phase in ('swing_ml','swing_mr','swing_bl','swing_br'):
        if not any(x['phase']==phase for x in picked):
            raise ValueError(f'missing phase {phase}; increase steps')
    a,b = mujoco_to_drake(picked[0]['qpos']),mujoco_to_drake(picked[-1]['qpos'])
    if np.max(np.abs(a[7:25]-b[7:25])) > 0.03:
        raise ValueError('not nominally joint-periodic')
    if abs(float(b[5]-a[5])-0.02)>0.005:
        raise ValueError('not nominally translating +20mm')
    return indices,picked

def generate(inzip, outdir, steps=28, dt=.1, iters=25):
    out=Path(outdir);out.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(inzip) as z:
        trace=json.loads(z.read('math_first_four_crawl_trace.json'))
    indices,picked=sample_nodes(trace['nodes'],steps=steps)
    src_yaml=Path(__file__).parent / 'source_cad30_smoke.yaml'
    if not src_yaml.exists():
        src_yaml=Path(__file__).resolve().parent.parent/'cad30_smoke/cad30.yaml'
    template=src_yaml.read_text()
    q_init=mujoco_to_drake(picked[0]['qpos']); q_end=mujoco_to_drake(picked[-1]['qpos'])
    import re
    overrides={
        'q_init':np.array2string(q_init,separator=', ',precision=11,max_line_width=9999),
        'q_nom_start':np.array2string(q_init,separator=', ',precision=11,max_line_width=9999),
        'q_nom_end':np.array2string(q_end,separator=', ',precision=11,max_line_width=9999),
        'q_guess':np.array2string(q_end,separator=', ',precision=11,max_line_width=9999),
        'num_steps':str(steps), 'time_step':str(dt),'max_iters':str(iters),
    }
    for name,value in overrides.items():
        template,count=re.subn(r'(?m)^'+name+r'\s*:[^\n]*',name+' : '+value,template)
        if count!=1:raise ValueError(f'{name}: {count} matches')
    cfg=out/'cad30_periodic.yaml';cfg.write_text(template)
    with (out/'nominal_4swing_drake.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerows(mujoco_to_drake(x['qpos']).tolist() for x in picked)
    info={'schema':'cad30-idto-all-4-swing-soft-reference-v1','source_pr':43,
          'phase':'four-leg cycle only; starts from mathematically prescribed post-prep 4-stance, not physically reached',
          'source_trace_indices':[int(i) for i in indices],
          'nominal_root_delta_y_mm':float(1000*(q_end[5]-q_init[5])),
          'nominal_max_leg_joint_closure_rad':float(np.max(np.abs(q_end[7:25]-q_init[7:25]))),
          'requested_swings':list(LEG_ORDER),
          'observed_reference_phase_samples':{p:int(sum(x['phase']==p for x in picked)) for p in ('swing_ml','swing_mr','swing_bl','swing_br')},
          'target_duration_s':steps*dt,'model_note':'Full original collision meshes + finite box floor; 6-leg CAD30; Drake 1.30; not MuJoCo contact equivalence.',
          'important':'IDTO soft reference has no periodic contact constraints; even a solved numerical trajectory is not G1/G2 gait PASS.'}
    (out/'reference_meta.json').write_text(json.dumps(info,indent=2)+'\n')
    return info

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--trace-zip',required=True);p.add_argument('--out',required=True);p.add_argument('--steps',type=int,default=28);p.add_argument('--dt',type=float,default=.1);p.add_argument('--iters',type=int,default=25)
    x=p.parse_args();print(json.dumps(generate(x.trace_zip,x.out,x.steps,x.dt,x.iters),indent=2))