"""Run Drake CAD30 penetration-allowance sweep from private recorded states.

CLI only; no private paths, qpos, STL/OBJ, or scene XML are bundled here.
The resulting JSON records only force summaries and initial root height.
"""
from __future__ import annotations
import argparse, json, os, subprocess
from pathlib import Path
from calibration_audit import LEGS, parse_contact_probe


def main() -> None:
    p=argparse.ArgumentParser()
    p.add_argument('--probe',type=Path,required=True)
    p.add_argument('--scene',type=Path,required=True)
    p.add_argument('--six-input',type=Path,required=True)
    p.add_argument('--four-input',type=Path,required=True)
    p.add_argument('--g0b-receipt',type=Path,required=True)
    p.add_argument('--out-dir',type=Path,required=True)
    p.add_argument('--delta-list',default='0.001,0.0002,0.0001,0.00006,0.000055,0.00005,0.00004')
    p.add_argument('--ticks',type=int,default=20)
    args=p.parse_args()
    if not 1 <= args.ticks <= 500:raise ValueError('ticks must be 1..500')
    args.out_dir.mkdir(parents=True,exist_ok=True)
    baseline=json.loads(args.g0b_receipt.read_text())
    output=[]
    for ds in args.delta_list.split(','):
        delta=float(ds)
        if not 0<delta<0.02:raise ValueError('invalid delta')
        for state,inpath in [('six',args.six_input),('four',args.four_input)]:
            cmd=[str(args.probe),str(args.scene),str(inpath),'--repair-actuators',str(delta),str(args.ticks)]
            run=subprocess.run(cmd,capture_output=True,text=True,env=os.environ.copy(),timeout=60)
            basename=f'{state}_delta_{delta:.6f}'
            (args.out_dir/f'{basename}.out').write_text(run.stdout)
            (args.out_dir/f'{basename}.err').write_text(run.stderr)
            if run.returncode:raise RuntimeError(f'{basename}: exit={run.returncode}; see stderr file')
            parsed=parse_contact_probe(run.stdout)
            values=list(map(float,inpath.read_text().split()))
            if len(values)<7:raise ValueError('expected 27q + 26v + 20u in Drake order')
            output.append({
             'delta_m':delta, 'state':state,
             'root_z_initial_m':values[6],
             'Fz_first_2ms':parsed['first_per_foot_N'],
             'mu_fz_ref':baseline['snapshots'][state]['mu_floor_box']['per_leg_Fz_N'],
             'force_l2_N':sum((parsed['first_per_foot_N'].get(l,0)-baseline['snapshots'][state]['mu_floor_box']['per_leg_Fz_N'].get(l,0))**2 for l in LEGS)**.5,
             'timeline':[{'tick':x['tick'],'total_force_N':x['force_N'],
                           'loaded_feet':x['loaded'],'root_z_m':x['root_z_m']}
                           for x in parsed['timeline']]
            })
    (args.out_dir/'refined_calibration_private.json').write_text(json.dumps(output,indent=2)+'\n')
    print(f'Completed {len(output)} actual physical Drake runs, {args.ticks} ticks each. See output directory.')
if __name__=='__main__':main()
