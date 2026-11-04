"""Confirmatory sweep after explicitly labelled exploratory foothold search.

A pass means *one rear-foot relocation while the front pair is airborne*.
It does not mean the middle leg lifted or four-legged gait succeeded.
"""
from pathlib import Path
import json, argparse
from rear_foothold_reposition_v1 import run
HERE=Path(__file__).resolve().parent

PLAN=[{'mass':mass,'mu':mu,'rear_leg':'br','dx':.04,'dy':.20}
      for mass in ('lower','nominal','upper') for mu in (1.1,.6,.3)]


def produce():
    rows=[]
    for x in PLAN:
        r=run(**x)
        row={k:v for k,v in r.items() if k not in ('trace','qpos_frames')}
        rows.append(row)
        print(f"{x['mass']:8} mu={x['mu']:.1f} safe={str(row.get('safe_rear_reposition')):5} "
              f"COM margin={1000*(row.get('margin_last_m') or 0):+.1f}mm "
              f"rear swing support min={row.get('min_rear_swing_support_legs')} "
              f"gate releases={row.get('gate_release_count')}",flush=True)
    control=run(dx=.04,dy=.12,mu=1.1,rear_leg='br')
    mirror=run(dx=-.04,dy=.20,mu=1.1,rear_leg='bl')
    result={
        'schema':'alfred_hexapod_CAD30_rear_foothold/1',
        'date':'2026-11-04',
        'model':'MuJoCo 3.14.0, Freenove FNK0052 CAD30, uncalibrated virtual physical model',
        'fixed_confirmatory_plan':PLAN,
        'selection_note':'Right rear +200mm forward/+40mm outward chosen after exploratory target sweep. 3x3 sweep is a confirmation conditional on that selection, not a blind preregistered test.',
        'control_same_rear_120mm_forward':{k:v for k,v in control.items() if k not in ('trace','qpos_frames')},
        'mirror_nominal_left_rear':{k:v for k,v in mirror.items() if k not in ('trace','qpos_frames')},
        'pass_counts':{'safe_rear_reposition':sum(r.get('safe_rear_reposition',False) for r in rows),'total':len(rows),
                       'middle_leg_swing_gate_release':sum(bool(r.get('gate_release_count')) for r in rows)},
        'results':rows,
        'conclusions':['Repositioning changes support polygon and can make leave-one-middle COM margin positive',
                       'No middle foot liftoff, no four-legged gait or forward walking validated',
                       'No physical servo calibration; mesh collision simplified to tibia hulls']}
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=HERE/'results/rear_foothold_v1_confirmatory_2026-11-04.json');args=p.parse_args()
    result=produce();args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print('WRITTEN',args.out)
