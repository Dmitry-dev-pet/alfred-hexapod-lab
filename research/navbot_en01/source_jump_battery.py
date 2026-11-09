"""Run the predeclared 3x4 paired source-jump diagnostic battery."""
from __future__ import annotations
import argparse,hashlib,json
from pathlib import Path
from .source_jump import ANCHORS,PERIODS_MS
from .source_jump_probe import run
from .source_jump_export import export

def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    rows=[]
    for anchor in ANCHORS:
        aname={-.04:'a-neg004',0.:'a-zero',.04:'a-pos004'}[anchor]
        for period in PERIODS_MS:
            for jump in (False,True):
                case=f'{aname}-p{period}-'+('jump' if jump else 'baseline')
                path=a.out/'raw'/(case+'_raw.json');s=run(path,loop_ms=period,anchor_q=anchor,jump=jump)
                r=export(path,a.out/'replay'/('en01-source-jump-v4-'+case+'-20261109.json'))
                rows.append({'case':case,**s,'export':r})
                print(json.dumps({'case':case,'jump_pass':s['jump_pass'],'flight':s['qualified_flight'],'clearance_mm':round(s['max_min_wheel_clearance_m']*1000,3),'max_x_cm':round(s['max_abs_x_m']*100,2),'nonwheel_steps':s['nonwheel_ground_steps']}),flush=True)
    summary={'schema':'en01-source-jump-battery/v1','cases':len(rows),'physical_steps':sum(s['physics_steps'] for s in rows),'frames':sum(s['frame_count'] for s in rows),'rows':rows}
    (a.out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    manifest={str(p.relative_to(a.out)):{'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in sorted(a.out.rglob('*.json'))}
    (a.out/'MANIFEST.json').write_text(json.dumps(manifest,indent=2)+'\n')

if __name__=='__main__':main()
