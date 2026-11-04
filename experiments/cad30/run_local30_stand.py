"""Pure in-session MuJoCo check; no Actions or network.
Uses pinned 30/32 Freenove visual meshes; motor COM and all six real tibia contact meshes.
"""
from pathlib import Path
import json,sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from hexlab import cad_mujoco_v04 as cad
from build_30meshes import build as ensure

def local_build(case='nominal',*,cache=None,offline=False):
    p=ROOT/'source'/'mjcf'/f'scene_local30_{case}.xml'
    if not p.exists():ensure(case)
    return p,{'upstream_commit':'5c9a84c16335b30036e451c691d2c544abf4bc72','estimated_assembly_mass_kg':{'nominal':1.804868,'lower':1.644,'upper':2.08}[case]}

cad.build=local_build
from hexlab.stand_v04 import experiment
if __name__=='__main__':
    for case,sec in [('nominal',5.0),('lower',3.0),('upper',3.0)]:
        rep=experiment(case,sec)
        (ROOT/'results'/f'cad30_stand_{case}.json').write_text(json.dumps(rep,indent=2)+'\n')
        print(json.dumps({'case':case,'success':rep['six_leg_standing_candidate_succeeded'],
            'fit':{k:v for k,v in rep['fit'].items() if k!='joint_poses'},
            'hold':{k:v for k,v in rep['hold_test'].items() if k not in ('time_series_50hz','final_qpos')},
            'condition':'30 original meshes plus 2 purely visual proxies'}))
