"""One-factor lift sweep, retaining the baseline mechanical/actuator model."""
import argparse
import hashlib
import json
from pathlib import Path
import platform

import mujoco
import numpy as np

import baseline as b


def run(assets, output, lifts, duration=24., timestep=.001):
    output.mkdir(parents=True,exist_ok=True)
    b.DT=timestep
    manifest,model,path,inertias,fk_error=b.build(assets,output,.020)
    records=[]
    for mm in lifts:
        name=f'lift-{mm:g}mm-step-20mm'
        try:
            case=b.run_case(manifest,model,path,output,name,True,duration,lift=mm*.001)
            case['status']='simulated'
        except AssertionError as error:
            if not str(error).startswith('IK target not reached:'):
                raise
            case={'case':name,'status':'infeasible_ik','error':str(error)}
        case['lift_mm']=mm
        records.append(case)
        receipt={'schema':'hexarth-lift-sweep/v1','baseline_commit':'bcb28c6e235bd413c8b74519f209a01a6034bbc3',
                 'python':platform.python_version(),'mujoco':mujoco.__version__,'timestep_s':timestep,
                 'duration_s':duration,'sample_s':b.SAMPLE,'source_step_sha256':manifest['step_sha256'],
                 'model_file':path.name,'model_sha256':b.digest(path),'mass_kg':float(model.body_mass.sum()),
                 'joint_range_rad':[-b.RANGE,b.RANGE],'rated_torque_nm':b.TORQUE,
                 'stall_torque_nm':b.STALL,'no_load_speed_rad_s':b.SPEED,'max_fk_error_m':fk_error,
                 'gait':{'stride_m':.020,'cycle_s':1.6,'settle_s':2.,'step_height_m':.020},
                 'source_sha256':{p.name:b.digest(p) for p in (Path(b.__file__),Path(__file__))},
                 'cases':records,'limitations':['Uncalibrated mass/COM/inertia and servo envelope.',
                 'Diagnostic +/-20 degree joint ranges, not hardware limits.',
                 'Fixed periodic gait; no terrain sensing or body height adaptation.',
                 'Collision uses convex hulls of individual CAD components.']}
        (output/'lift-receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
        print(json.dumps(case),flush=True)
    return receipt


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--assets',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--lifts',nargs='+',type=float,default=[12,18,24,30])
    p.add_argument('--duration',type=float,default=24.)
    p.add_argument('--timestep',type=float,default=.001)
    a=p.parse_args()
    run(a.assets,a.output,a.lifts,a.duration,a.timestep)
