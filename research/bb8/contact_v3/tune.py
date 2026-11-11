"""Bounded nominal-only inner-loop gain scan; a failed scan never validates behavior."""
from pathlib import Path
import argparse,json
import numpy as np
import mujoco
from simulate import simulate,make,zero_angle,load_parts,build

def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--tire',type=Path,required=True)
    p.add_argument('--fit',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--allow-estimated-mass',action='store_true')
    a=p.parse_args()
    if not a.allow_estimated_mass:p.error('--allow-estimated-mass required')
    a.out.mkdir(parents=True,exist_ok=True);profile=build(a.source,a.tire,'nominal');parts,_=load_parts(a.source,a.tire)
    fit=np.array(json.loads(a.fit.read_text())['fit']['rigid_translation_from_source_pose_mm'])*.001
    zero,_=zero_angle(profile,fit);xml,meta=make(profile,parts,fit,level=4,peak=.3,collision='facets')
    model=mujoco.MjModel.from_xml_string(xml);(a.out/'gain-scan-model.xml').write_text(xml);results=[]
    for kp,kd in [(.1,.01),(.1,.04),(.3,.01),(.3,.04),(.7,.04),(.7,.1),(1.2,.04),(1.2,.1),(-.1,-.01),(-.3,-.04)]:
        gains=dict(kp=kp,kd=kd,kv=0.,ki=0.);r=simulate(model,meta,fit,profile,zero,'hold',2.5,gains,peak=.3)
        s=r['summary'];score=s['rms_pitch_error_deg']+100*s['final_speed_m_s']
        results.append({'gains':gains,'score':score,'summary':s})
        print(json.dumps({'gains':gains,'score':score}),flush=True)
    (a.out/'gain_scan.json').write_text(json.dumps(results,indent=2)+'\n')

if __name__=='__main__':main()
