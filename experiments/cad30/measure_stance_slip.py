"""Postprocess recorded MuJoCo qpos/contacts to estimate stance foot slip.

This measures displacement of a distal *CAD-vertex proxy*, not the unknown
physical footpad contact centroid. It is a diagnostic of contact kinematics.
"""
from pathlib import Path
import argparse
import json
import numpy as np
import mujoco

from hexlab.stand_geometry_v04 import load_legs, tip_world
from locomotion_cad30 import ROOT


def inspect(report:dict)->dict:
    case=report['scenario']
    m=mujoco.MjModel.from_xml_path(str(ROOT/'source'/'mjcf'/f'scene_local30_{case}.xml'))
    d=mujoco.MjData(m)
    legs=load_legs(m,d,mujoco)
    snaps=report['qpos_samples']; trace=report['trace']
    tseries=np.array([x['t'] for x in trace])
    prev=None
    lengths={l['suffix']:[] for l in legs}
    for snap in snaps:
        t=snap['t']; d.qpos[:]=snap['qpos'];d.qvel[:]=0;mujoco.mj_forward(m,d)
        tip={l['suffix']:tip_world(m,d,l).copy() for l in legs}
        observed=trace[int(np.argmin(np.abs(tseries-t)))]
        contacts=set(observed['contacts'])
        if prev is not None:
            p,old_contacts=prev
            for c in (contacts & old_contacts):
                lengths[c].append(float(np.linalg.norm(tip[c][:2]-p[c][:2])))
        prev=(tip,contacts)
    all_s=np.array([v for arr in lengths.values() for v in arr])
    return {
      'samples':len(snaps),
      'stance_adjacent_foot_contact_pairs':len(all_s),
      'mean_stance_tip_proxy_displacement_mm_per_sample':round(float(all_s.mean())*1000,4) if len(all_s) else None,
      'p95_stance_tip_proxy_displacement_mm_per_sample':round(float(np.quantile(all_s,.95))*1000,4) if len(all_s) else None,
      'max_stance_tip_proxy_displacement_mm_per_sample':round(float(all_s.max())*1000,4) if len(all_s) else None,
      'total_stance_tip_proxy_displacement_mm':round(float(all_s.sum())*1000,2),
      'caution':'This is tibia farthest-vertex tracking, NOT measured real footpad slip. Some relative motion is foot rotation on mesh contact hull.'
    }

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('report',type=Path)
    a=p.parse_args()
    x=json.loads(a.report.read_text()); print(json.dumps(inspect(x),indent=2))