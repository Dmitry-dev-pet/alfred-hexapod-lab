"""Audit the immutable 2026-11-03 CAD30 active-support research ZIP.

Records how often *commanded* foot-lowering corrections were active in 50Hz
traces and how large the corrections were around actual liftoff events. These
are command offsets, not measured physical sole displacements. Also summarizes
pre-existing paired baseline/active physical stability endpoints (2ms).
"""
from __future__ import annotations
import argparse, hashlib, json, zipfile
from collections import defaultdict
from pathlib import Path
import numpy as np

CODES=('fl','ml','bl','br','mr','fr')
RING=('fl','ml','bl','br','mr','fr')
NEIGHBORS={leg:(RING[(i-1)%6],RING[(i+1)%6]) for i,leg in enumerate(RING)}


def audit(path:Path) ->dict:
    sha=hashlib.sha256(path.read_bytes()).hexdigest()
    records=[];paired=defaultdict(dict)
    with zipfile.ZipFile(path) as arc:
        for name in arc.namelist():
            if not (name.startswith('full_traces/') and name.endswith('.json')):
                continue
            run=json.loads(arc.read(name))
            k=(str(run['friction_mu_assumed']),int(run['seed']))
            paired[k][run['mode']]=run
            if run['mode']!='neighbor_active_support':continue
            trace=run['trace_50hz']
            pre=np.asarray([[x['preload_mm'][c] for c in CODES] for x in trace])
            t=np.asarray([x['t'] for x in trace])
            lifts=[e for e in run['events'] if e['event']=='liftoff']
            neighbor_at_lift=[]
            for e in lifts:
                i=int(np.argmin(abs(t-float(e['t']))))
                neighbor_at_lift.append(max(float(trace[i]['preload_mm'][c]) for c in NEIGHBORS[e['leg']]))
            records.append({
                'mu':run['friction_mu_assumed'],'seed':run['seed'],
                'mean_commanded_foot_bias_mm_all_legs_times':round(float(pre.mean()),6),
                'fraction_of_leg_times_bias_gt_0p1mm':round(float((pre>.1).mean()),6),
                'maximum_foot_bias_mm':round(float(pre.max()),6),
                'liftoffs':len(lifts),
                'mean_largest_neighbor_bias_at_liftoff_mm':round(float(np.mean(neighbor_at_lift)),6),
                'fraction_liftoffs_with_neighbor_bias_ge_1mm':round(float(np.mean(np.asarray(neighbor_at_lift)>=1.)),6),
                'fraction_below3_physics_2ms':run['fraction_below_3_2ms'],
                'max_tilt_deg':run['max_tilt_deg']})
    groups=[]
    for mu in [1.1,.6,.3]:
        selection=[r for r in records if r['mu']==mu]
        if not selection:continue
        p=[(a['neighbor_force_gate_supported'],a['neighbor_active_support'])
           for (k_mu,seed),a in paired.items()
           if float(k_mu)==mu and 'neighbor_force_gate_supported' in a and 'neighbor_active_support' in a]
        groups.append({'mu':mu,'n_paired_seeds':len(p),
            'mean_bias_mm_across_all_legs_times':round(float(np.mean([r['mean_commanded_foot_bias_mm_all_legs_times'] for r in selection])),6),
            'fraction_leg_times_bias_gt_0p1mm':round(float(np.mean([r['fraction_of_leg_times_bias_gt_0p1mm'] for r in selection])),6),
            'max_bias_mm':round(float(max(r['maximum_foot_bias_mm'] for r in selection)),6),
            'total_liftoffs':sum(r['liftoffs'] for r in selection),
            'mean_max_neighbor_bias_at_liftoff_mm':round(float(np.average(
               [r['mean_largest_neighbor_bias_at_liftoff_mm'] for r in selection],
               weights=[r['liftoffs'] for r in selection])),6),
            'fraction_liftoffs_with_neighbor_bias_ge_1mm':round(float(np.average(
               [r['fraction_liftoffs_with_neighbor_bias_ge_1mm'] for r in selection],
               weights=[r['liftoffs'] for r in selection])),6),
            'mean_fraction_below_3_contacts_2ms_baseline':round(float(np.mean([x[0]['fraction_below_3_2ms'] for x in p])),6),
            'mean_fraction_below_3_contacts_2ms_active':round(float(np.mean([x[1]['fraction_below_3_2ms'] for x in p])),6),
            'mean_peak_tilt_deg_baseline':round(float(np.mean([x[0]['max_tilt_deg'] for x in p])),6),
            'mean_peak_tilt_deg_active':round(float(np.mean([x[1]['max_tilt_deg'] for x in p])),6)})
    return {'schema':'fnk0052_preload_existing_trace_audit/1',
            'source_archive_basename':path.name,'sha256':sha,
            'preload_unit':'mm of commanded foot IK offset; NOT actual measured deflection',
            'trace_temporal_resolution_hz':50,'contacts_stability_resolution_ms':2,
            'groups':groups,'runs':records,
            'notes':['Time-nearest 50Hz preload estimate at liftoff is approximate by <=10 ms.',
                    'Small paired groups: not a statistical significance test.',
                    'Actual forces are available in existing 50Hz traces; controller lacks direct force setpoints.']}


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--zip',required=True,type=Path)
    p.add_argument('--out',required=True,type=Path)
    a=p.parse_args()
    summary=audit(a.zip)
    a.out.parent.mkdir(parents=True,exist_ok=True)
    a.out.write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    for x in summary['groups']:
        print(x)