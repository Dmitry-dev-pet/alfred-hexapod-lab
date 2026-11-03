"""Paired fixed plan: active stance-foot preload vs baseline force gate.
No post-hoc seed selection. 2ms contact samples, 50Hz trace, 8s per run.
"""
import json, statistics
from pathlib import Path
from active_load_transfer import run
from decentralized_ablation import ROOT
TRIALS={1.1:[11,23,37,51,97],0.6:[11,23,37],0.3:[11,23]}
MODES=('neighbor_force_gate_supported','neighbor_active_support')
rows=[]
for friction,seeds in TRIALS.items():
    for seed in seeds:
        for mode in MODES:
            r=run(mode,seed,friction=friction,write=True)
            keys=('mode','seed','friction_mu_assumed','root_advance_cm','min_grounded_legs',
                  'min_grounded_legs_2ms','fraction_below_3_contacts','fraction_below_3_2ms',
                  'fraction_zero_2ms','max_tilt_deg','peak_tau_nm','saturation_fraction',
                  'max_active_press_mm','active_press_time_fraction','lift_count',
                  'touchdown_count','finished_full_duration')
            row={k:r[k] for k in keys}
            rows.append(row)
            print(json.dumps(row),flush=True)
metrics={}
for friction in TRIALS:
    metrics[str(friction)]={}
    for mode in MODES:
        a=[r for r in rows if r['friction_mu_assumed']==friction and r['mode']==mode]
        metrics[str(friction)][mode]={
            'mean_advance_cm':round(statistics.mean(x['root_advance_cm'] for x in a),4),
            'mean_fraction_below_3_2ms':round(statistics.mean(x['fraction_below_3_2ms'] for x in a),5),
            'runs_below_3_2ms':sum(x['min_grounded_legs_2ms']<3 for x in a),
            'runs_with_zero_2ms':sum(x['fraction_zero_2ms']>0 for x in a),
            'mean_max_tilt_deg':round(statistics.mean(x['max_tilt_deg'] for x in a),4),
            'mean_peak_tau_nm':round(statistics.mean(x['peak_tau_nm'] for x in a),5),
        }
result={'date':'2026-11-03','experiment':'active_local_support_cad30_paired_fixed_protocol_v1',
  'model':'CAD30; not physically calibrated, no hardware measurements',
  'conditions':{'mass_case':'nominal','seconds':8.,'stride_m':.035,'lift_m':.020,
    'period_s':2.4,'press_depth_mm':4.,'max_press_rate_mm_s':20.,
    'friction_seed_plan':{str(k):v for k,v in TRIALS.items()},
    'normal_force_filter_tau_ms':50.,'sample_period_ms':2.,
    'note':'same initial seed per controller pair, no sensor delay/noise'},
  'runs':rows,'metrics':metrics}
(ROOT/'results/active_local_support_paired_summary_2026-11-03.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print('SUMMARIES:',json.dumps(result['metrics'],indent=2),flush=True)
