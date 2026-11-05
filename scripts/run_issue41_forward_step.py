#!/usr/bin/env python3
"""Issue #41 research runner: reproduce baseline, record new physical attempts.

All experiments use the original owner's private CAD30 model with real MuJoCo
integration. Never generate a visually fabricated replay; never alter CAD files.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import platform
from pathlib import Path
from importlib.metadata import version
from two_support_transfer import run_probe

PARTIAL_PARAMS=dict(rear_advance_mm=10, landing_body_shift_mm=6,
                    forward_accel=4.5, forward_weight=20.,
                    qp_duration_s=.02, return_ramp_s=.24,
                    final_hold_s=.7, soft_joint_gain=5., soft_joint_threshold=1.63)


def digest(path:Path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1<<20),b''): h.update(block)
    return h.hexdigest()


def execute(model:Path,state:Path,out_dir:Path):
    out_dir.mkdir(parents=True,exist_ok=True)
    if not model.is_file() or not state.is_file():
        raise FileNotFoundError('Owner private CAD30 source model and safe physical pose are required; do not substitute a test robot')
    configurations=[
        ('baseline_4to2to4_mu1p1',dict(mu=1.1,record_qpos=True,record_physics_steps=True)),
        ('rear_5mm_mu1p1',dict(**{**PARTIAL_PARAMS,'rear_advance_mm':5},mu=1.1,record_qpos=True,record_physics_steps=True)),
        ('rear_10mm_mu1p1',dict(**PARTIAL_PARAMS,mu=1.1,record_qpos=True,record_physics_steps=True)),
        ('rear_10mm_mu0p8',dict(**PARTIAL_PARAMS,mu=.8,record_qpos=True,record_physics_steps=True)),
        ('rear_10mm_mu0p6',dict(**PARTIAL_PARAMS,mu=.6,record_qpos=True,record_physics_steps=True)),
        ('rear_10mm_mu0p3',dict(**PARTIAL_PARAMS,mu=.3,record_qpos=True,record_physics_steps=True)),
        ('rear_20mm_preflight',dict(**{**PARTIAL_PARAMS,'rear_advance_mm':20},mu=1.1,record_qpos=False,record_physics_steps=False)),
    ]
    summary={
        'schema':'alfred-hexapod-issue41-forward-rear-pair-v1',
        'source_model':'Freenove FNK0052 CAD30 nominal, 30 original private CAD meshes',
        'source_model_xml_sha256':digest(model),
        'safe_state_sha256':digest(state),
        'runtime':{x:version(x) for x in ['mujoco','mink','qpsolvers','daqp','numpy','scipy']},
        'python':platform.python_version(),
        'disclaimer':'Single nominal model; repeated deterministic exploration (not out-of-sample robustness). The 20-mm forward COM target has not passed.',
        'goals':{'actual_rear_pair_flight_then_land':True,'final_four_load_min_N':.2,'COM_forward_goal_mm':20.,
                 'model_max_joint_rad':1.7,'model_max_tau_Nm':.7,'minimum_two_loaded_supports':2},
        'selected_parameters':PARTIAL_PARAMS,
        'runs':{},
    }
    for name,kw in configurations:
        print('PHYSICAL_ATTEMPT',name,flush=True)
        r=run_probe(model,state,**kw)
        dest=out_dir/(name+'.json')
        dest.write_text(json.dumps(r,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n')
        summary['runs'][name]={
            'source_file':dest.name,'sha256':digest(dest),'bytes':dest.stat().st_size,
            'status':r['status'],'actual_two_support_s':r.get('longest_contiguous_true_two_support_seconds'),
            'actual_forward_airborne_s':r.get('longest_forward_airborne_continuous_seconds'),
            'forward_COM_mm':r.get('net_forward_COM_mm'),
            'rear_final_forward_mm':{k:v['forward_from_initial_mm'] for k,v in r.get('final_rear_footholds',{}).items()},
            'two_support_lost_steps':r.get('below_two_support_steps'),
            'last_100_four_loaded_steps':r.get('last_0p2s_four_reserve_steps'),
            'max_joint_abs_rad':r.get('max_joint_abs_rad'),
            'max_torque_Nm':r.get('max_motor_tau_Nm'),
            'qp_solver_failed':r.get('QP_solver_failed'),
            'recorded_2ms_steps':len(r.get('physics_steps_2ms') or []),
            'recorded_qpos_20ms':len(r.get('qpos_samples_20ms') or []),
        }
    assert summary['runs']['baseline_4to2to4_mu1p1']['status']=='TRANSIENT_TWO_SUPPORT_AND_FOUR_SUPPORT_RECOVERED'
    assert summary['runs']['rear_10mm_mu1p1']['status']=='PARTIAL_REAR_PAIR_FORWARD_STEP_LT20MM'
    assert summary['runs']['rear_10mm_mu0p8']['status']=='RESEARCH_GATE_FAILED'
    assert summary['runs']['rear_10mm_mu0p6']['status']=='RESEARCH_GATE_FAILED'
    assert summary['runs']['rear_10mm_mu0p3']['status']=='RESEARCH_GATE_FAILED'
    assert summary['runs']['rear_20mm_preflight']['status']=='TERMINAL_PREFLIGHT_BLOCKED'
    (out_dir/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    for fn in sorted(out_dir.glob('*.json')):
        print('FILE_SHA256',digest(fn),fn.name,flush=True)
    return summary

if __name__=='__main__':
    a=argparse.ArgumentParser()
    a.add_argument('--model',type=Path,required=True)
    a.add_argument('--state',type=Path,required=True)
    a.add_argument('--output-dir',type=Path,required=True)
    args=a.parse_args()
    execute(args.model,args.state,args.output_dir)
