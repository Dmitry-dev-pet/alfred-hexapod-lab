"""Create fact-only G1 result from saved IDTO logs + independent MuJoCo audits.

This reads numerical results, never claims a sampled kinematic screen proves
real contact or a 6->4 physically reachable start. No models/CAD are included.
"""
from __future__ import annotations
import argparse,hashlib,json,re
from pathlib import Path
from acceptance_screen import screen

RUNS=(
    ('baseline_fast_80','idto80',0.1,80,2000.,.01,False),
    ('slow_default_70','idto_dt0p3',.3,70,2000.,.01,False),
    ('slow_sharper_contact_70','idto_sharp',.3,70,3000.,.002,False),
    ('intermediate_sharper_contact_70','idto_dt0p5',.5,70,3000.,.002,False),
    ('very_slow_sharper_contact_60','idto_dt0p9',.9,60,3000.,.002,False),
    ('equality_constraints_trial_35','idto_equality',.9,35,3000.,.002,True),
)

def sha(path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for b in iter(lambda:f.read(1<<20),b''):h.update(b)
    return h.hexdigest()

def build(root):
    root=Path(root);rows=[]
    for label,key,dt,maxit,stiffness,sigma,eq in RUNS:
        tag=key.replace('idto','run',1)
        log=root/(tag+'.stdout')
        a=json.loads((root/(key+'_audit.json')).read_text())
        s=log.read_text()
        iters=re.findall(r'^\|\s*(\d+)\s*\|\s*([\d.e+\-]+)\s*\|',s,re.M)
        if not iters: raise ValueError('No solver iterations '+str(log))
        iteration_count=int(iters[-1][0])+1
        m=re.search(r'Maximum iterations reached in ([\d.e+\-]+) seconds',s)
        status=('MAX_ITERATIONS_REACHED' if m else ('SUCCESS_REPORTED' if 'Solved in ' in s else 'FAILED_OR_UNKNOWN'))
        if iteration_count>maxit: raise ValueError('more iterations than config')
        row={
            'label':label,'time_step_s':dt,'period_duration_s':dt*28,
            'idto_iterations':iteration_count,'max_iters_configured':maxit,
            'contact_stiffness_N_m':stiffness,'contact_smoothing_factor_m':sigma,
            'unactuated_equality_constraints_requested':eq,
            'status':status,'final_printed_cost_approx':float(iters[-1][1]),
            'stdout_sha256':sha(log),
            'optimized_q_sha256':sha(root/(key+'_q.csv')),
            'optimized_v_sha256':sha(root/(key+'_v.csv')),
            'optimized_tau_sha256':sha(root/(key+'_tau.csv')),
            'kinematic_audit':a,'preliminary_gates':screen(a),
        }
        rows.append(row)
    physically=[]
    for label,key in [('pd_joint_targets_only','replay_dt0p9'),('pd_plus_idto_feedforward','replay_ff_dt0p9')]:
        r=json.loads((root/(key+'_physical_result.json')).read_text())
        physically.append({'label':label,'summary':r,'trace_sha256':sha(root/(key+'_physical_trace.jsonl.gz'))})
    return {
        'schema':'alfred-cad30-idto-full-4-swing-bounded-g1-v1',
        'date_utc':'2026-11-05',
        'status':'G1_FULL_PERIOD_NOT_FOUND',
        'preparation_from_original_six':'NOT_RUN',
        'source':'private original CAD30, full collision meshes in Drake and MuJoCo, no binary CAD in this receipt',
        'upstream_idto_commit':'de0629c7811aa9b330e56c4385629005b09495f0',
        'drake_version':'1.30.0','mujoco_version':'3.14.0',
        'known_initialization':'29 preplanned nodes from PR #43, first node 106 = mathematically prescribed post-prep four support; not physically reached in these trials',
        'numeric_runs':rows,'independent_mujoco_physical_diagnostics':physically,
        'limitations':[
           'IDTO internal contact model is smooth; no hard 3-support Fz/phase constraints are present.',
           'Unactuated equality=true run did not achieve zero unactuated residual before max iterations.',
           'Solver tau was not a hard constraint; reference mass, inertia and motor bounds are provisional.',
           'MuJoCo replay begins from virtual 4-support post-prep qpos; no physically executed 6->4 preparation.',
           'Real model front legs raised; 4 distinct toe transfers in sampled FK alone do not prove contact or periodic gait.',
           'No complete physical 4-swing gait and no three-cycle accepted result.'
        ]
    }

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',required=True);p.add_argument('--out',required=True);a=p.parse_args()
    result=build(a.root)
    Path(a.out).write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    for x in result['numeric_runs']:
        print(x['label'],x['status'],x['kinematic_audit']['max_abs_leg_tau_Nm'],x['preliminary_gates']['status'],x['preliminary_gates']['reasons'])
    print('real dyn', [(x['label'],x['summary']['real_root_y_delta_mm'],x['summary']['min_loaded_legs_any_step']) for x in result['independent_mujoco_physical_diagnostics']])
