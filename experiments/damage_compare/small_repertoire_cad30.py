"""Preliminary, finite-repertoire IT&E-style physical-physics smoke experiment.

Runs six handcrafted feasible policies on the *intact* Freenove CAD30 in MuJoCo,
then chooses up to 4 distinct policies for the same damaged robot based only on
intact prior and previous damaged trials (no lookahead). Not published IT&E:
no MAP-Elites/behavior-diversity optimization, not 36D resibots controllers,
and no adaptation during an individual episode. Not neuroWalknet either.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = next((p for p in HERE.parents if (p / 'hexlab').is_dir()), None)
if ROOT is None:
    raise RuntimeError('Restore original CAD30 tree alongside hexlab/')
for p in (str(HERE), str(HERE.parent), str(ROOT)):
    if p not in sys.path: sys.path.insert(0, p)
from ite_selector import select_next
from failure_one_leg_cad30 import initialize, run, get_trajectory, tip_world, solve_waypoint

# Pre-registered before inspecting any results of these proposed controller variants.
# Each design is a step length [m], foot lift [m], and period [s].
DESIGNS = (
    ('short_slow', 0.020, 0.012, 2.8),
    ('small_standard', 0.025, 0.014, 2.4),
    ('existing_baseline', 0.035, 0.020, 2.4),
    ('long_slow', 0.040, 0.018, 2.6),
    ('small_fast', 0.025, 0.018, 2.0),
    ('long_low_fast', 0.035, 0.012, 2.0),
)


def descriptor(design):
    _name, stride, lift, period = design
    # Fixed scaling from defensible engineering parameters, not learned from
    # outcome, normalized roughly into a [0,1] cube.
    return [(stride - .015) / .030, (lift - .010) / .015, (period - 1.8) / 1.2]


def context_with_design(base: dict, design: tuple) -> dict:
    _name, stride, lift, period = design
    model = base['model']
    legs = base['legs']
    q = base['start_qpos']
    plans, ikerr = get_trajectory(model, q, legs, base['forward'], stride, lift, base['duty'])
    ref = mujoco.MjData(model)
    land = {}
    for leg in legs:
        code = leg['suffix']
        ref.qpos[:] = q
        ref.qvel[:] = 0
        mujoco.mj_forward(model, ref)
        tgt = tip_world(model, ref, leg) + np.r_[base['forward'] * stride * .5, -.006]
        land[code] = solve_waypoint(model, ref, mujoco, leg, tgt,
                                     plans[code][min(35, int(base['duty'] * 36))])[0]
    result = base.copy()
    result.update(period=period, plans=plans, land=land)
    return result


def score_episode(out):
    """Predeclared utility, *not* proof of stability or an objective optimum."""
    s, trace = out['summary'], out['trace_50hz']
    t0 = 3.0
    post = [r for r in trace if r['t'] >= t0]
    frac_lt3 = sum(r['n_contacts'] < 3 for r in post) / len(post)
    frac_zero = sum(r['n_contacts'] == 0 for r in post) / len(post)
    advance_cm = float(s['post_fault_advance_cm'])
    # Largest penalty for real fall, mild risk-penalty for losing static support.
    reward = advance_cm - 8.0 * frac_lt3 - 20.0 * frac_zero
    if s['fell_by_threshold'] or not s['completed']:
        reward -= 50.0
    return {
        'reward': round(reward, 4),
        'progress_cm_t3_to_t8': advance_cm,
        'frac_lt3_post': round(frac_lt3, 5),
        'frac_zero_post': round(frac_zero, 5),
        'min_contacts_post': int(s['post_fault_min_contacts']),
        'max_tilt_deg_post': float(s['post_fault_max_tilt_deg']),
        'fell_by_threshold': bool(s['fell_by_threshold']),
        'peak_tau_nm': s['peak_tau_nm'],
        'finished_full_duration': bool(s['completed']),
    }


def main(args):
    base = initialize()
    all_contexts = [context_with_design(base, design) for design in DESIGNS]
    prior = []
    for i, (design, ctx) in enumerate(zip(DESIGNS, all_contexts)):
        print(f'INTACT repertoire {i+1}/{len(DESIGNS)}: {design[0]}', flush=True)
        out = run(ctx, seed=args.archive_seed, damaged=None, mode='neighbor_inhibit', retain_frames=False)
        prior.append(score_episode(out))
    prior_scores = np.array([r['reward'] for r in prior])
    x = np.asarray([descriptor(z) for z in DESIGNS])
    tested = []
    trials = []
    for t in range(args.trials):
        idx, diag = select_next(x, prior_scores, tested)
        print(f'DAMAGED trial {t+1}/{args.trials}: {idx}={DESIGNS[idx][0]}', flush=True)
        out = run(all_contexts[idx], seed=args.test_seed, damaged='fl', mode='neighbor_inhibit', retain_frames=False)
        scored = score_episode(out)
        tested.append((idx, scored['reward']))
        trials.append({'trial':t+1,'candidate':DESIGNS[idx][0],'idx':idx,'score':scored,'selection':diag})
    winner = max(trials, key=lambda z: z['score']['reward'])
    # Benchmark current fixed rule with *identical* physical settings and test seed.
    baseline = run(all_contexts[2], seed=args.test_seed, damaged='fl', mode='neighbor_inhibit', retain_frames=False)
    final = {
        'status': 'SMALL_6_CANDIDATE_IT_AND_E_STYLE_PROOF_OF_CONCEPT_NOT_PUBLISHED_IT_AND_E',
        'simulator': f'MuJoCo {mujoco.__version__}', 'robot':'CAD30 FNK0052 approx',
        'archive_seed_intact':args.archive_seed,'damaged_seed_heldout':args.test_seed,
        'fault':'fl 3 motors off and tibia collision off at t=3s, masses retained',
        'physics_dt_s':float(base['model'].opt.timestep),
        'online_evaluation_count':args.trials,
        'controls':{'duration_s':8,'mu':1.1,'mass_kg':1.804868,'motor_torque_max_nm_assumed':.7},
        'candidate_policies':[{'name':z[0],'stride_m':z[1],'lift_m':z[2],'period_s':z[3],
                               'descriptor':descriptor(z),'intact':prior[i]}
                              for i,z in enumerate(DESIGNS)],
        'online_trials':trials,'chosen_best':winner,
        'fixed_reference':score_episode(baseline),
        'notes':[
            'No MAP-Elites archive trained; six hand-selected candidates only.',
            'Not original pyite/ite_v2; independent compact GP selector follows its conceptual equations.',
            'All damaged evaluations reset the simulation and therefore consume full simulated trials, not single-episode adaptation.',
            'Intact archive was computed at a different seed, no damaged results entered the prior.',
            'No neuroWalknet run; the currently used neighbor-inhibition rule is only a heuristic.',
            'No validation against a physical servo or robot; do not compare speeds with original ResiBots.',
            'A successful episode may have one or two ground supports; none is asserted safe.',
        ]
    }
    path=ROOT/'results'/f'small_repertoire_ite_smoke_seed{args.test_seed}_2026-11-04.json'
    path.write_text(json.dumps(final,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    print('RESULT:',json.dumps({'prior':prior,'trials':[{'candidate':z['candidate'],'reward':z['score']['reward']} for z in trials], 'reference':final['fixed_reference'],'best':winner['candidate'],'saved':str(path)},ensure_ascii=False),flush=True)
    return final

if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--archive-seed',type=int,default=97)
    parser.add_argument('--test-seed',type=int,default=23)
    parser.add_argument('--trials',type=int,default=4)
    args=parser.parse_args()
    main(args)