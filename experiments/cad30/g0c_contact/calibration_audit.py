"""CAD30 G0c audit: compare physical contact histories and ex-post IDTO torque maxima.

The compiler/simulator input meshes and qpos/qvel remain private and are never
serialized. A contact match in two checkpoints does NOT certify gait/contact
parity. Does not install a hard constraint inside IDTO.
"""
from __future__ import annotations
import argparse, hashlib, json, math, re
from pathlib import Path

LEGS = ('ml', 'mr', 'bl', 'br', 'fl', 'fr')


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def idto_soft_contact_force(gap_m: float, stiffness_N_per_m: float = 2000,
                            smoothing_m: float = 0.01) -> float:
    """IDTO v-de0629 normal-force model at zero normal speed, exact softplus."""
    if stiffness_N_per_m <= 0 or smoothing_m <= 0:
        raise ValueError('positive stiffness/smoothing required')
    x = -gap_m / smoothing_m
    softplus = max(x, 0.) + math.log1p(math.exp(-abs(x)))
    return stiffness_N_per_m * smoothing_m * softplus


def extract_torque_peaks(log_text: str) -> list[float]:
    m = re.search(r'Max torques:\s*\[([^\]]+)\]', log_text, flags=re.I)
    if not m:
        raise ValueError('No IDTO Max torques vector in log')
    x = [float(v) for v in re.split(r'[\s,;]+', m.group(1).strip()) if v.strip()]
    if len(x) != 26 or not all(math.isfinite(v) and v >= 0 for v in x):
        raise ValueError('Expected 26 nonnegative finite peak generalized forces (6+18+2)')
    return x


def torque_posthoc_gate(log_text: str) -> dict:
    x = extract_torque_peaks(log_text)
    leg = x[6:24]
    head = x[24:26]
    cap_pass = max(leg) <= 0.7 + 1e-12 and max(head) <= 0.2 + 1e-12
    return {
        'generalized_force_dim': len(x),
        'leg_actuator_count': len(leg), 'head_actuator_count': len(head),
        'max_model_leg_torque_Nm': max(leg), 'leg_cap_Nm': 0.7,
        'max_model_head_torque_Nm': max(head), 'head_cap_Nm': 0.2,
        'ex_post_cap_check': 'PASS' if cap_pass else 'FAIL',
        'run_convergence': ('MAX_ITERATIONS_REACHED' if 'Maximum iterations reached' in log_text
                            else 'SOLVED_MESSAGE' if 'Solved in ' in log_text else 'UNKNOWN'),
        'hard_constraint_inside_idto': 'NOT_PRESENT_IN_PINNED_PROBLEM_DEFINITION',
        'certified_torque_feasible': False,
        'note': 'Ex-post maxima only; no IDTO hard box inequalities, real PD torque realization, or contact support certification.'
    }


def parse_contact_probe(raw: str) -> dict:
    fz: dict[str, float] = {}
    ticks = []
    for line in raw.splitlines():
        tokens = line.split()
        if len(tokens) == 3 and tokens[0] == 'drake_fz':
            leg = tokens[1].removeprefix('tibia')
            if leg in LEGS: fz[leg] = float(tokens[2])
        elif len(tokens) == 5 and tokens[0] == 'timeline':
            ticks.append({'tick':int(tokens[1]), 'force_N':float(tokens[2]),
                          'loaded':int(tokens[3]), 'root_z_m':float(tokens[4])})
    if not fz or not ticks or [x['tick'] for x in ticks] != list(range(1, len(ticks) + 1)):
        raise ValueError('Malformed or incomplete Drake 2-ms timeline')
    return {'first_per_foot_N':fz, 'timeline':ticks}


def summarize_refined(refined: list[dict], original: dict, temporal: list[dict]) -> dict:
    report={}
    six_original=original['snapshots']['six']['mu_floor_box']['per_leg_Fz_N']
    four_original=original['snapshots']['four']['mu_floor_box']['per_leg_Fz_N']
    for a in refined:
        state=a['state']; expected=6 if state=='six' else 4
        samples=a['timeline']; end=samples[-1]
        if len(samples)!=20 or [x['tick'] for x in samples]!=list(range(1,21)):
            raise ValueError('Every refined trial must cover exactly 20 physical ticks')
        target=six_original if state=='six' else four_original
        actual=a['Fz_first_2ms']
        per_leg_rmse=(sum((actual.get(x,0.)-target[x])**2 for x in LEGS)/len(LEGS))**0.5
        report.setdefault(f'{a["delta_m"]:.6f}',{})[state]={
            'reference_MuJoCo_Fz_2ms_N':round(sum(target.values()),6),
            'Drake_Fz_2ms_N':round(sum(actual.values()),6),
            'first_step_per_leg_rmse_N':round(per_leg_rmse,6),
            'min_loaded_foot_count_during_40ms':min(x['loaded_feet'] for x in samples),
            'all_needed_supports_each_2ms': all(x['loaded_feet']>=expected for x in samples),
            'final_Drake_Fz_40ms_N':round(end['total_force_N'],6),
            'max_Drake_Fz_in_40ms_N':round(max(x['total_force_N'] for x in samples),6),
            'root_vertical_sag_40ms_mm':round(1000*(a['root_z_initial_m']-end['root_z_m']),6),
            'contact_time_ticks':len(samples)
        }
    for t in temporal:
        state=t['state']
        report.setdefault('mu_temporal',{})[state]={
            'after_one_2ms_step_Fz_N':t['after_mj_step_unrecomputed_N'],
            'after_2ms_end_contact_recompute_N':t['after_recompute_at_end_N'],
            'last_mujoco_fixed_torque_40ms_Fz_N':t['sampled_fixed_u'][-1]['total_Fz_N'],
            'loaded_40ms':t['sampled_fixed_u'][-1]['loaded_count']
        }
    return report


def create_report(refined_file: Path, original_file: Path,
                  temporal_file: Path, idto_log: Path) -> dict:
    refined=json.loads(refined_file.read_text())
    original=json.loads(original_file.read_text())
    temporal=json.loads(temporal_file.read_text())
    dynamics=summarize_refined(refined,original,temporal)
    smoke=torque_posthoc_gate(idto_log.read_text(errors='replace'))
    return {
       'schema':'cad30-g0c-contact-and-torque-audit-v1',
       'date':'2026-11-05',
       'input_sha256':{'original_G0b_receipt':sha256(original_file),
                       'refined_real_drake_timelines':sha256(refined_file),
                       'mu_temporal_replay':sha256(temporal_file),
                       'original_IDTO_smoke_stdout':sha256(idto_log)},
       'physics':{'Drake':'1.30.0', 'MuJoCo':'3.14.0', 'dt_s':0.002,
                  'input_states':'same archived PR #40 six/four physical checkpoints',
                  'controls':'identical fixed initial PD torque for each separate hold; no qpos resets between its 20 ticks'},
       'contact_sweep':dynamics,
       'promising_delta_m':0.000055,
       'IDTO_smooth_contact_at_zero_normal_speed':{
           'stiffness_N_per_m':2000,'smoothing_m':0.01,
           'force_at_gap_0mm_N':idto_soft_contact_force(0),
           'force_at_gap_2mm_N':idto_soft_contact_force(.002),
           'force_at_gap_20mm_N':idto_soft_contact_force(.02),
           'note':'IDTO internal differentiable contact is distinct from Drake MultibodyPlant penetration_allowance.'},
       'idto_original_smoke_torque_audit':smoke,
       'verdict':{
         'Drake_discrete_contact_parity_40ms':'PARTIAL_TWO_CHECKPOINTS_ONLY',
         'selected_configuration_four_supports_all_20_steps':dynamics['0.000055']['four']['all_needed_supports_each_2ms'],
         'hard_IDTO_actuator_limits':'NOT_IMPLEMENTED',
         'full_gait':'NOT_FOUND',
         'cross_engine_contact_equivalence':'NOT_CERTIFIED'
       },
       'limitations':[
        '20-tick hold of two archived checkpoints only; not dynamic swinging gait, friction sweep or repeated period',
        'IDTO uses a separate smoothed contact model whose force at a positive gap is nonzero',
        'Output max torque check does not enforce inequality constraints inside IDTO',
        'Drake and MuJoCo use different contact manifolds and timestepping conventions',
        'Values of torque caps and friction are provisional research limits, not measured hardware specs',
        'This report contains no CAD/STL, no full qpos/qvel, and no proprietary assets.'
       ]
    }


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--refined',type=Path,required=True)
    p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--mujoco-temporal',type=Path,required=True)
    p.add_argument('--idto-stdout',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args()
    r=create_report(a.refined,a.baseline,a.mujoco_temporal,a.idto_stdout)
    a.out.parent.mkdir(parents=True,exist_ok=True)
    a.out.write_text(json.dumps(r,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    print(json.dumps(r['verdict'],indent=2))
if __name__=='__main__':main()
