"""Mink/MuJoCo sampled 4-step math-first lookahead with contact reconfiguration.

Exploratory NOMINAL-MODEL search result: six exploratory swings total (two
rear positioning swings in a PREPARATION phase and exactly four separate
swing-foot steps in the CYCLE phase). All body translations in this module
are *virtual* coordinate changes, not physical actuation. Each waypoint
uses real CAD30 kinematics and conservative gravity-wrench LP under assumed
friction and motor caps, but no dynamic contact or transition proof.
"""
from __future__ import annotations
import argparse,json,math
from pathlib import Path
import numpy as np
import mujoco as mj
from mink_contact_preflight import STANCE,StepPreflight,load_context,com_xyz,margin_xy,wrench_feasibility

REAR=('bl','br')
MIDDLE=('ml','mr')


def one_leg_waypoints(planner,leg,delta_xy,lift_height=.010,steps_per_stage=4):
    """Lossless logged step: lift, move xy in swing, then lower.

    Returns (status, last_row) and never approves an invalid waypoint.
    """
    x_delta,y_delta=np.asarray(delta_xy,float)
    origin=np.asarray(planner.targets[leg]).copy()
    rest=tuple(k for k in STANCE if k!=leg)
    for i in range(steps_per_stage):
        planner.targets[leg][2]=origin[2]+lift_height*(i+1)/steps_per_stage
        row=planner.checkpoint(f'{leg}_lift_{i+1}',rest)
        if not row['gate_passed']:return 'blocked_lift',row
    for i in range(steps_per_stage):
        planner.targets[leg][:2]=origin[:2]+np.array([x_delta,y_delta])*(i+1)/steps_per_stage
        row=planner.checkpoint(f'{leg}_swing_{i+1}',rest)
        if not row['gate_passed']:return 'blocked_swing',row
    for i in range(steps_per_stage):
        planner.targets[leg][2]=origin[2]+lift_height*(steps_per_stage-i-1)/steps_per_stage
        support=STANCE if i==steps_per_stage-1 else rest
        row=planner.checkpoint(f'{leg}_touchdown_{i+1}',support)
        if not row['gate_passed']:return 'blocked_touchdown',row
    return 'sampled_step_pass',row


def pose_stats(ctx,qpos):
    d=mj.MjData(ctx.model);d.qpos[:]=qpos;mj.mj_forward(ctx.model,d)
    return d,com_xyz(ctx.model,d)


def terminal_options(ctx,qpos,margin_required=.005,motor_fraction_cap=.9):
    d,c=pose_stats(ctx,qpos)
    result={}
    for leg in STANCE:
        other=tuple(k for k in STANCE if k!=leg)
        margin=margin_xy(ctx.model,d,other)
        wrench=wrench_feasibility(ctx.model,d,other)
        result[leg]={
            'margin_mm':float(margin*1000),
            'motor_fraction':wrench.get('min_motor_fraction'),
            'lp_feasible':bool(wrench['feasible']),
            'possible_now_static':bool(margin>=margin_required and wrench['feasible'] and
                         wrench['min_motor_fraction']<=motor_fraction_cap)
        }
    return result


def make_plan(ctx,*,prep_rear_outward_m=.030,prep_rear_forward_m=.030,
              prep_body_recenter_steps=54,middle_inward_m=.030,
              cycle_foot_forward_m=.020,first_rear_cycle_forward_m=.020,
              terminal_margin_m=.005):
    p=StepPreflight(ctx)
    prep_nodes=[]
    def collect(label,pose_ok):
        if not pose_ok['gate_passed']:
            raise RuntimeError(f'{label} blocked: '+str({k:pose_ok[k] for k in ('phase','margin_mm','max_contact_target_error_mm','max_abs_joint_rad','minimum_motor_fraction')}))
    # Preparation begins with claws ALREADY raised: original historical 6→4
    # path exceeded the provisional 1.7rad limit and remains unverified.
    for n in range(12):
        collect('prep_raise',p.checkpoint('PREP_raise',STANCE,base_delta=(0.,0.,.002)))
    for leg in REAR:
        dx=-prep_rear_outward_m if leg=='bl' else prep_rear_outward_m
        status,row=one_leg_waypoints(p,leg,(dx,prep_rear_forward_m))
        if status!='sampled_step_pass':raise RuntimeError(f'PREP {leg}: {status} {row["phase"]}')
    for n in range(prep_body_recenter_steps):
        collect('prep_recenter',p.checkpoint('PREP_recenter',STANCE,base_delta=(0.,-.002,0.)))
    prep_nodes=p.rows.copy()
    d,start_com=pose_stats(ctx,p.c.q)
    if any(margin_xy(ctx.model,d,tuple(k for k in STANCE if k!=leg))<.005 for leg in MIDDLE):
        raise RuntimeError('PREP terminal center cannot lift both middle legs')
    cycle_start_q=p.c.q.copy()
    p.rows=[]
    for leg in MIDDLE:
        delta=(middle_inward_m if leg=='ml' else -middle_inward_m,cycle_foot_forward_m)
        status,row=one_leg_waypoints(p,leg,delta)
        if status!='sampled_step_pass':raise RuntimeError(f'cycle {leg}: {status} {row["phase"]}')
    # Body recenter: follow *future* rear-foot tripods, not current 4 support alone.
    for index in range(40):
        collect('cycle_forward_rebalance',p.checkpoint('CYCLE_forward_pre_rear',STANCE,base_delta=(0.,.002,0.)))
        d,c=pose_stats(ctx,p.c.q)
        if all(margin_xy(ctx.model,d,tuple(k for k in STANCE if k!=leg))>=.006 for leg in REAR):break
    else:raise RuntimeError('Cannot prepare both rear-foot lifts')
    before_rear_shift_count=index+1
    status,row=one_leg_waypoints(p,'bl',(0.,first_rear_cycle_forward_m))
    if status!='sampled_step_pass':raise RuntimeError(f'cycle bl: {status} {row["phase"]}')
    # Predict BR support triangle after placing BL. Sometimes necessary to
    # rebalance even though the preceding BL step was successful.
    for index in range(40):
        d,c=pose_stats(ctx,p.c.q)
        if margin_xy(ctx.model,d,('ml','mr','bl'))>=.006:break
        collect('pre_br_rebalance',p.checkpoint('CYCLE_forward_pre_br',STANCE,base_delta=(0.,.002,0.)))
    else:raise RuntimeError('Cannot prepare last rear-foot liftoff')
    after_rear_shift_count=index
    status,row=one_leg_waypoints(p,'br',(0.,cycle_foot_forward_m))
    if status!='sampled_step_pass':raise RuntimeError(f'cycle br: {status} {row["phase"]}')
    terminal=terminal_options(ctx,p.c.q)
    d,end_com=pose_stats(ctx,p.c.q)
    progress=float(end_com[1]-start_com[1]);min_mar=min(float(x['margin_mm']) for x in p.rows)
    max_frac=max(float(x['minimum_motor_fraction']) for x in p.rows)
    max_q=max(float(x['max_abs_joint_rad']) for x in p.rows)
    min_fz=min(float(x['min_model_contact_Fz_N']) for x in p.rows)
    max_err=max(float(x['max_contact_target_error_mm']) for x in p.rows)
    # 'pass' is only a sampled quasi-static result, NEVER dynamic gait.
    finished=(progress>=.020 and min_mar>=5 and max_frac<=.9 and max_q<=1.700001
              and max_err<.2 and any(t['possible_now_static'] for t in terminal.values()))
    report={
        'schema':'cad30_mink_four_leg_horizon_sampled_v1',
        'status':'FOUR_STEP_SAMPLED_QUASISTATIC_CANDIDATE' if finished else 'RESTRICTED_CANDIDATE_NOT_PASS',
        'physical_gait_validated':False,
        'autonomous_motor_execution_authorized':False,
        'poses_source':ctx.safe_snapshot_source,
        'model':'FNK0052 CAD30 nominal, MuJoCo 3.14.0 + Mink 1.4.0, friction mu=1.1, no servo calibration',
        'prep':{
            'stages':['12x2mm body raise', 'bl outward+forward reposition', 'br outward+forward reposition', f'{prep_body_recenter_steps}x2mm body backward reposition'],
            'extra_preparation_swings':list(REAR),'waypoints':len(prep_nodes),
            'body_recenter_command_mm':float(-prep_body_recenter_steps*2),
            'cycle_start_com_xyz_m':start_com.tolist(),
            'initial_safe_com_xyz_m':pose_stats(ctx,ctx.qpos)[1].tolist(),
            'initial_6_to_4_strict_IK_verified':False,
        },
        'cycle':{
            'swung_legs':['ml','mr','bl','br'],
            'each_foot_advanced_mm':round(cycle_foot_forward_m*1000,3),
            'middle_toe_inward_mm':middle_inward_m*1000,
            'rear_toe_preposition_outward_mm':prep_rear_outward_m*1000,
            'cycle_forward_rebalance_mm':before_rear_shift_count*2,
            'additional_pre_br_balance_mm':after_rear_shift_count*2,
            'waypoints':len(p.rows),
            'COM_forward_progress_from_after_prep_mm':progress*1000,
            'COM_forward_progress_from_original_safe_snapshot_mm':float((end_com[1]-pose_stats(ctx,ctx.qpos)[1][1])*1000),
            'minimum_sampled_support_margin_mm':min_mar,
            'minimum_sampled_model_Fz_N':min_fz,
            'maximum_sampled_motor_fraction':max_frac,
            'maximum_joint_absolute_rad':max_q,
            'maximum_anchored_toe_target_error_mm':max_err,
            'final_com_xyz_m':end_com.tolist(),
            'terminal_immediate_next_step':terminal,
            'terminal_viable_at_least_one_quasistatic':any(t['possible_now_static'] for t in terminal.values()),
        },
        'limitations':[
          'PREPARATION includes two extra rear-foot swings; the counted cycle is the FOUR subsequent ML/MR/BL/BR swings.',
          'Whole-body COM retreats during preparation; progress from original nominal safe pose is NOT positive. Reported cycle progress starts after preparation.',
          'Virtual root translation between kinematic poses is not applied by actuators. No free-base dynamic rollout yet and NO successful hardware gait.',
          'Each point statically passed Mink IK and contact wrench LP; interpolation between sample points, collisions, slip, actual transient Fz and torque NOT checked.',
          'Recorded safe snapshot descended from historical 6→4 transition that transiently exceeded research bound 1.7rad.',
          'Terminal immediate static viability is a necessary snapshot proxy, NOT a viability proof of a second full cycle.',
        ],
    }
    return report,{'prep_waypoints':prep_nodes,'cycle_waypoints':p.rows,
                   'cycle_start_qpos':cycle_start_q.tolist(),
                   'final_qpos':p.c.q.copy().tolist(),
                   'final_footholds_world':{k:v.tolist() for k,v in p.targets.items()}}


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--model',type=Path,required=True);ap.add_argument('--state',type=Path,required=True)
    ap.add_argument('--out',type=Path);ap.add_argument('--trace-out',type=Path)
    args=ap.parse_args()
    ctx=load_context(args.model,args.state)
    receipt,full=make_plan(ctx)
    if args.out:args.out.parent.mkdir(parents=True,exist_ok=True);args.out.write_text(json.dumps(receipt,indent=2)+'\n')
    if args.trace_out:args.trace_out.parent.mkdir(parents=True,exist_ok=True);args.trace_out.write_text(json.dumps(full,separators=(',',':'))+'\n')
    print(json.dumps(receipt,indent=2))

if __name__=='__main__':main()