"""Physical CAD30 short two-support excursion + four-support recovery.

Authoritative object is the genuine free-base MuJoCo qpos/qvel/contact stream;
no frames, contacts or body motion are staged. The optimizers use a manually
selected contact pair ml/mr and fixed QP-to-PD phase duration: this is NOT an
MPC gait, not a guaranteed safe strategy or a hardware controller.

Requires private Freenove CAD30 XML/STL and archived rebalance pose B, plus
upstream Mink, scipy, qpsolvers/DAQP and MuJoCo. No third-party CAD in repo.
"""
from __future__ import annotations
import argparse
import json
import math
from collections import Counter
from pathlib import Path

import mink
import mujoco as mj
import numpy as np

from dynamic_support_qp import dynamic_qp
from mink_contact_preflight import StepPreflight
from mink_contact_preflight import load_context, StepPreflight, STANCE, PREFIX, com_xyz

ALL_LEGS = (*STANCE, 'fl', 'fr')
THRESHOLD_FZ_N = 0.08
RECONTACT_RESERVE_FZ_N = 0.2
DT = 0.002


def actual_leg_contacts(model, data):
    """Aggregate measured normal reaction and floor-pair count by tibia."""
    floor = int(model.geom('floor').id)
    geom_leg = {int(model.geom(f'tibia{k}_collision_0').id): k for k in ALL_LEGS}
    fz = {k: 0. for k in ALL_LEGS}
    pairs = {k: 0 for k in ALL_LEGS}
    for i in range(data.ncon):
        c = data.contact[i]
        gid = (int(c.geom2) if int(c.geom1)==floor else
               int(c.geom1) if int(c.geom2)==floor else -1)
        if gid not in geom_leg:
            continue
        k = geom_leg[gid]
        w = np.zeros(6)
        mj.mj_contactForce(model, data, i, w)
        world = np.asarray(c.frame).reshape(3,3).T @ w[:3]
        fz[k] += max(0.,float(world[2]))
        pairs[k] += 1
    return fz, pairs


def plan_prebalance(ctx, shift_mm=12, requested_rear_raise_mm=5.):
    """Same established Mink + four-contact static checkpoints as PR #34.

    Returns joint targets from virtual poses; body is never teleported during
    subsequent physical rollout. Failure is explicit, not an instruction.
    """
    if shift_mm < 0 or int(shift_mm)%2 or shift_mm>12:
        raise ValueError('Shift must be 0,2,...,12 mm within audited range')
    planner = StepPreflight(ctx)
    states=[]
    for i in range(int(shift_mm)//2):
        node = planner.checkpoint(f'forward_{(i+1)*2}mm', STANCE,base_delta=(0.,.002,0.))
        if not node['gate_passed']:
            return None, {'reason':'FOUR_SUPPORT_KINEMATIC_PREFLIGHT_BLOCK', 'node':node['phase']}
        states.append(np.asarray(planner.c.q).copy())
    return states,None


def run_probe(model_path, saved_state_path, *, shift_mm=12, qp_duration_s=.12,
              swing_accel_m_s2=4., return_target_mm=0, return_ramp_s=.6,
              rear_advance_mm=0, landing_body_shift_mm=0, forward_accel=0., forward_weight=0.,
              soft_joint_gain=0., soft_joint_threshold=1.66, soft_joint_damping=0.,
              record_physics_steps=False,
              final_hold_s=.5, mu=1.1, sample_stride=10, record_qpos=True):
    """Controlled, short QP experiment with physical contact measurement.

    QP on 2 hypothesized middle feet re-solved 100 Hz, physics 500 Hz; lift
    switched off BEFORE future infeasibility. Final PD returns to a prior
    stable commanded four-support posture; does not produce a walking cycle.
    """
    if qp_duration_s<=0 or qp_duration_s>.25:
        raise ValueError('Time-limited prototype, cap QP phase at 0.25s')
    if return_target_mm not in (0,2,4,6,8,10,12) or return_target_mm>shift_mm:
        raise ValueError('Return target must be an earlier 2-mm node')
    if not (0<=soft_joint_gain<=20 and 1.6<=soft_joint_threshold<=1.7 and 0<=soft_joint_damping<=1):
        raise ValueError('Soft-stop setting out of research bounds')
    if not (0<mu<=2 and 0<swing_accel_m_s2<=8):
        raise ValueError('Bounded μ and swing acceleration required')
    ctx=load_context(Path(model_path),Path(saved_state_path))
    model=ctx.model
    model.opt.timestep=DT
    floor=int(model.geom('floor').id)
    model.geom_friction[floor,0]=mu
    for k in ALL_LEGS:
        model.geom_friction[model.geom(f'tibia{k}_collision_0').id,0]=mu
    trajectory,error=plan_prebalance(ctx,shift_mm)
    if error:return {'status':'PREPLAN_BLOCKED', 'error':error}
    # G2: certify a feasible *future touchdown* before any physical step.
    landing_goal_qpos=None;landing_preflight=None
    if rear_advance_mm:
        if rear_advance_mm not in (5,10,20):raise ValueError('bounded stride 5/10/20 mm')
        if landing_body_shift_mm not in (0,2,4,6,8,10,12):raise ValueError('bounded shift 0..12mm')
        landing_planner=StepPreflight(ctx)
        for leg in ('bl','br'):
            landing_planner.targets[leg][1]+=rear_advance_mm/1000.
        landing_preflight=landing_planner.checkpoint('candidate_landing',STANCE,
                        base_delta=(0.,landing_body_shift_mm/1000.,0.))
        if not landing_preflight['gate_passed']:
            return {'status':'TERMINAL_PREFLIGHT_BLOCKED',
                    'candidate_stride_mm':rear_advance_mm,
                    'candidate_body_shift_mm':landing_body_shift_mm,
                    'landing_preflight':{k:v for k,v in landing_preflight.items() if k!='qpos'}}
        landing_goal_qpos=np.array(landing_preflight['qpos'])
    d=mj.MjData(model);d.qpos[:]=ctx.qpos;d.qvel[:]=ctx.qvel;mj.mj_forward(model,d)
    motor_joints=model.actuator_trnid[:,0].astype(int)
    qadr=model.jnt_qposadr[motor_joints].astype(int)
    dadr=model.jnt_dofadr[motor_joints].astype(int)
    caps=np.maximum(abs(model.actuator_ctrlrange[:,0]),abs(model.actuator_ctrlrange[:,1]))
    goal=d.qpos[qadr].copy()
    root_q0=d.qpos[3:7].copy()
    initial_com=com_xyz(model,d).copy()
    site_ids={k:int(model.site(PREFIX+k).id) for k in STANCE}
    feet_initial_z={k:float(d.site_xpos[site_ids[k]][2]) for k in STANCE}
    feet_initial_y={k:float(d.site_xpos[site_ids[k]][1]) for k in STANCE}
    max_both_rear_forward_mm=-1e5
    last_qualified_forward_streak=0;max_qualified_forward_streak=0
    physics_steps=[]
    max_both_rear_clearance_mm=-1e5
    qualified_forward_steps=0
    max_simultaneous_rear_delta_mm={k:-1e5 for k in ('bl','br')}
    farthest_qualified_two_support_y={k:-1e5 for k in ('bl','br')}
    final_rear_positions={}
    last_stance_xy={k:d.site_xpos[site_ids[k],:2].copy() for k in ('ml','mr')}
    slide_path_mm={k:0. for k in ('ml','mr')}
    contact_counts={};trace=[];motion=[];end_4_reserve=[]
    max_tau=0.;max_joint=0.;max_quat_tilt=0.;min_body_z=float('inf')
    max_joint_info={}
    raw_torque_clip_steps=0;under_two_steps=0;qualified_steps=0;longest_qualified_steps=0;qualified_streak=0
    first_qualified=None;last_qualified=None;QP_calls=0;QP_failed=False

    def observe(phase):
        nonlocal max_tau,max_joint,max_quat_tilt,min_body_z,under_two_steps,max_joint_info
        nonlocal qualified_steps,longest_qualified_steps,qualified_streak,first_qualified,last_qualified
        nonlocal max_both_rear_forward_mm,max_both_rear_clearance_mm,qualified_forward_steps
        nonlocal last_qualified_forward_streak,max_qualified_forward_streak
        fz,pair=actual_leg_contacts(model,d)
        loaded=[k for k in STANCE if fz[k]>THRESHOLD_FZ_N]
        n=len(loaded);contact_counts.setdefault(phase,Counter())[n]+=1
        under_two_steps+=int(n<2)
        max_tau=max(max_tau,float(np.max(abs(d.ctrl))))
        angles=np.asarray(d.qpos[ctx.joint_qpos_addrs])
        jnum=int(np.argmax(abs(angles)))
        if abs(angles[jnum])>max_joint:
            max_joint=float(abs(angles[jnum]))
            qaddr=int(ctx.joint_qpos_addrs[jnum]);jid=int(np.flatnonzero(model.jnt_qposadr==qaddr)[0])
            did=int(model.jnt_dofadr[jid])
            max_joint_info={'phase':phase,'time_s':float(d.time),'joint_name':model.joint(jid).name,
                            'value_rad':float(angles[jnum]),'velocity_rad_s':float(d.qvel[did]),
                            'joint_id':jid,'qpos_address':qaddr}
        dot=abs(float(np.dot(d.qpos[3:7],root_q0)))
        tilt=math.degrees(2*math.acos(np.clip(dot,-1.,1.)))
        max_quat_tilt=max(max_quat_tilt,tilt)
        min_body_z=min(min_body_z,float(d.qpos[2]))
        foot_up_mm={k:float((d.site_xpos[site_ids[k]][2]-feet_initial_z[k])*1000) for k in STANCE}
        if phase in ('two_support_QP','recontact_PD'):
            for k in ('ml','mr'):
                xy=d.site_xpos[site_ids[k],:2].copy()
                if fz[k]>.08:slide_path_mm[k]+=float(np.linalg.norm(xy-last_stance_xy[k])*1000)
                last_stance_xy[k]=xy
        if phase=='final_hold':
            end_4_reserve.append(all(fz[k]>=RECONTACT_RESERVE_FZ_N for k in STANCE))
        if phase in ('two_support_QP','recontact_PD'):
            reary={k:float((d.site_xpos[site_ids[k]][1]-feet_initial_y[k])*1000) for k in ('bl','br')}
            max_both_rear_forward_mm=max(max_both_rear_forward_mm,min(reary.values()))
            max_both_rear_clearance_mm=max(max_both_rear_clearance_mm,min(foot_up_mm[k] for k in ('bl','br')))
            for k in ('bl','br'):
                max_simultaneous_rear_delta_mm[k]=max(max_simultaneous_rear_delta_mm[k],reary[k])
        qualified=(set(loaded)=={'ml','mr'} and all(fz[k]<.08 for k in ('bl','br'))
                   and all(pair[k]==0 for k in ('bl','br'))
                   and all(foot_up_mm[k]>=2 for k in ('bl','br')))
        if qualified:
            if all((d.site_xpos[site_ids[k]][1]-feet_initial_y[k])>0.002 for k in ('bl','br')):
                qualified_forward_steps+=1
                last_qualified_forward_streak+=1
                max_qualified_forward_streak=max(max_qualified_forward_streak,last_qualified_forward_streak)
                for k in ('bl','br'):
                    farthest_qualified_two_support_y[k]=max(farthest_qualified_two_support_y[k],float((d.site_xpos[site_ids[k]][1]-feet_initial_y[k])*1000))
            if not all((d.site_xpos[site_ids[k]][1]-feet_initial_y[k])>0.002 for k in ('bl','br')):
                last_qualified_forward_streak=0
            qualified_steps+=1;qualified_streak+=1
            longest_qualified_steps=max(longest_qualified_steps,qualified_streak)
            if first_qualified is None:first_qualified=round(float(d.time),4)
            last_qualified=round(float(d.time),4)
        else:
            qualified_streak=0;last_qualified_forward_streak=0
        if record_physics_steps:
            physics_steps.append({
                't_s':round(float(d.time),4),'phase':phase,
                'loaded':loaded,
                'Fz_N':{k:round(float(fz[k]),5) for k in STANCE},
                'pairs':{k:int(pair[k]) for k in STANCE},
                'rear_site_y_delta_mm':{k:round(float((d.site_xpos[site_ids[k]][1]-feet_initial_y[k])*1000),5) for k in ('bl','br')},
                'rear_site_z_delta_mm':{k:round(float(foot_up_mm[k]),5) for k in ('bl','br')},
                'COM_y_mm':round(float(com_xyz(model,d)[1])*1000,5),
                'max_motor_tau_Nm':round(float(np.max(abs(d.ctrl))),5),
                'max_abs_leg_joint_rad':round(float(np.max(abs(d.qpos[ctx.joint_qpos_addrs]))),6),
                'two_support_with_real_rear_clearance':qualified,
            })
        if int(round(d.time/DT))%sample_stride==0:
            sample={'t':round(float(d.time),4),'phase':phase,'loaded':loaded,
                    'Fz_N':{k:round(float(v),5) for k,v in fz.items()},
                    'floor_contact_pairs':pair,
                    'rear_clearance_mm':{k:round(foot_up_mm[k],5) for k in ('bl','br')},
                    'rear_forward_mm':{k:round(float((d.site_xpos[site_ids[k]][1]-feet_initial_y[k])*1000),5) for k in ('bl','br')},
                    'body_tilt_delta_deg':round(float(tilt),4),
                    'COM_y_mm':round(float(com_xyz(model,d)[1])*1000,5),
                    'max_torque_Nm':round(float(np.max(abs(d.ctrl))),5),
                    'genuine_two_support':qualified}
            trace.append(sample)
            if record_qpos:
                motion.append({**sample,'qpos':np.asarray(d.qpos).round(10).tolist(),
                               'qvel':np.asarray(d.qvel).round(9).tolist()})

    def pd_step(target,phase,blend=1.,prior=None):
        nonlocal raw_torque_clip_steps
        raw=7*(target-d.qpos[qadr])-.32*d.qvel[dadr]
        if prior is not None:raw=(1-blend)*prior+blend*raw
        if soft_joint_gain and phase in ('recontact_PD','final_hold'):
            monitored=set(int(q) for q in ctx.joint_qpos_addrs)
            for i,jq in enumerate(qadr):
                if int(jq) not in monitored:continue
                qi=float(d.qpos[jq]);direction=np.sign(qi)
                depth=max(0.,abs(qi)-soft_joint_threshold)
                if depth>0.:
                    outward=max(0.,direction*float(d.qvel[dadr[i]]))
                    raw[i]-=direction*(soft_joint_gain*depth+soft_joint_damping*outward)
        raw_torque_clip_steps+=int(np.any(abs(raw)>caps+1e-8))
        d.ctrl[:]=np.clip(raw,-caps,caps);mj.mj_step(model,d);observe(phase)

    def hold(seconds,phase):
        for _ in range(round(seconds/DT)):pd_step(goal,phase)

    def ramp(end,seconds,phase,prior_tau=None):
        nonlocal goal
        old=goal.copy();N=round(seconds/DT)
        for i in range(N):
            f=(i+1)/N;f=f*f*(3-2*f)
            pd_step(old+(end-old)*f,phase,blend=f,prior=prior_tau)
        goal=end.copy()

    hold(.3,'initial_hold')
    for virtual in trajectory:
        ramp(virtual[qadr],.3,'prebalance')
    hold(.5,'after_prebalance')
    prepared_COM=com_xyz(model,d).copy();prepared_fz,_=actual_leg_contacts(model,d)
    reference_quat=d.qpos[3:7].copy()
    QP_solution=None
    for i in range(round(qp_duration_s/DT)):
        if i%5==0:
            quat_delta=np.zeros(3)
            mj.mju_subQuat(quat_delta,d.qpos[3:7],reference_quat)
            correction=np.clip(-.8*quat_delta-.5*d.qvel[3:6],-4.,4.)
            QP_solution=dynamic_qp(model,d,site_ids,('ml','mr'),mu=mu,
                       swing_accel=swing_accel_m_s2,angular_target=correction,
                       angular_accel_cap=10.,linear_accel_cap=1.5,
                       joint_accel_cap=35.,torque_util=.9,prev_tau=d.ctrl.copy(),
                       forward_accel=forward_accel,forward_weight=forward_weight)
            QP_calls+=1
            if QP_solution is None:
                QP_failed=True;break
        d.ctrl[:]=QP_solution['tau']
        mj.mj_step(model,d);observe('two_support_QP')
        if under_two_steps or max_quat_tilt>12 or max_joint>1.7001 or d.qpos[2]<.09:
            break
    after_qp_COM=com_xyz(model,d).copy();after_qp_fz,_=actual_leg_contacts(model,d)
    # Physically commanded return; no root pose manipulation during mj_step.
    prev_tau=d.ctrl.copy()
    if landing_goal_qpos is not None:old_goal=landing_goal_qpos[qadr].copy()
    elif return_target_mm==0:old_goal=ctx.qpos[qadr].copy()
    else:old_goal=trajectory[return_target_mm//2-1][qadr].copy()
    ramp(old_goal,return_ramp_s,'recontact_PD',prior_tau=prev_tau)
    hold(final_hold_s,'final_hold')
    final_fz,final_contacts=actual_leg_contacts(model,d)
    final_COM=com_xyz(model,d).copy()
    final_rear_positions={k:{'forward_from_initial_mm':float((d.site_xpos[site_ids[k]][1]-feet_initial_y[k])*1000),
                             'site_z_delta_mm':float((d.site_xpos[site_ids[k]][2]-feet_initial_z[k])*1000),
                             'Fz_N':float(final_fz[k]), 'floor_pairs':int(final_contacts[k])}
                         for k in ('bl','br')}
    final_strong=len(end_4_reserve)>=100 and all(end_4_reserve[-100:])
    passed=(longest_qualified_steps>=25 and final_strong and not QP_failed
            and under_two_steps==0 and raw_torque_clip_steps==0
            and max_joint<=1.7001 and max_tau<=float(max(caps))+1e-9
            and max_quat_tilt<=8 and min_body_z>=.09)
    rear_forward_min=min(x['forward_from_initial_mm'] for x in final_rear_positions.values())
    actual_rear_touchdown=(all(v['Fz_N']>=RECONTACT_RESERVE_FZ_N and v['floor_pairs']>=1 for v in final_rear_positions.values()))
    partial_step_pass=bool(
        rear_advance_mm and passed and qualified_forward_steps>=25
        and max_qualified_forward_streak>=25
        and rear_forward_min>=5.0 and actual_rear_touchdown
    )
    reached_20mm=bool(partial_step_pass and (final_COM[1]-initial_com[1])>=.020)
    if rear_advance_mm:
        status=('ONE_REAR_PAIR_FORWARD_STEP_GE20MM' if reached_20mm else
                'PARTIAL_REAR_PAIR_FORWARD_STEP_LT20MM' if partial_step_pass else
                'RESEARCH_GATE_FAILED')
    else:status='TRANSIENT_TWO_SUPPORT_AND_FOUR_SUPPORT_RECOVERED' if passed else 'RESEARCH_GATE_FAILED'
    return {
        'schema':'cad30_forward_foot_candidate_2support_v1',
        'status':status,
        'partial_rear_pair_step_pass':partial_step_pass,
        'target_20mm_net_forward_pass':reached_20mm,
        'terminal_rear_touchdown_verified':actual_rear_touchdown,
        'longest_forward_airborne_continuous_seconds':round(max_qualified_forward_streak*DT,4),
        'is_full_walk':False,'is_full_MPC':False,'hardware_verified':False,
        'candidate_stride_mm':rear_advance_mm,
        'candidate_body_shift_mm':landing_body_shift_mm,
        'forward_accel_m_s2':forward_accel,'forward_weight':forward_weight,
        'soft_joint_gain':soft_joint_gain,'soft_joint_threshold':soft_joint_threshold,'soft_joint_damping':soft_joint_damping,
        'landing_preflight':{k:v for k,v in (landing_preflight or {}).items() if k!='qpos'},
        'final_rear_footholds':final_rear_positions,
        'max_both_rear_forward_mm':max_both_rear_forward_mm,
        'max_both_rear_clearance_mm':max_both_rear_clearance_mm,
        'qualified_forward_physics_steps':qualified_forward_steps,
        'farthest_forward_during_true_2support_mm':farthest_qualified_two_support_y,
        'net_forward_COM_mm':float((final_COM[1]-initial_com[1])*1000),
        'parameters':{'virtual_prebalance_mm':shift_mm,'qp_requested_seconds':qp_duration_s,
             'swing_up_accel_m_s2':swing_accel_m_s2,'return_target_mm':return_target_mm,
             'return_ramp_s':return_ramp_s,'final_hold_s':final_hold_s,'mu':mu,
             'sample_stride_steps':sample_stride,'dt_seconds':DT,'Fz_loaded_threshold_N':THRESHOLD_FZ_N,
             'strong_return_Fz_threshold_N':RECONTACT_RESERVE_FZ_N,'servo_PD':[7,.32],
             'provisional_joint_bound_rad':1.7,'QP_torque_utilization_cap':.9},
        'total_time_s':round(float(d.time),4),
        'initial_COM_m':np.round(initial_com,8).tolist(),
        'prepared_COM_m':np.round(prepared_COM,8).tolist(),
        'after_QP_COM_m':np.round(after_qp_COM,8).tolist(),
        'final_COM_m':np.round(final_COM,8).tolist(),
        'prepared_Fz_N':prepared_fz,'after_QP_Fz_N':after_qp_fz,
        'final_Fz_N':final_fz,'final_floor_contact_pairs':final_contacts,
        'actual_qualified_two_support_seconds':round(qualified_steps*DT,4),
        'longest_contiguous_true_two_support_seconds':round(longest_qualified_steps*DT,4),
        'first_qualified_t_s':first_qualified,'last_qualified_t_s':last_qualified,
        'last_0p2s_all_four_Fz_ge_0p2N':final_strong,
        'last_0p2s_four_reserve_steps':sum(end_4_reserve[-100:]),
        'contact_histogram_by_phase_2ms':{phase:{str(k):int(v) for k,v in counts.items()} for phase,counts in contact_counts.items()},
        'max_motor_tau_Nm':round(float(max_tau),6),
        'clipped_PD_physics_steps':int(raw_torque_clip_steps),
        'max_joint_abs_rad':round(float(max_joint),6),
        'max_joint_info':max_joint_info,
        'max_quaternion_orientation_change_deg':round(float(max_quat_tilt),5),
        'min_base_height_m':round(float(min_body_z),6),
        'support_foot_horizontal_path_mm':{k:round(float(v),5) for k,v in slide_path_mm.items()},
        'below_two_support_steps':int(under_two_steps), 'QP_calls':int(QP_calls),
        'QP_solver_failed':bool(QP_failed),
        'notes':[
            'Only a timed two-support excursion and return, not a repeated step cycle or predictive controller.',
            'Fixed nominal CAD30: arm torque/velocity/friction/mass estimates uncalibrated for physical hardware.',
            'Initial archived safe B pose itself follows an earlier historical 6->4 trajectory with unvalidated joint travel.',
            'Actual floor contact pairs + Fz measured on every 2ms simulation step; viewer samples are decimated.',
            'Finite-difference Jdot*qdot, one force point per tibia and linearized friction cone are QP approximations.',
            'Two-op feet lift is observed, not scripted; no advancing foothold placements, terminal viability for long-term gait unproven.'
        ],
        'physics_steps_2ms':physics_steps if record_physics_steps else None,
        'trace_20ms':trace,
        'qpos_samples_20ms':motion if record_qpos else None,
    }


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--model',type=Path,required=True)
    p.add_argument('--state',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--mu',type=float,default=1.1)
    p.add_argument('--qp-duration',type=float,default=.12)
    p.add_argument('--return-mm',type=int,default=0)
    p.add_argument('--rear-advance-mm',type=int,default=0,choices=[0,5,10,20])
    p.add_argument('--landing-body-shift-mm',type=int,default=0,choices=[0,2,4,6,8,10,12])
    p.add_argument('--forward-accel',type=float,default=0.)
    p.add_argument('--forward-weight',type=float,default=0.)
    p.add_argument('--return-ramp-s',type=float,default=.6)
    p.add_argument('--final-hold-s',type=float,default=.5)
    p.add_argument('--soft-joint-gain',type=float,default=0.)
    p.add_argument('--soft-joint-threshold',type=float,default=1.66)
    p.add_argument('--record-physics-2ms',action='store_true')
    p.add_argument('--without-qpos',action='store_true')
    args=p.parse_args()
    result=run_probe(args.model,args.state,mu=args.mu,qp_duration_s=args.qp_duration,
                     return_target_mm=args.return_mm,record_qpos=not args.without_qpos,
                     rear_advance_mm=args.rear_advance_mm,
                     landing_body_shift_mm=args.landing_body_shift_mm,
                     forward_accel=args.forward_accel,forward_weight=args.forward_weight,
                     return_ramp_s=args.return_ramp_s,final_hold_s=args.final_hold_s,
                     soft_joint_gain=args.soft_joint_gain,
                     soft_joint_threshold=args.soft_joint_threshold,
                     record_physics_steps=args.record_physics_2ms)
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('trace_20ms','qpos_samples_20ms','physics_steps_2ms')},indent=2,ensure_ascii=False))

if __name__=='__main__':main()
