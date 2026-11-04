"""Event-driven contact optimizer for the CAD30 Freenove hexapod.

An exploratory *greedy* contact scheduler, NOT MPC, not neuroWalknet and NOT a
claim of a globally optimal dynamic gait. No fixed tripod/wave leg order.
At each liftoff event we enumerate currently available single/double swing
subsets, verify static equilibrium against real MuJoCo contact locations,
friction and actuator moments, and rank feasible support configurations.
The robot remains a free-body MuJoCo simulation with estimated servos and no
fixed feet/root. Flight targets use measured-geometry IK, not animated body.
"""
from __future__ import annotations
import argparse
from dataclasses import dataclass
import itertools
import json
import math
from pathlib import Path
import sys

import mujoco
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path[:0]=[str(ROOT),str(HERE)]
from support_wrench_qp import support_margin_xy
from optimized_claw_gait import (
    ALL, init_model, physical_contact_forces, force_oracle, actuator_mapping, prepare_sensitivities,
)
from locomotion_cad30 import get_trajectory
from hexlab.step_one_leg_v04 import forward_from_geometry, smoothstep, floor_contacts, solve_waypoint
from hexlab.stand_geometry_v04 import load_legs, tip_world

STRIDES=(.020,.035)
PERIOD=2.4
DUTY=.18
MAX_PARALLEL=2
# This is an experimental safety envelope applied to instantaneous static
# candidates. It is NOT a certificate for the entire dynamic trajectory.
MIN_PREDICTED_MARGIN_M=.010
MAX_MOTOR_FRACTION=.82

@dataclass
class Leg:
    state:str='stance'
    stance_progress:float=0.0
    swing_age:float=0.0
    chosen_stride:float=.035
    liftoffs:int=0
    touchdowns:int=0
    missed_touchdowns:int=0


def sample_cyclic(plan:np.ndarray,phi:float)->np.ndarray:
    value=(float(phi)%1.)*len(plan)
    i=int(math.floor(value));a=value-i
    return (1-a)*plan[i%len(plan)]+a*plan[(i+1)%len(plan)]


def choose_contact_set(candidates:list[dict], *, min_margin_m:float=MIN_PREDICTED_MARGIN_M,
                       max_motor_fraction:float=MAX_MOTOR_FRACTION)->dict|None:
    """Deterministic finite candidate optimizer; never choose an infeasible plan.

    Tradeoff between preserving margin, motor reserve, normal-load spread and
    the number of useful foot relocations. The weights are exploratory and
    must NOT be called a proof of global optimality or of safe dynamics.
    """
    feasible=[]
    for c in candidates:
        if not c.get('feasible',False):continue
        if not np.isfinite(c.get('margin_m',float('nan'))) or c['margin_m']<min_margin_m:continue
        if not np.isfinite(c.get('min_motor_fraction',float('nan'))) or c['min_motor_fraction']>max_motor_fraction:continue
        if not c.get('reachable',False):continue
        count=len(c['new_lifts'])
        spread=c.get('load_std_N',5.)
        advance=c.get('predicted_step_sum_m',0.)
        # All terms dimensionless. Conservative support/motor cost dominates
        # until a multi-leg action has enough reserve to be worthwhile.
        value=(1.8*advance/.035 + .12*count
               - .8*c['min_motor_fraction']
               - .13*spread/3.0
               + .40*min(c['margin_m'],.06)/.06
               - .12*(count-1))
        feasible.append((value,tuple(c['new_lifts']),c))
    return max(feasible,key=lambda x:(x[0],x[1]))[2] if feasible else None


def touchdown_metric(contacts:list[np.ndarray],com:np.ndarray)->float:
    if len(contacts)<3:return float('-inf')
    return support_margin_xy(np.asarray(contacts)[:,:2],com[:2])


def _stance_and_swing_plans(model,data,legs):
    qpos=data.qpos.copy();forward=np.asarray(forward_from_geometry(legs))
    table={};max_errors={}
    for stride in STRIDES:
        plans,errors=get_trajectory(model,qpos,legs,forward,stride,.020,DUTY)
        for leg in legs:
            c=leg['suffix'];table[(c,stride)]=plans[c];max_errors[(c,stride)]=errors[c]
    # The target foot touchdown is at the first stance sample phi≈duty.
    # Compute it in the current world frame for auditing, not as a body command.
    landing={}
    for leg in legs:
        c=leg['suffix']
        for stride in STRIDES:
            tmp=mujoco.MjData(model);tmp.qpos[:]=qpos;tmp.qvel[:]=0.
            tmp.qpos[leg['qadrs']]=sample_cyclic(table[(c,stride)],DUTY)
            mujoco.mj_forward(model,tmp)
            landing[(c,stride)]=tip_world(model,tmp,leg).copy()
    return table,max_errors,landing,forward


def run(seed:int=23,mu:float=1.1,seconds:float=8.,max_parallel:int=2,
        decision_period:float=.10,save_frames:bool=False,force_feedback:bool=False)->dict:
    if not (.2<=mu<=1.5):raise ValueError('friction out of range')
    if max_parallel not in (1,2):raise ValueError('only one/two simultaneous swings')
    model,data,legs=init_model(mu)
    model.opt.timestep=.002
    bycode={l['suffix']:l for l in legs}
    qplan,ik_error,landing,forward=_stance_and_swing_plans(model,data,legs)
    # Same 4-Nov prior-study gain/offset bounds, fixed before A/B comparisons.
    dz_per_m=prepare_sensitivities(model,data,legs) if force_feedback else {}
    foot_z_offset={c:0. for c in ALL}
    force_targets={c:0. for c in ALL}
    target_last_at=None;target_updates=0;force_errors=[]
    qa,da,indices=actuator_mapping(model,data,legs)
    q0=data.qpos[qa].copy()
    root=int(model.body('base_link_2').id)
    start=data.xpos[root].copy();rot0=data.xmat[root].reshape(3,3).copy()
    local_up=rot0.T@np.array([0.,0.,1.])
    rng=np.random.default_rng(seed)
    progress=dict(zip(ALL,rng.uniform(.20,.76,size=6)))
    states={c:Leg(stance_progress=float(progress[c])) for c in ALL}
    bounds=model.actuator_ctrlrange.copy()
    tstart=float(data.time)
    period=float(model.opt.timestep)
    stance_duration=PERIOD*(1-DUTY);swing_duration=PERIOD*DUTY
    decision_every=max(1,int(round(decision_period/period)))
    sample_every=max(1,int(round(.02/period)))
    frame_every=max(1,int(round(.125/period)))
    events=[];trace=[];frames=[]
    scores_evaluated=0;no_feasible_decisions=0;guard_stops=0;num_lt3=0;num_zero=0
    peak_tau=0.;sat=0;fell=False;peak_foot_offset_mm=0.;max_active_swing=0
    previous_selection=None
    min_contacts_2ms=6
    for k in range(round(seconds/period)):
        now=k*period
        contact=floor_contacts(model,data,mujoco)
        # Local per-leg clock/foot contact controls the stance/swing cycle.
        for c,s in states.items():
            if s.state=='stance':
                s.stance_progress=min(1.,s.stance_progress+period/stance_duration)
            else:
                s.swing_age+=period
                if s.swing_age >= swing_duration*.83 and c in contact:
                    s.state='stance';s.stance_progress=0.;s.swing_age=0.;s.touchdowns+=1
                    events.append({'t':round(now,3),'event':'touchdown_observed','leg':c})
                elif s.swing_age>swing_duration+.24 and s.missed_touchdowns==0:
                    s.missed_touchdowns+=1
                    events.append({'t':round(now,3),'event':'touchdown_timeout','leg':c})
        active=tuple(c for c in ALL if states[c].state=='swing')
        max_active_swing=max(max_active_swing,len(active))
        # Refresh the same static wrench oracle when the contact phase changes.
        # It is a desired force allocation, NOT a commanded root acceleration.
        if k%decision_every==0 and active:
            oracle_now=force_oracle(model,data,legs,active,mu)
            if oracle_now.get('feasible',False):
                force_targets.update(oracle_now['fz_targets_N'])
                for c in active:force_targets[c]=0.
                target_last_at=now;target_updates+=1
        # Event-driven global mathematical oracle, not a centralized imposed
        # phase schedule. We only evaluate ready legs when new actions possible.
        if k%decision_every==0 and now>=.58 and len(active)<max_parallel:
            ready=[c for c in ALL if states[c].state=='stance' and
                   states[c].stance_progress>=.99 and c in contact]
            if ready:
                alternatives=[]
                forces_now,points=physical_contact_forces(model,data)
                com_now=(np.asarray(data.xipos)*np.asarray(model.body_mass)[:,None]).sum(axis=0)/model.body_mass.sum()
                cap=max_parallel-len(active)
                for count in range(1,min(cap,len(ready))+1):
                    for chosen in itertools.combinations(ready,count):
                        disabled=tuple(sorted(set(active).union(chosen)))
                        # Current swing legs may be airborne and have no contact,
                        # but all other stance tibias must be in actual contact.
                        oracle=force_oracle(model,data,legs,disabled,mu)
                        scores_evaluated+=1
                        if not oracle.get('feasible',False):
                            alternatives.append({'new_lifts':list(chosen),'feasible':False,
                                                 'reason':oracle.get('reason','infeasible')})
                            continue
                        reachable=True;selected={};dist=0.;margin=float(oracle['margin_m'])
                        for c in chosen:
                            possible=[]
                            for stride in STRIDES:
                                if ik_error[(c,stride)]>.005:continue
                                qmax=float(np.max(np.abs(qplan[(c,stride)])))
                                if qmax>1.72:continue  # UNCALIBRATED optimizer bound
                                foot_after=landing[(c,stride)]
                                if not np.all(np.isfinite(foot_after)):continue
                                # Approximate touchdown stability at today's COM.
                                # Not a forecast: the body will move during swing.
                                cp=[foot_after]+[points[j] for j in ALL if j!=c and j not in disabled and points[j] is not None]
                                margin_after=touchdown_metric(cp,com_now)
                                if margin_after<0:continue
                                scale=stride/max(STRIDES)
                                # Reserve-aware scoring of two finite touchdown
                                # placements. No trained policy, no hand-coded
                                # fore/mid/hind leg order.
                                reward=(scale - 1.25*float(oracle['min_motor_fraction'])*scale*scale
                                        +.10*min(margin_after,.06)/.06
                                        -.05*max(0.,qmax-1.55)/.17)
                                possible.append((reward,stride))
                            if not possible:reachable=False;break
                            stride=max(possible)[1]
                            selected[c]=stride;dist+=stride
                        spread=float(oracle.get('force_balancing',{}).get('target_vertical_force_std_N',5.)
                                     if oracle.get('force_balancing') else 5.)
                        alternatives.append({'new_lifts':list(chosen),'feasible':True,
                            'margin_m':margin,'min_motor_fraction':float(oracle['min_motor_fraction']),
                            'load_std_N':spread,'predicted_step_sum_m':dist,
                            'step_lengths_m':selected,'reachable':reachable,
                            'support_count':len(ALL)-len(disabled),
                            'target_fz_N':oracle.get('fz_targets_N',{})})
                chosen=choose_contact_set(alternatives)
                if chosen is None:
                    no_feasible_decisions+=1
                    # Do not bypass infeasibility with an arbitrary recovery timer.
                    if no_feasible_decisions<=10 or no_feasible_decisions%10==0:
                        events.append({'t':round(now,3),'event':'no_feasible_liftoff',
                                       'ready':ready,'active':list(active),
                                       'alternatives':len(alternatives)})
                else:
                    new_lifts=chosen['new_lifts']
                    for c in new_lifts:
                        s=states[c];s.state='swing';s.swing_age=0.;s.chosen_stride=chosen['step_lengths_m'][c];s.liftoffs+=1
                    previous_selection=chosen
                    force_targets.update(chosen.get('target_fz_N',{}))
                    for c in active+tuple(new_lifts):force_targets[c]=0.
                    target_last_at=now
                    events.append({'t':round(now,3),'event':'optimized_liftoff',
                                   'new_lifts':new_lifts,'active_before':list(active),
                                   'remaining_support_count':chosen['support_count'],
                                   'predicted_margin_mm':round(chosen['margin_m']*1000,3),
                                   'min_motor_fraction':round(chosen['min_motor_fraction'],4),
                                   'predicted_fz_std_N':round(chosen['load_std_N'],3),
                                   'foot_step_m':chosen['step_lengths_m']})
        # Use a task-space IK stance/swing cycle only as the motor primitive;
        # the discrete sequence and touchdown choice were optimized above.
        qtar=q0.copy()
        for c,s in states.items():
            if s.state=='swing':
                phi=DUTY*min(1.,s.swing_age/swing_duration)
            else:
                phi=DUTY+(1-DUTY)*min(.99999,s.stance_progress)
            qtar[indices[c]]=sample_cyclic(qplan[(c,s.chosen_stride)],phi)
        # Bounded foot-height feedback to approach mathematically balanced Fz.
        # This is an *approximate* position-controller wrapper: Fz cannot be
        # imposed exactly, and this must be checked in dynamic MuJoCo results.
        if force_feedback:
            measured,_=physical_contact_forces(model,data)
            for c in ALL:
                if (states[c].state=='stance' and c in contact
                        and force_targets[c]>.05 and target_last_at is not None
                        and now-target_last_at<.5):
                    error=float(force_targets[c]-measured[c][2])
                    desired_delta=float(np.clip(.009*error*period,-.008*period,.008*period))
                    foot_z_offset[c]=float(np.clip(foot_z_offset[c]+desired_delta,-.0018,.0018))
                    if abs(foot_z_offset[c])>1e-9:
                        qtar[indices[c]]+=dz_per_m[c]*foot_z_offset[c]
                else:
                    foot_z_offset[c]*=.98
            peak_foot_offset_mm=max(peak_foot_offset_mm,max(abs(v) for v in foot_z_offset.values())*1000)
        ramp=smoothstep(min(1.,now/.58))
        qtar=q0+ramp*(qtar-q0)
        unbounded=7.*(qtar-data.qpos[qa])-.32*data.qvel[da]
        saturation=np.any(unbounded>bounds[:,1]) or np.any(unbounded<bounds[:,0])
        sat+=int(saturation)
        tau=np.clip(unbounded,bounds[:,0],bounds[:,1]);peak_tau=max(peak_tau,float(np.max(np.abs(tau))))
        data.ctrl[:]=tau
        mujoco.mj_step(model,data)
        contacts=floor_contacts(model,data,mujoco)
        num_lt3+=int(len(contacts)<3);num_zero+=int(len(contacts)==0)
        min_contacts_2ms=min(min_contacts_2ms,len(contacts))
        if k%sample_every==0 or k==round(seconds/period)-1:
            com=(np.asarray(data.xipos)*np.asarray(model.body_mass)[:,None]).sum(axis=0)/model.body_mass.sum()
            f,p=physical_contact_forces(model,data)
            support_force_errors=[abs(f[c][2]-force_targets[c]) for c in ALL
                if force_targets[c]>.05 and states[c].state=='stance' and p[c] is not None]
            if support_force_errors:force_errors.extend(support_force_errors)
            present=[c for c in ALL if p[c] is not None]
            margin=touchdown_metric([p[c] for c in present],com) if len(present)>=3 else float('-inf')
            base=data.xpos[root].copy()
            tilt=math.degrees(math.acos(float(np.clip((data.xmat[root].reshape(3,3)@local_up)[2],-1,1))))
            trace.append({'t':round(now+period,3),'advance_cm':round(100*float((base[:2]-start[:2])@forward),4),
                          'lateral_drift_cm':round(100*float((base[:2]-start[:2])@np.array([-forward[1],forward[0]])),4),
                          'contacts':present,'n_contacts':len(present),
                          'support_margin_mm':round(float(margin*1000),3) if np.isfinite(margin) else None,
                          'tilt_deg':round(tilt,3),'body_z_m':round(float(base[2]),6),
                          'tau_max_nm':round(float(np.max(abs(tau))),4),
                          'normal_forces_N':{c:round(float(f[c][2]),3) for c in ALL},
                          'swinging':[c for c in ALL if states[c].state=='swing'],
                          'force_target_errors_N':round(float(np.mean(support_force_errors)),4)
                               if support_force_errors else None,
                          'force_targets_N':{c:round(force_targets[c],3) for c in ALL},
                          'foot_z_offset_mm':{c:round(1000*foot_z_offset[c],3) for c in ALL}})
            if base[2]<.075 or tilt>50 or not np.all(np.isfinite(data.qpos)):
                fell=True;events.append({'t':round(now+period,3),'event':'fall_guard'});break
        if save_frames and (k%frame_every==0 or k==round(seconds/period)-1):
            frames.append({'t':round(now+period,3),'qpos':np.round(data.qpos,7).tolist()})
    return {'schema':'fnk0052_CAD30_event_contact_planner/1','seed':seed,'mu':mu,
      'model_mass_kg':round(float(model.body_mass.sum()),6),'control':'greedy_event_contact_oracle',
      'no_preassigned_gait_phase_order':True,'uses_full_body_math_oracle':True,
      'not_MPC':True,'global_dynamic_optimality_proven':False,
      'max_parallel_swing':max_parallel,'force_feedback':force_feedback,
      'force_oracle_target_updates':target_updates,
      'mean_target_force_abs_error_N':round(float(np.mean(force_errors)),4) if force_errors else None,
      'max_abs_foot_z_offset_mm':round(peak_foot_offset_mm,3),
      'fixed_IK_strides_tested_m':list(STRIDES),
      'nominal_step_lift_m':.020,'nominal_foot_cycle_s':PERIOD,
      'step_selection':'finite reachable foot placements, torque-reserve and estimated post-contact COM-margin score',
      'decision_period_s':decision_period,'static_motor_cap_assumed_Nm':.7,
      'min_candidate_margin_m':MIN_PREDICTED_MARGIN_M,
      'max_candidate_motor_fraction':MAX_MOTOR_FRACTION,
      'duration_s':round(float(data.time-tstart),3),
      'finished':bool(not fell and k+1==round(seconds/period)),
      'advance_cm':trace[-1]['advance_cm'] if trace else None,
      'min_contacts_2ms':min_contacts_2ms,
      'maximum_simultaneous_swing_legs':max_active_swing,
      'fraction_negative_support_margin_sampled_50hz':round(
         sum(x['support_margin_mm'] is None or x['support_margin_mm']<0 for x in trace)/max(1,len(trace)),6),
      'single_or_double_liftoffs':{str(i):sum(e['event']=='optimized_liftoff' and len(e['new_lifts'])==i for e in events) for i in (1,2)},
      'fraction_below3_2ms':round(num_lt3/max(1,k+1),6),
      'fraction_zero_2ms':round(num_zero/max(1,k+1),6),
      'min_sampled_support_margin_mm':min((x['support_margin_mm'] for x in trace if x['support_margin_mm'] is not None),default=None),
      'peak_tau_nm':round(peak_tau,5),'motor_saturation_fraction':round(sat/max(1,k+1),6),
      'liftoffs':sum(s.liftoffs for s in states.values()),
      'touchdowns':sum(s.touchdowns for s in states.values()),
      'foot_relocations_by_leg':{c:s.liftoffs for c,s in states.items()},
      'candidate_oracle_checks':scores_evaluated,'infeasible_decision_events':no_feasible_decisions,
      'fall_guard_triggered':fell,'events':events,'trace_50hz':trace,'qpos_frames':frames,
      'limitations':['Single-position static LP/QP is NOT a dynamic stability certificate',
        'Foot target choice is finite among 20mm/35mm IK primitives, not continuous optimal placement',
        'Motor torques, friction, masses and contact pad are uncalibrated',
        'Momentum and future touchdown are not optimized over a horizon',
        'This is centralized contact choice, not local self-organized biological control']}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seed',type=int,default=23)
    p.add_argument('--mu',type=float,default=1.1)
    p.add_argument('--seconds',type=float,default=8.)
    p.add_argument('--max-parallel',type=int,default=2)
    p.add_argument('--force-feedback',action='store_true')
    p.add_argument('--save-frames',action='store_true')
    p.add_argument('--out',type=Path,default=None)
    args=p.parse_args()
    result=run(seed=args.seed,mu=args.mu,seconds=args.seconds,
               max_parallel=args.max_parallel,save_frames=args.save_frames,
               force_feedback=args.force_feedback)
    if args.out:
        args.out.parent.mkdir(parents=True,exist_ok=True)
        args.out.write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('events','trace_50hz','qpos_frames')},indent=2))

if __name__=='__main__':main()