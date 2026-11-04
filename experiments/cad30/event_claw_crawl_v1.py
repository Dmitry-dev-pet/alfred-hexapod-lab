"""Evidence-first 4-support crawl PROBE on Freenove FNK0052 CAD30 (Issue #21).

Front pair stays in the air. Whenever a leg is to move, a central static
wrench-balance oracle considers the *actual* remaining supports. When no leg
can move, a new body-placement target is computed from the leave-one-out
support triangles, commanded only through the leg joints, and verified in
free-body MuJoCo. No root teleport, pinned feet, scripted leg-order, or claim
of full robot actuator calibration. A rejected experiment remains a failure.
"""
from __future__ import annotations
import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
import sys
import numpy as np
import mujoco

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path[:0]=[str(ROOT),str(HERE)]
from optimized_claw_gait import (
    ALL, FRONT, init_model, actuator_mapping, physical_contact_forces,
    force_oracle, cartesian_shift_paths, sample_ik_path, shift_and_lift_plans,
    optimal_com_target,
)
from support_wrench_qp import support_margin_xy
from hexlab.step_one_leg_v04 import smoothstep, solve_waypoint, floor_contacts
from hexlab.stand_geometry_v04 import tip_world, geom_mesh_world_vertices

SUPPORT=tuple(c for c in ALL if c not in FRONT)


def com_world(model,data):
    return (np.asarray(data.xipos) * np.asarray(model.body_mass)[:,None]).sum(axis=0) / model.body_mass.sum()


def candidate_order(feasible:list[dict], stepped:set[str]):
    """Choose by measured force margin, actuator reserve and gait coverage.

    No hard-coded gait phases; only previously unstepped legs receive a bonus.
    """
    candidates=[c for c in feasible if c['allow'] and abs(c['ik_stride_m'])>.004]
    return sorted(candidates,key=lambda c:(
        c['leg'] not in stepped,
        c['margin_m']-.012*c['torque_fraction'],
        c['ik_stride_m'],c['leg']), reverse=True)


def capped_shift(target_xy,current_xy,max_m=.040):
    d=np.asarray(target_xy)-np.asarray(current_xy)
    n=np.linalg.norm(d)
    return d if n<max_m or n==0 else d*(max_m/n)


def choose_target(model,data,supports, *, reserve=.010, only_legs=None):
    """Rank desired COM shifts for all hypothetical leave-one-out triangles.

    This proposal is purely geometric; physical feasibility is rechecked by
    MuJoCo and LP before every actual liftoff.
    """
    _forces,points=physical_contact_forces(model,data)
    cm=com_world(model,data)
    result=[]
    for c in (tuple(only_legs) if only_legs is not None else SUPPORT):
        others=[j for j in SUPPORT if j!=c]
        if any(points[j] is None for j in others):continue
        pts=np.array([points[j][:2] for j in others])
        try:target=optimal_com_target(cm[:2],pts,margin_m=reserve)
        except (ValueError,RuntimeError):continue
        delta=target-cm[:2]
        result.append({'leg':c,'target_xy':target,'delta_xy':delta,
                       'needed_m':float(np.linalg.norm(delta)),
                       'margin_now_m':float(support_margin_xy(pts,cm[:2]))})
    return sorted(result,key=lambda r:(r['needed_m'],r['leg']))


def make_body_shift_plans(model,data,legs,delta_cm,indices,max_ik_error=.006):
    """Convert proposed COM move into relative foot IK targets.

    A body translation is *never* directly applied to the MuJoCo free root.
    All support toes are commanded opposite to desired COM motion; the ground
    contact response determines the actual displacement.
    """
    initial=data.qpos.copy()
    targets={};errors={}
    for leg in legs:
        c=leg['suffix']
        if c in FRONT:continue
        d=mujoco.MjData(model);d.qpos[:]=initial;d.qvel[:]=0.;mujoco.mj_forward(model,d)
        toe=tip_world(model,d,leg)
        # Empirical 0.85 body-motion ratio from earlier stationary calibration.
        goal=toe+np.r_[-np.asarray(delta_cm)/.85, 0.]
        q,error=solve_waypoint(model,d,mujoco,leg,goal,initial[leg['qadrs']].copy())
        targets[c]=q
        errors[c]=float(error)
    return targets, errors, (max(errors.values())<=max_ik_error)


def make_step(model,data,leg, strides=(.018,.012,.007,-.008,-.015,-.025),lift=.005):
    d=mujoco.MjData(model);d.qpos[:]=data.qpos;d.qvel[:]=0.;mujoco.mj_forward(model,d)
    tip=tip_world(model,d,leg).copy()
    qstart=data.qpos[leg['qadrs']].copy()
    for stride in strides:
        cmds=[];errors=[];prev=qstart
        for dy,dz in ((0,lift),(stride/2,lift),(stride,lift),(stride,0)):
            d.qpos[:]=data.qpos;d.qvel[:]=0.;mujoco.mj_forward(model,d)
            q,error=solve_waypoint(model,d,mujoco,leg,tip+np.array([0,dy,dz]),prev)
            cmds.append(q);errors.append(error);prev=q
        if max(errors)<.006:
            return {'stride_m':stride,'qstart':qstart,'qway':np.asarray(cmds),
                    'max_error_m':max(errors),'start_tip':tip.tolist()}
    return None


def step_cmd(plan,phase):
    # Lift, move forwards at clearance, touch down at the new world point.
    pts=np.vstack((plan['qstart'],plan['qway']))
    # last 35% used for touchdown, to allow sufficiently long support transfer
    boundaries=(0,.22,.48,.62,1.)
    phase=float(np.clip(phase,0,1))
    for i in range(4):
        if phase<=boundaries[i+1]:
            a=smoothstep((phase-boundaries[i])/(boundaries[i+1]-boundaries[i]))
            return pts[i]*(1-a)+pts[i+1]*a
    return pts[-1]


def run(*,seconds=15.5,mu=1.1,shift=.035,seed=101,record=True,step_limit=4):
    model,data,legs=init_model(mu)
    bycode={l['suffix']:l for l in legs}
    qa,da,indices=actuator_mapping(model,data,legs)
    dt=float(model.opt.timestep)
    root=int(model.body('base_link_2').id)
    q0=data.qpos[qa].copy();bounds=model.actuator_ctrlrange.copy()
    init_body=data.xpos[root].copy();up0=data.xmat[root].reshape(3,3).T@np.array([0,0,1.])
    _,_,_,base,_,clawgoals=shift_and_lift_plans(model,data,legs,shift,FRONT,.036)
    prep_paths,errs=cartesian_shift_paths(model,data,legs,FRONT,shift)
    rng=np.random.default_rng(seed)
    assert all(e<.006 for e in errs.values())
    # Continuous foot targets at 4-foot stance, later replaced by feedback-driven plans.
    targets={c:base[indices[c]].copy() for c in ALL}
    lift_approved=False;phase='initial_prep';events=[];trace=[];frames=[]
    active_step=None;stepped=set();successful=[];reposition_attempts=0
    hold_start=None;phase_start=0.0
    com_shift_paths=None;com_shift_from=None;desired_com=None
    max_tau=0.;saturations=0;min_contact=6;below3=0;negative_margin_samples=0
    fall=False;fail_reason=None;oracle_checks=0;steps_veto=0
    stride_m=0.;walk_started=None;walk_ended=None
    sample_every=10;frame_every=50
    tmax=round(seconds/dt)
    for k in range(tmax):
        t=k*dt
        present=floor_contacts(model,data,mujoco)
        if phase=='step' and active_step is not None and active_step['leg'] not in present:
            active_step['off_contact_steps']+=1
        command=base.copy()
        if t<2.:  # prep: all 4 supporting feet move in their local frame
            alpha=smoothstep(t/2.)
            for c in SUPPORT:command[indices[c]]=sample_ik_path(prep_paths[c],alpha)
        else:
            for c in SUPPORT:command[indices[c]]=targets[c]
            if phase=='initial_prep':
                # Save the settled prep targets; check actual physical contacts
                for c in SUPPORT:targets[c]=sample_ik_path(prep_paths[c],1.)
                w=force_oracle(model,data,legs,FRONT,mu)
                oracle_checks+=1;lift_approved=bool(w['feasible'] and w['margin_m']>=.008)
                events.append({'t':round(t,3),'event':'front_lift_oracle','approved':lift_approved,
                               'support_margin_mm':round(w.get('margin_m',0.)*1000,3),
                               'max_motor_fraction':w.get('min_motor_fraction')})
                phase='lift' if lift_approved else 'recover';phase_start=t
            if phase=='lift' and t>=3.25:
                phase='choose';phase_start=t
                events.append({'t':round(t,3),'event':'claws_elevated'})
            if phase=='choose' and t>=phase_start+.20:
                if len(successful)>=step_limit or t>seconds-3.5:
                    phase='recover';phase_start=t;walk_ended=t
                else:
                    eligible=[]
                    for c in SUPPORT:
                        w=force_oracle(model,data,legs,(*FRONT,c),mu)
                        oracle_checks+=1
                        stride_plan=make_step(model,data,bycode[c]) if w.get('feasible') else None
                        eligible.append({'leg':c,'allow':bool(w.get('feasible')
                                     and w.get('margin_m',0)>=.008 and w.get('min_motor_fraction',1)<=.80
                                     and stride_plan is not None and c in present),
                            'margin_m':w.get('margin_m',-10.),
                            'torque_fraction':w.get('min_motor_fraction',1.5),
                            'ik_stride_m':stride_plan['stride_m'] if stride_plan else 0,
                            'plan':stride_plan})
                    events.append({'t':round(t,3),'event':'candidates_checked','choices':
                                   [{k:x[k] for k in ('leg','allow','margin_m','torque_fraction','ik_stride_m')} for x in eligible]})
                    sorted_eligible=candidate_order(eligible,stepped)
                    unstepped=[c for c in SUPPORT if c not in stepped]
                    unstepped_eligible=[e for e in sorted_eligible if e['leg'] in unstepped]
                    if not unstepped_eligible and unstepped and len(stepped)>=2:
                        # Do not masquerade as a gait by endlessly cycling rear legs.
                        sorted_eligible=[]
                    if sorted_eligible:
                        selected=sorted_eligible[0]
                        active_step={'leg':selected['leg'],'plan':selected['plan'],
                                     'started':t,'contacts_initial':set(present),
                                     'start_body_y':float(data.xpos[root,1]),
                                     'off_contact_steps':0,'max_tibia_clearance_m':0.}
                        stepped.add(selected['leg'])
                        if walk_started is None:walk_started=t
                        phase='step';phase_start=t
                        events.append({'t':round(t,3),'event':'swing_approved',
                                       'leg':selected['leg'],'margin_mm':round(selected['margin_m']*1000,3),
                                       'motor_fraction':round(selected['torque_fraction'],3),
                                       'stride_m':selected['plan']['stride_m']})
                    elif reposition_attempts<2 and t<seconds-4.0:
                        proposals=choose_target(model,data,SUPPORT,only_legs=
                                                [c for c in SUPPORT if c not in stepped] if stepped else SUPPORT)
                        com=com_world(model,data)
                        found=None
                        for prop in proposals:
                            dc=capped_shift(prop['delta_xy'],com[:2],max_m=.035)
                            if np.linalg.norm(dc)<.003:continue
                            plan,errors,good=make_body_shift_plans(model,data,legs,dc,indices)
                            if good:
                                found=(prop,plan,dc);break
                        if found:
                            prop,com_shift_paths,dc=found
                            com_shift_from={c:targets[c].copy() for c in SUPPORT}
                            desired_com=(com[:2]+dc).tolist()
                            phase='reposition';phase_start=t;reposition_attempts+=1
                            events.append({'t':round(t,3),'event':'reposition_for_candidate',
                                           'leg':prop['leg'],'delta_cm_m':np.round(dc,5).tolist(),
                                           'margin_before_mm':round(prop['margin_now_m']*1000,2)})
                        else:
                            steps_veto+=1;phase='recover';phase_start=t;walk_ended=t
                            events.append({'t':round(t,3),'event':'no_reachable_support_target'})
                    else:
                        steps_veto+=1;phase='recover';phase_start=t;walk_ended=t
                        events.append({'t':round(t,3),'event':'no_safe_liftoff'})
            if phase=='reposition':
                a=smoothstep(min(1.,(t-phase_start)/1.7))
                for c in SUPPORT:
                    command[indices[c]]=(1-a)*com_shift_from[c]+a*com_shift_paths[c]
                if t>=phase_start+1.7:
                    for c in SUPPORT:targets[c]=com_shift_paths[c].copy()
                    cm=com_world(model,data)
                    events.append({'t':round(t,3),'event':'reposition_measured',
                                   'desired_com_xy':desired_com,
                                   'actual_com_xy':np.round(cm[:2],5).tolist(),
                                   'contacts':sorted(present)})
                    phase='choose';phase_start=t
            if phase=='step':
                c=active_step['leg'];time_delta=t-phase_start
                if k%10==0:
                    z=float(geom_mesh_world_vertices(model,data,bycode[c]['geom'])[:,2].min())
                    active_step['max_tibia_clearance_m']=max(active_step['max_tibia_clearance_m'],z)
                command[indices[c]]=step_cmd(active_step['plan'],time_delta/1.2)
                if time_delta>=1.2:
                    targets[c]=active_step['plan']['qway'][-1].copy()
                    phase='touchdown';phase_start=t
                    events.append({'t':round(t,3),'event':'swing_command_finished','leg':c,
                                   'contact_after':c in present})
            if phase=='touchdown' and t>=phase_start+.42:
                c=active_step['leg']
                contacted=c in present
                verified_liftoff=(active_step['off_contact_steps']>=5 and
                                  active_step['max_tibia_clearance_m']>=.002)
                events.append({'t':round(t,3),'event':'touchdown_checked','leg':c,
                               'contacted':contacted,'verified_liftoff':verified_liftoff,
                               'off_contact_ms':round(active_step['off_contact_steps']*dt*1000,1),
                               'max_tibia_clearance_mm':round(active_step['max_tibia_clearance_m']*1000,2),
                               'body_y_change_m':round(float(data.xpos[root,1]-active_step['start_body_y']),5)})
                successful.append({'leg':c,'touchdown':contacted,
                                   'verified_liftoff':verified_liftoff,'t':t})
                if not contacted:
                    phase='recover';phase_start=t;walk_ended=t
                    fail_reason='swing_no_touchdown'
                else:
                    phase='choose';phase_start=t
                active_step=None
            if phase=='recover':
                # Gradual safe lowering. Full recenter intentionally NOT attempted
                # until all six real contacts return, to avoid PR#18 failure.
                a=1.-smoothstep(min(1.,(t-phase_start)/1.5))
            else:
                a=smoothstep((t-2.15)/1.10) if t<3.25 else 1.0
            if lift_approved:
                for c in FRONT:command[indices[c]]=q0[indices[c]]+a*(clawgoals[c]-q0[indices[c]])
        if t>=seconds-.15 and phase!='recover':
            phase='recover';phase_start=t
        unc=7.*(command-data.qpos[qa])-.32*data.qvel[da]
        saturations+=int(np.any(unc<bounds[:,0]-1e-9) or np.any(unc>bounds[:,1]+1e-9))
        tau=np.clip(unc,bounds[:,0],bounds[:,1])
        max_tau=max(max_tau,float(np.max(np.abs(tau))));data.ctrl[:]=tau
        mujoco.mj_step(model,data)
        active_contacts=floor_contacts(model,data,mujoco)
        min_contact=min(min_contact,len(active_contacts));below3+=int(len(active_contacts)<3)
        if k%sample_every==0 or k==tmax-1:
            cm=com_world(model,data);f,p=physical_contact_forces(model,data)
            actual=[c for c in ALL if p[c] is not None]
            margin=(support_margin_xy(np.array([p[c][:2] for c in actual]),cm[:2])
                    if len(actual)>=3 else -1.)
            negative_margin_samples+=int(margin<0)
            tilt=math.degrees(math.acos(float(np.clip((data.xmat[root].reshape(3,3)@up0)[2],-1,1))))
            trace.append({'t':round(t+dt,3),'phase':phase,'support_count':len(actual),
                          'contacts':actual,'front_contacts':[c for c in FRONT if c in actual],
                          'swing_leg':active_step['leg'] if active_step else None,
                          'body_y_m':round(float(data.xpos[root,1]),5),
                          'forward_since_initial_m':round(float(data.xpos[root,1]-init_body[1]),5),
                          'body_z_m':round(float(data.xpos[root,2]),5),
                          'COM_y_m':round(float(cm[1]),5),
                          'support_margin_mm':round(margin*1000,3),
                          'tilt_deg':round(tilt,3),
                          'max_tau_Nm':round(float(np.max(abs(tau))),4),
                          'normal_fz_N':{c:round(float(f[c][2]),3) for c in ALL}})
            if (tilt>40 or data.xpos[root,2]<.065 or not np.all(np.isfinite(data.qpos))):
                fail_reason='fall_or_invalid';fall=True
                events.append({'t':round(t+dt,3),'event':'physics_abort',
                               'tilt_deg':round(tilt,3),'body_z_m':round(float(data.xpos[root,2]),5)})
                break
        if record and k%frame_every==0:
            frames.append({'t':round(t+dt,3),'qpos':np.round(data.qpos,7).tolist(),
                           'phase':phase})
    active=[r for r in trace if r['phase'] in ('choose','reposition','step','touchdown')]
    front_clear=bool(active) and all(not r['front_contacts'] for r in active)
    finished=bool(not fall and k==tmax-1)
    front_returned=all(c in floor_contacts(model,data,mujoco) for c in FRONT)
    real_steps=[s['leg'] for s in successful if s['touchdown'] and s['verified_liftoff']]
    verified_steps=[{'leg':s['leg'],'t':round(s['t'],3)} for s in successful if s['touchdown'] and s['verified_liftoff']]
    actual_forward=(active[-1]['body_y_m']-active[0]['body_y_m']) if len(active)>1 else 0.
    strict_success=(front_clear and finished and front_returned and
                    set(real_steps)==set(SUPPORT) and len(active)>1 and actual_forward>=.02 and
                    all(r['support_count']>=3 and r['support_margin_mm']>0 for r in active) and
                    saturations==0)
    return {'schema':'FNK0052_event_claw_crawl_v1/1','seed':seed,'mass_kg':round(float(model.body_mass.sum()),6),
            'mu':mu,'assumed_tau_cap_Nm':float(bounds[0,1]),'real_hardware_calibrated':False,
            'uses_central_static_LP_QP':True,'leg_order_preprogrammed':False,
            'seconds_simulated':round(float(data.time-1.),3),'integration_finished':finished,
            'strict_four_leg_walk':strict_success,'reason_if_not_success':None if strict_success else
               (fail_reason or 'full_4-leg_cycle_or_support_margin_not_met'),
            'lift_approved':lift_approved,'front_pair_held_clear':front_clear,
            'front_pair_returned':front_returned,
            'minimum_actual_contacts_2ms':min_contact,'fraction_below_three_2ms':round(below3/max(k+1,1),6),
            'negative_margin_sample_fraction_50Hz':round(negative_margin_samples/max(len(trace),1),6),
            'max_torque_Nm':round(max_tau,5),'saturation_fraction':round(saturations/max(k+1,1),6),
            'steps_attempted':len(successful),
            'commanded_touchdown_legs':[s['leg'] for s in successful if s['touchdown']],
            'verified_liftoff_touchdowns':verified_steps,
            'new_leg_cycle_coverage':len(set(real_steps)),
            'forward_motion_4leg_interval_m':round(actual_forward,5),
            'oracle_calls':oracle_checks,'support_reposition_attempts':reposition_attempts,
            'swing_vetoes':steps_veto,
            'events':events,'trace_50hz':trace,'qpos_10hz':frames,
            'limitations':['Exploratory central quasistatic oracle is not neuroWalknet or a proven stable controller',
                           'Actual moments/friction/mass/servo speed are not calibrated',
                           'Viewer/MP4 MUST use these saved physical qpos only; no staged gait',
                           'Recorded contacts and COM margin are sampled and cannot prove continuous-time safety']}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=15.5)
    parser.add_argument('--mu',type=float,default=1.1)
    parser.add_argument('--seed',type=int,default=101)
    parser.add_argument('--step-limit',type=int,default=4)
    parser.add_argument('--no-frames',action='store_true')
    parser.add_argument('--out',type=Path,default=ROOT/'results/event_claw_crawl_v1_trace.json')
    args=parser.parse_args()
    out=run(seconds=args.seconds,mu=args.mu,seed=args.seed,record=not args.no_frames,
            step_limit=args.step_limit)
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(out,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in out.items() if k not in ('events','trace_50hz','qpos_10hz')},indent=2))
    for evt in out['events']:
        print('EVENT',evt)

if __name__=='__main__':main()