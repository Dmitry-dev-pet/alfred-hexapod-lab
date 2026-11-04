"""Pre-liftoff local closed-loop force transfer for FNK0052 CAD30 MuJoCo.

Compare an unchanged existing baseline with a NEW controller variant that
prepares for liftoff by asking up to two neighbors to actively unload a leg.

Based on original active_load_transfer.py; scientifically exploratory.

ORIGINAL: Active neighbor-supported *local* load-transfer ablation on the FNK0052 CAD30 MuJoCo model.

Compare same six independent leg clocks and neighbor inhibition with/without
contact NORMAL FORCE feedback. Force is reconstructed from the MuJoCo contact
solver (mj_contactForce) rather than a physical transducer. The extra policy
makes decisions using only this leg's force and its two adjacent legs' forces,
without global foot-load sums, central phase clocks, or hardcoded tripod order.

Scientific limitations: assumed CAD mass/friction/servo model; contact hull is
convex and loads have no noise/delay other than an explicitly modeled EMA.
This is a diagnostic local-rule heuristic and NOT neuroWalknet or a calibrated
real robot gait controller.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from decentralized_ablation import (
    CODES, NEIGHBORS, MODES as OLD_MODES, ROOT, LegState, decide_liftoffs,
    initial_progresses, get_trajectory, hold_under_gravity, static_pose,
    load_legs, forward_from_geometry, tip_world, solve_waypoint, sample_plan,
    smoothstep, floor_contacts,
)

MODES=('neighbor_inhibit','neighbor_force_gate','neighbor_force_gate_supported','neighbor_active_support','neighbor_pre_lift_transfer')
TARGET_UNLOAD_N=1.50  # from pre-registered 4 Nov static calibration; not tuned on running seeds
MAX_UNLOAD_FOR_LIFT_N=1.65  # target plus 0.15 N smoothing/tolerance
PREPARATION_PROGRESS=0.90  # starts before the existing 0.99 liftoff threshold
PRELOAD_GAIN_M_PER_N_S=3.0  # same local integral gain as prior static test
MAX_SUPPORT_FORCE_FACTOR=1.8  # cap raising already overloaded neighbors
# Preconditions for load-sensitive liftoff, in units of the ideal per-leg load.
MAX_SELF_LOAD_MULTIPLIER=1.40
MIN_NEIGHBOR_LOAD_MULTIPLIER=.35
LOAD_FILTER_TAU_S=.050


def normal_contact_forces(model, data) -> dict[str,float]:
    """Sum positive normal solver forces (N) of each tibia against the floor.

    mj_contactForce uses the active contact frame. The normal component is the
    first entry; restricting to named tibia-floor contacts avoids counting
    the torso contact or contact between other parts. Forces are not impulses.
    """
    floor=int(model.geom('floor').id)
    names={int(model.geom(f'tibia{leg}_collision_0').id):leg for leg in CODES}
    totals={leg:0. for leg in CODES}
    wrench=np.zeros(6,dtype=float)
    for i in range(data.ncon):
        c=data.contact[i]
        a,b=int(c.geom1),int(c.geom2)
        if a==floor and b in names:
            leg=names[b]
        elif b==floor and a in names:
            leg=names[a]
        else:
            continue
        mujoco.mj_contactForce(model,data,i,wrench)
        value=float(wrench[0])
        if not math.isfinite(value):
            raise FloatingPointError(f'Non-finite normal contact force for {leg}')
        totals[leg]+=max(0.,value)
    return totals


def local_force_liftoffs(states: dict[str,LegState], contacts: set[str],
                         mode: str, priority: dict[str,int], t:float,
                         filtered:dict[str,float], nominal_leg_weight_N:float,
                         reasons:dict[str,int]|None=None) -> list[str]:
    """Apply force tests to the *same* simultaneous decisions as old protocol.

    Self + 2 neighboring load values are the only new sensor inputs.
    The two neighbors are not chosen by a global load maximum; no central
    decision maker chooses which leg to unload. Existing local tie-break still
    prevents neighboring simultaneous requests (same for all modes).
    """
    if mode not in MODES:
        raise ValueError(mode)
    candidates=decide_liftoffs(states,contacts,'neighbor_inhibit',priority,t)
    if mode=='neighbor_inhibit':
        return candidates
    max_self=MAX_SELF_LOAD_MULTIPLIER*nominal_leg_weight_N
    neighbor_min=MIN_NEIGHBOR_LOAD_MULTIPLIER*nominal_leg_weight_N
    result=[]
    for leg in candidates:
        if filtered[leg]>max_self:
            if reasons is not None:reasons['heavy_lifting_leg']=reasons.get('heavy_lifting_leg',0)+1
            continue
        if mode in ('neighbor_force_gate_supported','neighbor_active_support','neighbor_pre_lift_transfer') and any(filtered[n]<neighbor_min for n in NEIGHBORS[leg]):
            if reasons is not None:reasons['unloaded_neighbor']=reasons.get('unloaded_neighbor',0)+1
            continue
        if mode=='neighbor_pre_lift_transfer' and filtered[leg]>MAX_UNLOAD_FOR_LIFT_N:
            if reasons is not None: reasons['preload_incomplete']=reasons.get('preload_incomplete',0)+1
            continue
        result.append(leg)
    return result


def support_pressure_targets(states: dict[str,LegState], contacts: set[str],
                             loads: dict[str,float], ideal_N:float) -> dict[str,float]:
    """Return 0..1 downward stance-foot preload, independently for each leg.

    Each controller uses only its own contact+filtered force, and messages
    (contact, stance progress, force) from its two ring neighbors. It pushes
    when an adjacent heavily loaded leg approaches liftoff, and stops pushing
    when its own load is already high. No COM, global normal sum, phase plan or
    centralized stance command is used.
    """
    if ideal_N <= 0:
        raise ValueError('ideal_N')
    pressures={}
    for leg in CODES:
        s=states[leg]
        if s.phase!='stance' or leg not in contacts:
            pressures[leg]=0.0
            continue
        # Do not drive an already-high-force support foot further into ground.
        own_headroom=float(np.clip((1.40*ideal_N-loads[leg])/(0.50*ideal_N),0.,1.))
        need=0.
        for nb in NEIGHBORS[leg]:
            sn=states[nb]
            if sn.phase=='stance' and nb in contacts and sn.stance_progress>=.85:
                demand=float(np.clip((loads[nb]/ideal_N-1.08)/.38,0.,1.))
                need=max(need,demand)
        pressures[leg]=min(1.,own_headroom*need)
    return pressures


def pre_lift_requests(states, contacts, priority, filtered, t):
    """Locally initiated load-transfer requests, before final stance completion.

    A leg can only ask its TWO neighbors.  Adjacent simultaneous requests are
    resolved by the very same stable per-neighbor priority as the old code.
    No global support predicate or prescribed phase order is consulted.
    """
    if t < .58: return set()
    ready={c:states[c].phase=='stance' and c in contacts and
           states[c].stance_progress>=PREPARATION_PROGRESS and
           filtered[c]>TARGET_UNLOAD_N for c in CODES}
    return {c for c in CODES if ready[c] and
            not any(states[n].phase=='swing' for n in NEIGHBORS[c]) and
            not any(ready[n] and priority[n]<priority[c] for n in NEIGHBORS[c])}


def transfer_pressure_update(states, contacts, filtered, requests, pressure,
                             ideal_N, dt, max_press, max_rate):
    """Each STANCE neighbor integrates its adjacent requester's force error.

    Only direct-neighbor filtered reaction forces and local state are read.
    Commands are joint-IK *positions*, which cannot guarantee exact loads.
    For no request, unload/retract gradually; a swinging foot has no preload.
    """
    new={}
    for leg in CODES:
        if states[leg].phase!='stance' or leg not in contacts:
            desire_delta=-max_rate*dt
        else:
            eligible=[r for r in NEIGHBORS[leg] if r in requests]
            excess=max([max(0.,filtered[r]-TARGET_UNLOAD_N) for r in eligible],default=0.)
            # Reject further pressure when this stance leg is heavily loaded.
            if filtered[leg]>MAX_SUPPORT_FORCE_FACTOR*ideal_N:
                desire_delta=-max_rate*dt
            elif excess>0:
                desire_delta=min(max_rate*dt,PRELOAD_GAIN_M_PER_N_S*excess*dt)
            else:
                desire_delta=-max_rate*dt
        new[leg]=float(np.clip(pressure[leg]+desire_delta,0.,max_press))
    return new


def compute_press_delta(model, stand_qpos, legs, depth_m:float) -> dict[str,np.ndarray]:
    """One-time IK for a 0..depth_m downward stance-foot displacement.

    The small joint offset is applied to the existing stance trajectory;
    this is not full inverse-dynamics force control.
    """
    ref=mujoco.MjData(model)
    delta={}
    for leg in legs:
        ref.qpos[:]=stand_qpos
        ref.qvel[:]=0.
        mujoco.mj_forward(model,ref)
        q0=np.array(ref.qpos[leg['qadrs']])
        target=tip_world(model,ref,leg)+np.array([0.,0.,-depth_m])
        q,err=solve_waypoint(model,ref,mujoco,leg,target,q0)
        if err>0.0005:
            raise RuntimeError(f'Unreachable foot preload for {leg["suffix"]}: {err:.4f} m')
        delta[leg['suffix']]=q-q0
    return delta


def run(mode: str='neighbor_force_gate', seed:int=23, *,mass:str='nominal',
        seconds:float=8., friction:float=1.1, stride:float=.035,
        lift:float=.020, period:float=2.4, frame_hz:int=9,
        press_mm:float=4.,press_rate_mm_s:float=20.,write:bool=True) -> dict:
    if mode not in MODES:raise ValueError(mode)
    if mass not in ('lower','nominal','upper'):raise ValueError(mass)
    if not(0.<press_mm<=8.):raise ValueError('press_mm')
    if not(0.<press_rate_mm_s<=50.):raise ValueError('press_rate_mm_s')
    if not (.1<=friction<=2.):raise ValueError('friction')
    model=mujoco.MjModel.from_xml_path(str(ROOT/'source'/'mjcf'/f'scene_local30_{mass}.xml'))
    model.geom_friction[model.geom('floor').id,0]=friction
    for code in CODES:model.geom_friction[model.geom(f'tibia{code}_collision_0').id,0]=friction
    data=mujoco.MjData(model)
    legs=load_legs(model,data,mujoco)
    static_pose(model,data,mujoco,legs,.145,.15)
    stand_pose=data.qpos.copy()
    pre=hold_under_gravity(model,data,mujoco,stand_pose,1.)
    if not pre['stable_by_proxy_thresholds'] or pre['last_0p4s_min_contact_legs']<6:
        raise RuntimeError('Six-contact standing initialization failed')
    orig=data.qpos.copy()
    base=int(model.body('base_link_2').id)
    body_start=data.xpos[base].copy()
    body_orientation_start=data.xmat[base].reshape(3,3).copy()
    local_up=body_orientation_start.T@np.array([0.,0.,1.])
    forward=np.array(forward_from_geometry(legs))
    duty=.18
    stance_s=period*(1.-duty); swing_s=period*duty
    plan,err=get_trajectory(model,orig,legs,forward,stride,lift,duty)
    maxq=max(float(abs(q)) for tab in plan.values() for q in tab.flat)
    ref=mujoco.MjData(model)
    land={}
    for leg in legs:
        ref.qpos[:]=orig;ref.qvel[:]=0;mujoco.mj_forward(model,ref)
        goal=tip_world(model,ref,leg)+np.r_[forward*stride*.5,-.006]
        tab=plan[leg['suffix']]
        land[leg['suffix']]=solve_waypoint(model,ref,mujoco,leg,goal,tab[min(len(tab)-1,int(duty*len(tab)))])[0]
    press_depth=press_mm*0.001
    press_rate=press_rate_mm_s*0.001
    press_delta=compute_press_delta(model,orig,legs,press_depth)
    pressure={c:0. for c in CODES}
    pressure_active_steps=0
    request_steps=0;request_at_least_one_steps=0
    pre_lift_events=[];completed_unloads=0
    accepted_loads=[];accepted_pressure=[]
    integrated_press_mm_leg_time=0.;transfer_request_unmet=0
    pressure_max_mm=0.
    contacts_below3_2ms=0
    contacts_zero_2ms=0
    min_grounded_2ms=6
    jid=np.array(model.actuator_trnid[:,0],dtype=int)
    qa=np.array([model.jnt_qposadr[j] for j in jid],dtype=int)
    da=np.array([model.jnt_dofadr[j] for j in jid],dtype=int)
    index={int(j):i for i,j in enumerate(jid)}
    act={leg['suffix']:[index[j] for j in leg['jids']] for leg in legs}
    bounds=np.asarray(model.actuator_ctrlrange)
    q_initial=data.qpos[qa].copy()
    initials=initial_progresses(seed)
    states={name:LegState(name,stance_progress=initials[name]) for name in CODES}
    priority={c:i for i,c in enumerate(np.random.default_rng(seed+729).permutation(CODES))}
    nominal_leg_weight_N=float(model.body_mass.sum())*9.81/6
    dt=float(model.opt.timestep)
    filt_alpha=1-math.exp(-dt/LOAD_FILTER_TAU_S)
    mu_normal=normal_contact_forces(model,data)
    filtered=dict(mu_normal)
    events=[];trace=[];frames=[];force_traces=[]
    peak=0.;sat_steps=0;fall=False;lift_block_reasons={};max_swing=0
    n=round(seconds/dt);s_every=max(1,round(.02/dt));f_every=max(1,round((1./frame_hz)/dt))
    sim_start=float(data.time)
    for k in range(n):
        t=k*dt
        contacts=floor_contacts(model,data,mujoco)
        min_grounded_2ms=min(min_grounded_2ms,len(contacts))
        contacts_below3_2ms+=int(len(contacts)<3)
        contacts_zero_2ms+=int(len(contacts)==0)
        raw=normal_contact_forces(model,data)
        for c in CODES:
            filtered[c]+=filt_alpha*(raw[c]-filtered[c])
        for name in CODES:
            s=states[name]
            s.excitation=max(0.,s.excitation-dt*.72)
            if s.phase=='stance':
                s.stance_progress=min(1.,s.stance_progress+dt/stance_s)
            else:
                s.swing_age+=dt
                if s.swing_age>=swing_s*.83 and name in contacts:
                    s.phase='stance';s.stance_progress=0.;s.swing_age=0.;s.touchdowns+=1
                    events.append({'t':round(t,3),'event':'touchdown','leg':name,
                                   'normal_force_N':round(raw[name],3)})
                elif s.swing_age>swing_s+.24 and s.timeouts==0:
                    s.timeouts+=1
                    events.append({'t':round(t,3),'event':'touchdown_timeout','leg':name})
        # NEW: local force requests are prepared BEFORE nominal liftoff readiness.
        requests=(pre_lift_requests(states,contacts,priority,filtered,t)
                  if mode=='neighbor_pre_lift_transfer' else set())
        request_steps+=len(requests)
        request_at_least_one_steps+=int(bool(requests))
        # Local load gate can reject only its own lifted-leg candidate.
        accepted=local_force_liftoffs(states,contacts,mode,priority,t,
                                      filtered,nominal_leg_weight_N,lift_block_reasons)
        for name in accepted:
            accepted_loads.append(float(raw[name]))
            accepted_pressure.append(max(pressure[x] for x in NEIGHBORS[name])*1000.)
            if mode=='neighbor_pre_lift_transfer':
                completed_unloads+=1
                pre_lift_events.append({'leg':name,'time_s':round(t,3),
                    'raw_lift_force_N':round(raw[name],4),
                    'filtered_lift_force_N':round(filtered[name],4),
                    'adjacent_pressure_mm':{n:round(pressure[n]*1000,4) for n in NEIGHBORS[name]}})
            s=states[name];s.phase='swing' ;s.swing_age=0.;s.liftoffs+=1
            events.append({'t':round(t,3),'event':'liftoff','leg':name,
                           'normal_force_N':round(raw[name],3),
                           'filtered_force_N':round(filtered[name],3),
                           'left_neighbor_N':round(raw[NEIGHBORS[name][0]],3),
                           'right_neighbor_N':round(raw[NEIGHBORS[name][1]],3),
                           'other_five_normal_N':round(sum(v for c,v in raw.items() if c!=name),3)})
        if mode=='neighbor_pre_lift_transfer':
            requests-=set(accepted)
            pressure=transfer_pressure_update(states,contacts,filtered,requests,pressure,
                                    nominal_leg_weight_N,dt,press_depth,press_rate)
        else:
            targets=(support_pressure_targets(states,contacts,filtered,nominal_leg_weight_N)
                     if mode=='neighbor_active_support' else {c:0. for c in CODES})
            for c in CODES:
                desire=press_depth*targets[c]
                pressure[c]+=float(np.clip(desire-pressure[c],-press_rate*dt,press_rate*dt))
        integrated_press_mm_leg_time+=1000*sum(pressure.values())*dt/6.
        pressure_max_mm=max(pressure_max_mm,1000.*max(pressure.values()))
        pressure_active_steps+=int(any(d>.0001 for d in pressure.values()))
        qtarget=q_initial.copy()
        for name in CODES:
            s=states[name]
            if s.phase=='swing':
                phase=duty*min(1.,s.swing_age/swing_s)
                q=sample_plan(plan[name],phase)
                if s.swing_age>=swing_s:
                    alpha=smoothstep(min(1.,(s.swing_age-swing_s)/.22))
                    q=(1-alpha)*q+alpha*land[name]
            else:
                phase=duty+(1-duty)*min(.99999,s.stance_progress)
                q=sample_plan(plan[name],phase)
            if s.phase=='stance':
                q=q+(pressure[name]/press_depth)*press_delta[name]
            qtarget[act[name]]=q
        ramp=smoothstep(min(1.,t/.58))
        qtarget=q_initial+ramp*(qtarget-q_initial)
        tau_unclip=7.*(qtarget-data.qpos[qa])-.32*data.qvel[da]
        sat_steps+=int(np.any(tau_unclip>bounds[:,1]) or np.any(tau_unclip<bounds[:,0]))
        tau=np.clip(tau_unclip,bounds[:,0],bounds[:,1]);data.ctrl[:]=tau
        peak=max(peak,float(np.max(abs(tau))))
        mujoco.mj_step(model,data)
        if k%s_every==0 or k==n-1:
            pose=data.xpos[base].copy()
            tilt=float(math.degrees(math.acos(float(np.clip((data.xmat[base].reshape(3,3)@local_up)[2],-1,1)))))
            newc=floor_contacts(model,data,mujoco)
            fs=normal_contact_forces(model,data)
            swing_set={c for c in CODES if states[c].phase=='swing'}
            max_swing=max(max_swing,len(swing_set))
            trace.append({'t':round(float(data.time-sim_start),3),
                          'advance_m':round(float((pose[:2]-body_start[:2])@forward),6),
                          'body_z_m':round(float(pose[2]),6),'tilt_deg':round(tilt,3),
                          'n_contacts':len(newc),'contacts':sorted(newc),
                          'swinging':sorted(swing_set),
                          'sum_normal_force_N':round(sum(fs.values()),3),
                          'max_torque_nm':round(float(np.max(abs(tau))),4),
                          'preload_mm':{c:round(1000*pressure[c],3) for c in CODES}})
            force_traces.append({'t':round(float(data.time-sim_start),3),
                                 'forces_N':{c:round(fs[c],3) for c in CODES}})
            if pose[2]<.075 or tilt>50 or not np.all(np.isfinite(data.qpos)):
                fall=True;break
        if k%f_every==0 or k==n-1:
            frames.append({'t':round(float(data.time-sim_start),3),
                           'qpos':np.round(data.qpos,8).tolist()})
    advances=[x['advance_m'] for x in trace]
    grounded=[x['n_contacts'] for x in trace]
    tilts=[x['tilt_deg'] for x in trace]
    ntr=len(trace)
    report={
      'schema':'fnk0052_cad30_pre_lift_force_transfer/1',
      'mode':mode,'seed':seed,'initial_stance_progress':initials,
      'priority_tiebreak_order':priority,
      'mass_case':mass,'mass_kg':round(float(model.body_mass.sum()),6),
      'friction_mu_assumed':friction,
      'physics':'MuJoCo '+mujoco.__version__,
      'model':'CAD30 30/32 genuine STL, 6 true convex tibia contacts, two visual-only box proxies',
      'step_command_m':stride,'lift_command_m':lift,'period_local_s':period,
      'local_filter_tau_s':LOAD_FILTER_TAU_S,
      'nominal_ideal_leg_load_N':round(nominal_leg_weight_N,4),
      'max_self_load_for_lift_N':round(MAX_SELF_LOAD_MULTIPLIER*nominal_leg_weight_N,4),
      'min_each_adjacent_load_for_lift_N':round(MIN_NEIGHBOR_LOAD_MULTIPLIER*nominal_leg_weight_N,4),
      'uses_own_load':mode!='neighbor_inhibit',
      'uses_adjacent_load':mode in ('neighbor_force_gate_supported','neighbor_active_support'),
      'uses_active_local_support':mode=='neighbor_active_support',
      'uses_pre_lift_force_transfer':mode=='neighbor_pre_lift_transfer',
      'target_leg_force_N':TARGET_UNLOAD_N if mode=='neighbor_pre_lift_transfer' else None,
      'allow_lift_force_N':MAX_UNLOAD_FOR_LIFT_N if mode=='neighbor_pre_lift_transfer' else None,
      'active_press_target_mm':press_mm,'active_press_rate_limit_mm_s':press_rate_mm_s,
      'max_active_press_mm':round(pressure_max_mm,4),
      'mean_press_mm_per_leg_over_time':round(integrated_press_mm_leg_time/max(.001,(k+1)*dt),5),
      'fraction_times_any_prelift_request':round(request_at_least_one_steps/max(1,k+1),5),
      'mean_requesting_legs':round(request_steps/max(1,k+1),5),
      'successful_force_gated_liftoffs':completed_unloads,
      'mean_raw_force_at_liftoff_N':round(float(np.mean(accepted_loads)),4) if accepted_loads else None,
      'mean_max_adjacent_press_at_liftoff_mm':round(float(np.mean(accepted_pressure)),4) if accepted_pressure else None,
      'active_press_time_fraction':round(pressure_active_steps/max(1,k+1),5),
      'uses_total_load_to_decide':False,
      'centrally_phased':False,'no_global_one_swing_cap':True,
      'duration_s':trace[-1]['t'] if trace else 0.,
      'finished_full_duration':bool(not fall and k==n-1),
      'root_advance_cm':round(100*advances[-1],3) if advances else None,
      'min_grounded_legs':min(grounded) if grounded else None,
      'fraction_below_3_contacts':round(sum(n<3 for n in grounded)/max(1,ntr),5),
      'min_grounded_legs_2ms':min_grounded_2ms,
      'fraction_below_3_2ms':round(contacts_below3_2ms/max(1,k+1),5),
      'fraction_zero_2ms':round(contacts_zero_2ms/max(1,k+1),5),
      'fraction_zero_contacts':round(sum(n==0 for n in grounded)/max(1,ntr),5),
      'max_tilt_deg':round(max(tilts),3) if tilts else None,
      'peak_tau_nm':round(peak,5),
      'saturation_fraction':round(sat_steps/max(1,k+1),5),
      'lift_count':sum(s.liftoffs for s in states.values()),
      'touchdown_count':sum(s.touchdowns for s in states.values()),
      'touchdown_timeouts':sum(s.timeouts for s in states.values()),
      'active_leg_count':sum(s.liftoffs>0 for s in states.values()),
      'max_parallel_swing':max_swing,
      'liftoff_block_reasons':lift_block_reasons,
      'pre_lift_events':pre_lift_events,
      'events':events,'trace_50hz':trace,'force_trace_50hz':force_traces,'qpos_frames':frames,
      'warnings':['This is a static-parameter local force-to-position integral in dynamic walking, not exact force tracking',
                  'MuJoCo normal force is from solver; no physical load cell',
                  'One filter and threshold fixed for all seeds; unvalidated',
                  'Friction, servo torques, inertia and footpad shape uncalibrated',
                  'No systematic sensitivity to sensor latency/measurement noise yet']
    }
    if write:
        outfile=ROOT/'results'/f'active_support_{mode}_seed{seed}_{mass}_mu{str(friction).replace(".","p")}_press{press_mm:g}mm.json'
        outfile.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    return report


def compact(r:dict)->dict:
    keys=('mode','seed','root_advance_cm','min_grounded_legs','fraction_below_3_contacts',
          'max_tilt_deg','peak_tau_nm','saturation_fraction','liftoff_block_reasons',
          'lift_count','touchdown_count','finished_full_duration')
    return {k:r[k] for k in keys}

if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--mode',choices=MODES,default='neighbor_pre_lift_transfer')
    p.add_argument('--seed',type=int,default=23)
    p.add_argument('--mass',choices=('lower','nominal','upper'),default='nominal')
    p.add_argument('--seconds',type=float,default=8.)
    p.add_argument('--friction',type=float,default=1.1)
    p.add_argument('--press-mm',type=float,default=4.)
    p.add_argument('--press-rate-mm-s',type=float,default=20.)
    a=p.parse_args()
    print(json.dumps(compact(run(a.mode,a.seed,mass=a.mass,seconds=a.seconds,friction=a.friction,press_mm=a.press_mm,press_rate_mm_s=a.press_rate_mm_s)),indent=2))