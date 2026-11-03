"""Load-aware *local* liftoff ablation on the FNK0052 CAD30 MuJoCo model.

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

MODES=('neighbor_inhibit','neighbor_force_gate','neighbor_force_gate_supported')
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
        if mode=='neighbor_force_gate_supported' and any(filtered[n]<neighbor_min for n in NEIGHBORS[leg]):
            if reasons is not None:reasons['unloaded_neighbor']=reasons.get('unloaded_neighbor',0)+1
            continue
        result.append(leg)
    return result


def run(mode: str='neighbor_force_gate', seed:int=23, *,mass:str='nominal',
        seconds:float=8., friction:float=1.1, stride:float=.035,
        lift:float=.020, period:float=2.4, frame_hz:int=9,
        write:bool=True) -> dict:
    if mode not in MODES:raise ValueError(mode)
    if mass not in ('lower','nominal','upper'):raise ValueError(mass)
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
        # Local load gate can reject only its own lifted-leg candidate.
        accepted=local_force_liftoffs(states,contacts,mode,priority,t,
                                      filtered,nominal_leg_weight_N,lift_block_reasons)
        for name in accepted:
            s=states[name];s.phase='swing';s.swing_age=0.;s.liftoffs+=1
            events.append({'t':round(t,3),'event':'liftoff','leg':name,
                           'normal_force_N':round(raw[name],3),
                           'filtered_force_N':round(filtered[name],3),
                           'left_neighbor_N':round(raw[NEIGHBORS[name][0]],3),
                           'right_neighbor_N':round(raw[NEIGHBORS[name][1]],3),
                           'other_five_normal_N':round(sum(v for c,v in raw.items() if c!=name),3)})
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
                          'max_torque_nm':round(float(np.max(abs(tau))),4)})
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
      'schema':'fnk0052_cad30_local_normal_force_feedback/1',
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
      'uses_adjacent_load':mode=='neighbor_force_gate_supported',
      'uses_total_load_to_decide':False,
      'centrally_phased':False,'no_global_one_swing_cap':True,
      'duration_s':trace[-1]['t'] if trace else 0.,
      'finished_full_duration':bool(not fall and k==n-1),
      'root_advance_cm':round(100*advances[-1],3) if advances else None,
      'min_grounded_legs':min(grounded) if grounded else None,
      'fraction_below_3_contacts':round(sum(n<3 for n in grounded)/max(1,ntr),5),
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
      'events':events,'trace_50hz':trace,'force_trace_50hz':force_traces,'qpos_frames':frames,
      'warnings':['MuJoCo normal force is from solver; no physical load cell',
                  'One filter and threshold fixed for all seeds; unvalidated',
                  'Friction, servo torques, inertia and footpad shape uncalibrated',
                  'No systematic sensitivity to sensor latency/measurement noise yet']
    }
    if write:
        outfile=ROOT/'results'/f'load_feedback_{mode}_seed{seed}_{mass}_mu{str(friction).replace(".","p")}.json'
        outfile.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    return report


def compact(r:dict)->dict:
    keys=('mode','seed','root_advance_cm','min_grounded_legs','fraction_below_3_contacts',
          'max_tilt_deg','peak_tau_nm','saturation_fraction','liftoff_block_reasons',
          'lift_count','touchdown_count','finished_full_duration')
    return {k:r[k] for k in keys}

if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--mode',choices=MODES,default='neighbor_force_gate')
    p.add_argument('--seed',type=int,default=23)
    p.add_argument('--mass',choices=('lower','nominal','upper'),default='nominal')
    p.add_argument('--seconds',type=float,default=8.)
    p.add_argument('--friction',type=float,default=1.1)
    a=p.parse_args()
    print(json.dumps(compact(run(a.mode,a.seed,mass=a.mass,seconds=a.seconds,friction=a.friction)),indent=2))
