"""Minimal-rule emergence A/B/C ablation on Freenove FNK0052 CAD30 MuJoCo.

A: six identical proprioceptive leg controllers: no inter-leg messages.
B: A + inhibit swing when an adjacent leg is swinging.
C: B + short neighbor readiness excitation after *observed* touchdown.

Shared physical body transmits forces in all 3 conditions; no central gait
clock, fixed phase offsets, global one-swing cap, COM-based gate, or driven body.
This is an exploratory diagnostic, not neuroWalknet or hardware validation.
"""
from __future__ import annotations
import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

for par in (Path(__file__).resolve().parent, *Path(__file__).resolve().parents):
    if (par / 'hexlab').is_dir():
        sys.path.insert(0, str(par))
        break
else:
    raise RuntimeError('Restore the project hexlab/ Python package first')

from locomotion_cad30 import ROOT, CODES, get_trajectory
from hexlab.stand_v04 import static_pose, hold_under_gravity
from hexlab.stand_geometry_v04 import load_legs, tip_world
from hexlab.step_one_leg_v04 import smoothstep, forward_from_geometry, floor_contacts, solve_waypoint
from local_contact_gait import LEG_RING, sample_plan

MODES = ('independent', 'neighbor_inhibit', 'neighbor_touchdown')
RING = LEG_RING
NEIGHBORS = {c: (RING[(i-1)%6], RING[(i+1)%6]) for i,c in enumerate(RING)}

@dataclass
class LegState:
    name: str
    phase: str = 'stance'
    stance_progress: float = 0.
    swing_age: float = 0.
    excitation: float = 0.
    liftoffs: int = 0
    touchdowns: int = 0
    timeouts: int = 0


def decide_liftoffs(states: dict[str,LegState], contacts: set[str],
                    mode: str, priority: dict[str,int], t: float) -> list[str]:
    """Simultaneous local decisions, no central candidate sorting.

    A requesting leg may know its own stance/contact, neighbors' swing/request
    state and stable neighbor priority only. Independent mode sees no peers.
    All decisions are based on the *same* pre-decision snapshot.
    """
    if mode not in MODES: raise ValueError(mode)
    ready={c: (t>=.58 and states[c].phase=='stance' and c in contacts and
               (states[c].stance_progress>=.99 or
                (mode=='neighbor_touchdown' and states[c].excitation>=.35 and
                 states[c].stance_progress>=.78))) for c in CODES}
    accepted=[]
    for c in CODES:
        if not ready[c]:continue
        if mode!='independent':
            if any(states[nb].phase=='swing' for nb in NEIGHBORS[c]):continue
            if any(ready[nb] and priority[nb]<priority[c] for nb in NEIGHBORS[c]):continue
        accepted.append(c)
    return accepted


def initial_progresses(seed: int) -> dict[str, float]:
    # Randomized initial *local* clocks, same sample across all variants.
    # No preassigned alternating tripod or spatial wave phases.
    rng = np.random.default_rng(seed)
    return {name: float(v) for name,v in zip(CODES, rng.uniform(.20, .76, size=6))}


def run(mode: str, seed: int, *, mass: str='nominal', seconds:float=8.,
        friction:float=1.1, stride:float=.035, lift:float=.020,
        period:float=2.4, frame_hz:int=9, write:bool=True) -> dict:
    if mode not in MODES: raise ValueError(mode)
    if mass not in ('lower','nominal','upper'): raise ValueError(mass)
    if not (.1 <= friction <= 2.): raise ValueError('friction')
    model=mujoco.MjModel.from_xml_path(str(ROOT/'source'/'mjcf'/f'scene_local30_{mass}.xml'))
    model.geom_friction[model.geom('floor').id, 0]=friction
    for code in CODES:
        model.geom_friction[model.geom(f'tibia{code}_collision_0').id,0]=friction
    data=mujoco.MjData(model)
    legs=load_legs(model,data,mujoco)
    static_pose(model,data,mujoco,legs,.145,.15)
    stand_pose=np.array(data.qpos)
    pre=hold_under_gravity(model,data,mujoco,stand_pose,1.)
    if not pre['stable_by_proxy_thresholds'] or pre['last_0p4s_min_contact_legs']<6:
        raise RuntimeError('Six-contact standing initialization failed')
    original=np.array(data.qpos)
    root=int(model.body('base_link_2').id)
    start_pos=np.array(data.xpos[root])
    start_rot=data.xmat[root].reshape(3,3).copy()
    local_up=start_rot.T @ np.array([0.,0.,1.])
    forward=np.array(forward_from_geometry(legs))
    duty=.18
    stance_time=period*(1-duty)
    swing_time=period*duty
    qplans, ikerrors=get_trajectory(model,original,legs,forward,stride,lift,duty)
    maxq=max(abs(float(q)) for table in qplans.values() for q in table.flat)
    # Same reachable touchdown correction in all variants.
    ref=mujoco.MjData(model)
    land={}
    for l in legs:
        ref.qpos[:]=original;ref.qvel[:]=0.;mujoco.mj_forward(model,ref)
        goal=tip_world(model,ref,l)+np.r_[forward*stride*.5,-.006]
        table=qplans[l['suffix']]
        land[l['suffix']]=solve_waypoint(model,ref,mujoco,l,goal,table[min(len(table)-1, int(duty*len(table)))])[0]
    joint=np.array(model.actuator_trnid[:,0],dtype=int)
    qa=np.array([model.jnt_qposadr[j] for j in joint],dtype=int)
    da=np.array([model.jnt_dofadr[j] for j in joint],dtype=int)
    joint_to_motor={int(j):i for i,j in enumerate(joint)}
    act={l['suffix']:[joint_to_motor[j] for j in l['jids']] for l in legs}
    limits=np.asarray(model.actuator_ctrlrange)
    ref_start=data.qpos[qa].copy()

    initial=initial_progresses(seed)
    states={c:LegState(c,stance_progress=initial[c]) for c in CODES}
    events=[];trace=[];snapshots=[]
    max_tau=0.;saturation=0;failed=False;near_neighbor_overlap=0;max_swing=0;bothswing_count=0
    n=round(seconds/model.opt.timestep)
    dt=model.opt.timestep
    sample_every=max(1,round(.02/dt));frame_every=max(1,round((1./frame_hz)/dt))
    sim_start=float(data.time)
    # Symmetry breaker is a per-leg persistent local priority. It is used ONLY
    # if both this leg and a directly adjacent leg request liftoff in the same
    # time step. This is not a global arbitration or shared phase ranking.
    priority={c:i for i,c in enumerate(np.random.default_rng(seed+729).permutation(CODES))}
    swing_no_contact=0;swing_samples=0
    for k in range(n):
        t=k*dt
        contacts=floor_contacts(model,data,mujoco)
        # Each leg's mechanical contacts and neighbor messages are only accessible
        # to the corresponding local controller. Diagnostics see the whole robot.
        for name in CODES:
            st=states[name]
            st.excitation=max(0.,st.excitation-dt*.72)
            if st.phase=='stance':
                st.stance_progress=min(1.,st.stance_progress+dt/stance_time)
                continue
            st.swing_age+=dt
            swing_samples+=1
            if name not in contacts:swing_no_contact+=1
            if st.swing_age>=swing_time*.83 and name in contacts:
                st.phase='stance';st.stance_progress=0.;st.swing_age=0.;st.touchdowns+=1
                events.append({'t':round(t,3),'event':'touchdown','leg':name})
                if mode=='neighbor_touchdown':
                    for nb in NEIGHBORS[name]:
                        states[nb].excitation=min(.85,states[nb].excitation+.65)
            elif st.swing_age > swing_time+.24 and st.timeouts==0:
                st.timeouts+=1
                events.append({'t':round(t,3),'event':'touchdown_timeout','leg':name})
        # Evaluate all local liftoff decisions on the same state snapshot.
        will_lift=decide_liftoffs(states, contacts, mode, priority, t)
        for name in will_lift:
            st=states[name]
            st.phase='swing';st.swing_age=0.;st.excitation=0.;st.liftoffs+=1
            events.append({'t':round(t,3),'event':'liftoff','leg':name,'contacts_before':sorted(contacts)})
        qtarget=ref_start.copy()
        for name in CODES:
            st=states[name]
            if st.phase=='swing':
                phase=duty*min(1.,st.swing_age/swing_time)
                q=sample_plan(qplans[name],phase)
                if st.swing_age>=swing_time:
                    alpha=smoothstep(min(1.,(st.swing_age-swing_time)/.22))
                    q=(1-alpha)*q+alpha*land[name]
            else:
                phase=duty+(1-duty)*min(.99999,st.stance_progress)
                q=sample_plan(qplans[name],phase)
            qtarget[act[name]]=q
        ramp=smoothstep(min(1.,t/.58))
        qtarget=ref_start+ramp*(qtarget-ref_start)
        unbounded=7.*(qtarget-data.qpos[qa])-.32*data.qvel[da]
        saturation+=int(np.any(unbounded>limits[:,1]) or np.any(unbounded<limits[:,0]))
        tau=np.clip(unbounded,limits[:,0],limits[:,1]);data.ctrl[:]=tau
        max_tau=max(max_tau,float(np.max(abs(tau))))
        mujoco.mj_step(model,data)
        if k % sample_every == 0 or k==n-1:
            pose=data.xpos[root].copy()
            up=data.xmat[root].reshape(3,3)@local_up
            tilt=float(math.degrees(math.acos(float(np.clip(up[2],-1.,1.)))))
            actual_contacts=floor_contacts(model,data,mujoco)
            swing_set={c for c in CODES if states[c].phase=='swing'}
            n_pairs=sum(c in swing_set and NEIGHBORS[c][0] in swing_set for c in CODES)
            if n_pairs:near_neighbor_overlap+=1
            n_swing=len(swing_set)
            max_swing=max(max_swing,n_swing)
            if n_swing>=2:bothswing_count+=1
            trace.append({'t':round(float(data.time-sim_start),3),
                          'advance_m':round(float((pose[:2]-start_pos[:2])@forward),6),
                          'body_z_m':round(float(pose[2]),6),
                          'tilt_deg':round(tilt,3),
                          'contacts':sorted(actual_contacts),'n_contacts':len(actual_contacts),
                          'swinging':sorted(swing_set),'neighbor_overlap_pairs':n_pairs,
                          'max_tau_nm':round(float(np.max(abs(tau))),4)})
            if pose[2]<.075 or tilt>50 or not np.all(np.isfinite(data.qpos)):
                failed=True
                break
        if k % frame_every==0 or k==n-1:
            snapshots.append({'t':round(float(data.time-sim_start),3),'qpos':np.round(data.qpos,8).tolist()})
    lc=len(trace)
    report={
        'schema':'fnk0052_cad30_decentralization_ablation/1',
        'mode':mode,'seed':seed,'initial_stance_progress':initial,'priority_tiebreak_order':priority,
        'mass':mass,'mass_kg':round(float(model.body_mass.sum()),6),'friction_mu_assumed':friction,
        'physics':'MuJoCo '+mujoco.__version__, 'model':'30/32 original Freenove STL; six tibia contact hulls, 2 visual boxes',
        'period_local_s':period,'swing_duty':duty,'step_command_m':stride,'lift_command_m':lift,
        'each_leg_has_its_own_clock':True, 'centrally_assigned_phases':False,
        'global_support_or_one_swing_gate':False,
        'globally_sorted_candidate_list':False,
        'simultaneous_neighbor_requests_resolved_locally':True,
        'uses_contact_own_leg':True,
        'neighbor_inhibition':mode in ('neighbor_inhibit','neighbor_touchdown'),
        'neighbor_touchdown_excitation':mode=='neighbor_touchdown',
        'body_actuated_directly':False,'feet_pinned':False,
        'duration_s':trace[-1]['t'] if trace else 0.,
        'finished_full_duration':bool(not failed and k==n-1),
        'root_advance_cm':round(trace[-1]['advance_m']*100,3) if trace else None,
        'min_grounded_legs':min(x['n_contacts'] for x in trace) if trace else None,
        'fraction_below_3_contacts':round(sum(x['n_contacts']<3 for x in trace)/lc,4) if lc else None,
        'fraction_0_contacts':round(sum(x['n_contacts']==0 for x in trace)/lc,4) if lc else None,
        'max_tilt_deg':round(max(x['tilt_deg'] for x in trace),3) if trace else None,
        'peak_tau_nm':round(max_tau,5),
        'saturation_fraction':round(saturation/max(1,k+1),5),
        'lift_count':sum(s.liftoffs for s in states.values()),
        'touchdown_count':sum(s.touchdowns for s in states.values()),
        'touchdown_timeouts':sum(s.timeouts for s in states.values()),
        'active_leg_count':sum(s.liftoffs>0 for s in states.values()),
        'per_leg_liftoffs':{c:states[c].liftoffs for c in CODES},
        'max_parallel_swing':max_swing,
        'fraction_two_or_more_swinging':round(bothswing_count/lc,4) if lc else None,
        'fraction_adjacent_swinging':round(near_neighbor_overlap/lc,4) if lc else None,
        'swing_no_contact_fraction':round(swing_no_contact/max(1,swing_samples),4),
        'IK_max_error_mm':round(max(ikerrors.values())*1000,3),
        'IK_max_abs_joint_rad':round(maxq,4),
        'events':events,
        'trace_50hz':trace,'qpos_frames':snapshots,
        'notes':[
            'Comparisons are observational within one physics model, not a hardware result',
            'No central gait clock or support-polygon safety check in any arm',
            'Touchdown needs actual tibia-floor contact after >= 83% nominal swing',
            'Motor torque/angle, foot friction and mass are not calibrated',
            'Neighbor rule uses fixed spatial adjacency graph; success does not prove biological emergence',
            'All controllers are executed by one simulation process; only logical information is decentralized',
            'Independent legs still interact physically through the common torso',
        ]
    }
    if write:
        dest=ROOT/'results'/f'decentral_{mode}_seed{seed}_{mass}_mu{str(friction).replace(".","p")}.json'
        dest.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    return report


def compact(r:dict) -> dict:
    keys=('mode','seed','finished_full_duration','duration_s','root_advance_cm','min_grounded_legs','fraction_below_3_contacts','max_tilt_deg','peak_tau_nm','saturation_fraction','lift_count','touchdown_count','active_leg_count','max_parallel_swing','fraction_adjacent_swinging')
    return {key:r[key] for key in keys}

if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--mode',choices=MODES,required=True)
    p.add_argument('--seed',type=int,default=11)
    p.add_argument('--mass',choices=('nominal','lower','upper'),default='nominal')
    p.add_argument('--seconds',type=float,default=8.)
    p.add_argument('--friction',type=float,default=1.1)
    p.add_argument('--step',type=float,default=.035)
    p.add_argument('--lift',type=float,default=.020)
    args=p.parse_args()
    result=run(args.mode,args.seed,mass=args.mass,seconds=args.seconds,friction=args.friction,stride=args.step,lift=args.lift)
    print(json.dumps(compact(result),ensure_ascii=False,indent=2))