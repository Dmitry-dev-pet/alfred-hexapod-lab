"""First CONTACT-GATED, independent-leg schedule on Freenove FNK0052 CAD30.

This is a cautious insect-inspired heuristic (local liftoff inhibition,
contact-gated touchdown, support-polygon guard), NOT Cruse/neuroWalknet,
not trained, not calibrated to real hardware.  No prescribed root translation.

Code executed inside ChatGPT's local MuJoCo Python runtime without Actions.
"""
from __future__ import annotations

from dataclasses import dataclass
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np
import mujoco

# Running from a repo checkout under experiments/cad30/ or a restored private
# Drive archive at its root should both find the first-party hexlab package.
for _path in (Path(__file__).resolve().parent, *Path(__file__).resolve().parents):
    if (_path / "hexlab").is_dir():
        sys.path.insert(0, str(_path))
        break
else:
    raise ImportError("Cannot find first-party hexlab package; restore project repo alongside experiments")

from locomotion_cad30 import ROOT, CODES, get_trajectory
from hexlab.stand_v04 import static_pose, hold_under_gravity
from hexlab.stand_geometry_v04 import load_legs, tip_world
from hexlab.step_one_leg_v04 import smoothstep, forward_from_geometry, floor_contacts, solve_waypoint
from hexlab.simulation import support_margin

# CAD suffix ring order, not Freenove vendor FR/FL labels. The first leg in the
# sequence carries no global phase; event release occurs at actual touchdown.
LEG_RING = ("fl", "ml", "bl", "br", "mr", "fr")


@dataclass
class LocalLeg:
    code: str
    state: str = "stance"
    age: float = 0.
    position: float = .5    # fraction through stance
    swing_age: float = 0.
    excitation: float = 0.
    count: int = 0
    missed_touchdown: int = 0
    delays: int = 0


def sample_plan(qtable: np.ndarray, phase: float) -> np.ndarray:
    # The last row samples phi < 1, and the first row samples phi==0;
    # with a smooth cyclic stride the interpolation is continuous.
    a = float(phase) % 1.
    x = a*len(qtable)
    low = int(math.floor(x)) % len(qtable)
    high = (low + 1) % len(qtable)
    return (1-(x-math.floor(x)))*qtable[low] + (x-math.floor(x))*qtable[high]


def may_lift(code: str, local: dict[str,LocalLeg], contacts: set[str],
             com_xy: np.ndarray, feet: dict[str,np.ndarray], margin_m: float, tilt_deg: float) -> tuple[bool,str]:
    """Feedback-only gate: the stance leg must actually touch ground."""
    if code not in contacts:
        return False,"no_contact_at_liftoff"
    if tilt_deg >= 5.:
        return False,"body_tilt"
    # Neighbor rule is bilateral contact/swing inhibition. A global 1-leg
    # swing cap is a separate conservative safety envelope, not pure locality.
    i=LEG_RING.index(code)
    neighbors=(LEG_RING[(i-1)%6],LEG_RING[(i+1)%6])
    if any(local[x].state=="swing" for x in neighbors):
        return False,"neighbor_swing"
    if any(x.state=="swing" for x in local.values()):
        return False,"safety_one_swing_cap"
    supports=[c for c in CODES if c != code and c in contacts]
    if len(supports)<5:
        return False,"less_than_five_remaining_contacts"
    margin=support_margin(tuple(com_xy), [tuple(feet[c][:2]) for c in supports])
    if not math.isfinite(margin) or margin < margin_m:
        return False,"unsafe_five_leg_support_polygon"
    return True,"accepted"


def run(case: str="nominal", *, seconds:float=8., stride:float=.035,
        foot_lift:float=.020, period:float=2.4, support_margin_m:float=.025,
        sample_hz:int=50, frames_hz:int=9, friction:float=1.1,
        landing_extension:float=.006, landing_target_enabled:bool=True,
        touchdown_dwell:float=0.0) -> dict:
    path=ROOT/"source"/"mjcf"/f"scene_local30_{case}.xml"
    if not (0. <= touchdown_dwell <= .25):
        raise ValueError("Touchdown dwell outside 0..250 ms")
    if not (0. <= landing_extension <= .012):
        raise ValueError("Landing extension must be between 0 and 12mm")
    model=mujoco.MjModel.from_xml_path(str(path))
    if not (.10 <= friction <= 2.):
        raise ValueError("Contact friction is outside tested parameter bounds")
    floor_gid=model.geom("floor").id
    model.geom_friction[floor_gid,0]=friction
    for name in CODES:
        model.geom_friction[model.geom(f"tibia{name}_collision_0").id,0]=friction
    data=mujoco.MjData(model)
    legs=load_legs(model,data,mujoco)
    static_pose(model,data,mujoco,legs,.145,.15)
    q0=data.qpos.copy()
    baseline=hold_under_gravity(model,data,mujoco,q0,1.)
    if not baseline['stable_by_proxy_thresholds'] or baseline['last_0p4s_min_contact_legs']<6:
        raise ValueError("Initial CAD standing pose was not stable")
    initial=data.qpos.copy()
    baseid=model.body("base_link_2").id
    base_initial=data.xpos[baseid].copy()
    forward=np.asarray(forward_from_geometry(legs))
    local={c:LocalLeg(c) for c in CODES}
    # All six legs are initially in midstance. No preassigned leg offsets.
    # A deterministic tie-breaker on ring location seeds the first touchdown.
    duty=.18
    stand_duration=period*(1-duty)
    swing_duration=period*duty
    plan,err=get_trajectory(model,initial,legs,forward,stride,foot_lift,duty)
    max_q=max(float(abs(q)) for pp in plan.values() for q in pp.flat)
    # A small toe-down correction recovers from a foot that hovers 1-2mm
    # above the floor due to torso bobbing: solve a landing IK target with
    # a 6mm extra extension. No fabricated ground-contact event is used.
    ref=mujoco.MjData(model)
    emergency_land={}
    for leg in legs:
        ref.qpos[:]=initial
        ref.qvel[:]=0
        mujoco.mj_forward(model,ref)
        nominal_target=tip_world(model,ref,leg)+np.r_[forward*stride*.5,-landing_extension]
        emergency_land[leg['suffix']]=solve_waypoint(
            model,ref,mujoco,leg,nominal_target,plan[leg['suffix']][int(duty*len(plan[leg['suffix']]))])[0]
    motor_jids=np.array(model.actuator_trnid[:,0],dtype=int)
    qadrs=np.array([model.jnt_qposadr[j] for j in motor_jids],dtype=int)
    dadrs=np.array([model.jnt_dofadr[j] for j in motor_jids],dtype=int)
    joint_indices={int(j):i for i,j in enumerate(motor_jids)}
    mover={l['suffix']:[joint_indices[x] for x in l['jids']] for l in legs}
    bounds=np.asarray(model.actuator_ctrlrange)
    init_tar=data.qpos[qadrs].copy()
    initial_rotation=data.xmat[baseid].reshape(3,3).copy()
    local_up=initial_rotation.T@np.array([0.,0.,1.])
    sample_every=max(1,round((1./sample_hz)/model.opt.timestep))
    frame_every=max(1,round((1./frames_hz)/model.opt.timestep))
    timeline=[];states=[];events=[];reason_counts={}
    saturations=0;max_tau=0.;fall=False
    swing_no_contact=0;swing_sample_count=0
    last_touchdown_t=-1e3
    n=round(seconds/model.opt.timestep)
    start_sim_time=float(data.time)
    for k in range(n):
        dt=float(model.opt.timestep)
        t=k*dt
        contacts=floor_contacts(model,data,mujoco)
        feet={l['suffix']:tip_world(model,data,l).copy() for l in legs}
        total_mass=float(model.body_mass.sum())
        com=(np.asarray(data.xipos)*np.asarray(model.body_mass)[:,None]).sum(axis=0)/total_mass
        rot=data.xmat[baseid].reshape(3,3)
        tilt=math.degrees(math.acos(float(np.clip((rot@local_up)[2],-1.,1.))))

        for code,state in local.items():
            state.excitation=max(0.,state.excitation - .5*dt)
            if state.state=='swing':
                state.swing_age+=dt
                swing_sample_count+=1
                if code not in contacts:
                    swing_no_contact+=1
                # Only an *actual* ground contact is a valid touchdown event.
                if state.swing_age>=swing_duration*.83 and code in contacts:
                    state.state='stance'
                    state.position=0.
                    state.age=0.
                    state.swing_age=0.
                    state.excitation=0.
                    events.append({'t_s':round(t,3),'event':'touchdown','leg':code})
                    last_touchdown_t=t
                    idx=LEG_RING.index(code)
                    for neighbor in (LEG_RING[(idx-1)%6], LEG_RING[(idx+1)%6]):
                        local[neighbor].excitation=min(1.,local[neighbor].excitation+.7)
                elif state.swing_age > swing_duration+.24:
                    # No invented touchdown: report, hold down foot target.
                    if state.missed_touchdown==0:
                        events.append({'t_s':round(t,3),'event':'touchdown_timeout','leg':code})
                    state.missed_touchdown+=1
            else:
                state.age+=dt
                state.position=min(1.,state.position+dt/stand_duration)

        # Contact-gated independent liftoff. Readiness is leg-local stance age
        # and the *neighbor's observed touchdown event*, not global phase.
        candidates=sorted(local.values(),key=lambda s:(-(s.position+min(s.excitation,.7)*.20),LEG_RING.index(s.code)))
        for state in candidates:
            if state.state!='stance':continue
            if state.position < .94 and not (state.excitation>.40 and state.position>.78):continue
            if t-last_touchdown_t < touchdown_dwell:
                state.delays+=1
                reason_counts['post_touchdown_load_transfer']=reason_counts.get('post_touchdown_load_transfer',0)+1
                continue
            approved,reason=may_lift(state.code,local,contacts,com[:2],feet,support_margin_m,tilt)
            if approved:
                state.state='swing';state.swing_age=0.;state.count+=1;state.excitation=0.
                events.append({'t_s':round(t,3),'event':'liftoff','leg':state.code,
                               'contacts_before':sorted(contacts),
                               'support_margin_before_m':round(float(support_margin(
                                tuple(com[:2]),[tuple(feet[c][:2]) for c in CODES if c!=state.code and c in contacts])),5)})
                break
            state.delays+=1
            reason_counts[reason]=reason_counts.get(reason,0)+1

        qtarget=init_tar.copy()
        for c in CODES:
            state=local[c]
            if state.state=='swing':
                phase=duty*min(1.,state.swing_age/swing_duration)
                q=sample_plan(plan[c],phase)
                if landing_target_enabled and state.swing_age>=swing_duration:
                    correction=smoothstep(min(1., (state.swing_age-swing_duration)/.22))
                    q=(1-correction)*q + correction*emergency_land[c]
            else:
                phase=duty+(1.-duty)*min(.99999,state.position)
                q=sample_plan(plan[c],phase)
            qtarget[mover[c]]=q
        # Smooth startup so midstance foot trajectory does not impulse from
        # the initial standing solver qpos.
        ramp=smoothstep(min(1.,t/.5))
        qtarget=init_tar + ramp*(qtarget-init_tar)
        tau_unc=7.*(qtarget-data.qpos[qadrs])-.32*data.qvel[dadrs]
        saturations+=int(np.any(tau_unc>bounds[:,1]) or np.any(tau_unc<bounds[:,0]))
        tau=np.clip(tau_unc,bounds[:,0],bounds[:,1]);data.ctrl[:]=tau
        max_tau=max(max_tau,float(np.max(abs(tau))))
        mujoco.mj_step(model,data)
        if k % sample_every==0 or k==n-1:
            xyz=data.xpos[baseid].copy()
            rot=data.xmat[baseid].reshape(3,3)
            tilt=math.degrees(math.acos(float(np.clip((rot@local_up)[2],-1,1))))
            actual=floor_contacts(model,data,mujoco)
            timeline.append({
              't':round(float(data.time-start_sim_time),3),
              'advance_m':round(float((xyz[:2]-base_initial[:2])@forward),6),
              'height_m':round(float(xyz[2]),6),
              'tilt_deg':round(tilt,4),
              'n_contacts':len(actual),'contacts':sorted(actual),
              'swinging':[c for c in CODES if local[c].state=='swing'],
              'max_torque_nm':round(float(np.max(abs(tau))),4),
            })
            if xyz[2]<.075 or tilt>50 or not np.all(np.isfinite(data.qpos)):
                fall=True;break
        if k % frame_every==0 or k==n-1:
            states.append({'t':round(float(data.time-start_sim_time),4),'qpos':np.round(data.qpos,8).tolist()})
    report={
      'gait':'contact_gated_local_heuristic','scenario':case,
      'model_source':'MattiaGrigoli 5c9a84c: 30 original STL and 2 visual proxies; 6 true tibia collision hulls',
      'robot_mass_kg':round(float(model.body_mass.sum()),6),'mujoco_version':mujoco.__version__,
      'ground_tibia_sliding_friction_mu_assumed':friction,
      'source_type':'local actual rigid-body physics; free base; no pinned feet',
      'duration_s':round(float(data.time-start_sim_time),3),
      'finished_full_duration':bool(not fall and k==n-1),'fall_or_nonfinite':bool(fall),
      'step_length_command_m':stride,'lift_command_m':foot_lift,'gait_period_s':period,
      'swing_duration_s':swing_duration,'swing_duty':duty,
      'feedback':{
        'true_tibia_ground_contact_gates_liftoff':True,
        'touchdown_event_measured':True,
        'neighbor_liftoff_inhibition':True,
        'max_simultaneously_swinging_legs':1,
        'support_polygon_margin_safety_m':support_margin_m,
        'post_touchdown_dwell_s':touchdown_dwell,
        'explicit_global_safety_envelope':True,
        'centrally_phased_oscillators':False,
        'biologically_validated_neuroWalknet':False,
      },
      'IK_max_error_mm':round(max(err.values())*1000,3),
      'IK_max_joint_abs_rad':round(max_q,5),
      'advance_m':timeline[-1]['advance_m'],
      'min_body_height_m':min(x['height_m'] for x in timeline),
      'max_body_tilt_deg':max(x['tilt_deg'] for x in timeline),
      'max_abs_torque_nm':round(max_tau,5),
      'saturation_fraction':round(saturations/max(1,n),5),
      'min_grounded_legs':min(x['n_contacts'] for x in timeline),
      'mean_grounded_legs':round(float(np.mean([x['n_contacts'] for x in timeline])),3),
      'samples_below_three_contacts':sum(x['n_contacts']<3 for x in timeline),
      'samples_below_five_contacts':sum(x['n_contacts']<5 for x in timeline),
      'cycles_per_leg':{c:s.count for c,s in local.items()},
      'missed_touchdown_steps':sum(s.missed_touchdown for s in local.values()),
      'touchdowns':sum(x['event']=='touchdown' for x in events),
      'liftoffs':sum(x['event']=='liftoff' for x in events),
      'swing_off_ground_fraction':round(swing_no_contact/max(1,swing_sample_count),4),
      'emergency_foot_extension_m':landing_extension,
      'explicit_landing_ik_enabled':bool(landing_target_enabled),
      'gating_reasons':reason_counts,
      'events':events,
      'trace':timeline,'qpos_samples':states,
      'physical_hardware_verified':False,
      'limitations':[
        'Independent local clocks plus local touchdown excitation, but a global one-leg safety cap',
        'Five actual contacts at liftoff is guaranteed by gate, not at every subsequent dynamic instant',
        'PD actuators, friction and mass distribution are not calibrated',
        'Not Cruse/neuroWalknet, nor a proof of insect-like locomotion',
      ]
    }
    suffix='' if abs(friction-1.1)<1e-10 else f'_mu{str(friction).replace(".","p")}'
    if abs(landing_extension-.006)>1e-10: suffix+='_extension{:.0f}mm'.format(landing_extension*1000)
    if not landing_target_enabled: suffix+='_no_landing_target'
    if touchdown_dwell: suffix+='_dwell{:.0f}ms'.format(touchdown_dwell*1000)
    (ROOT/'results'/f'cad30_gait_contact_{case}{suffix}.json').write_text(json.dumps(report,indent=2)+'\n')
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--mass',choices=('nominal','lower','upper'),default='nominal')
    p.add_argument('--seconds',type=float,default=8.)
    p.add_argument('--step',type=float,default=.035)
    p.add_argument('--lift',type=float,default=.020)
    p.add_argument('--period',type=float,default=2.4)
    p.add_argument('--margin',type=float,default=.025)
    p.add_argument('--friction',type=float,default=1.1)
    p.add_argument('--landing-extension-mm',type=float,default=6.)
    p.add_argument('--no-landing-target',action='store_true')
    p.add_argument('--touchdown-dwell-ms',type=float,default=0.)
    a=p.parse_args()
    x=run(a.mass,seconds=a.seconds,stride=a.step,foot_lift=a.lift,period=a.period,support_margin_m=a.margin,friction=a.friction,landing_extension=a.landing_extension_mm/1000.,landing_target_enabled=not a.no_landing_target,touchdown_dwell=a.touchdown_dwell_ms/1000.)
    print(json.dumps({k:v for k,v in x.items() if k not in ('trace','qpos_samples','events')},indent=2))