"""Single-leg loss at t=3s on restored FNK0052 CAD30 MuJoCo.

Independent private research script: physically disable three leg actuators and
its tibia contact at runtime, but keep mass and joint geometry. The decentralized
controller receives NO explicit damage-id notification; it can observe normal
floor contacts and two neighboring legs' phase/contact status.

Two intact-body controls and two failure arms use identical seeded initial leg
clocks, friction and motor limits. A/B causal comparisons are limited to this
un-calibrated simulation (not a physical Freenove trial).
"""
from __future__ import annotations
from dataclasses import dataclass
import copy, json, math, os, sys
from pathlib import Path
import mujoco
import numpy as np

ROOT=next((p for p in Path(__file__).resolve().parents if (p/'hexlab').is_dir()),None)
if ROOT is None: raise RuntimeError('Project hexlab/ missing: restore the main project sources')
sys.path.insert(0,str(ROOT))
from locomotion_cad30 import get_trajectory, CODES
from hexlab.stand_v04 import static_pose, hold_under_gravity
from hexlab.stand_geometry_v04 import load_legs, tip_world
from hexlab.step_one_leg_v04 import smoothstep, forward_from_geometry, floor_contacts, solve_waypoint

LEG_RING=('fl','ml','bl','br','mr','fr')
NEIGHBORS={name:(LEG_RING[(i-1)%6], LEG_RING[(i+1)%6]) for i,name in enumerate(LEG_RING)}

@dataclass
class State:
    phase:str='stance'
    stance_progress:float=0.
    swing_age:float=0.
    lifts:int=0
    touchdowns:int=0
    timeouts:int=0


def sample_plan(tab,phase):
    z=(float(phase)%1)*len(tab)
    k=int(z); alpha=z-k
    return tab[k%len(tab)]*(1-alpha)+tab[(k+1)%len(tab)]*alpha


def forces_by_leg(model,data):
    floor=int(model.geom('floor').id)
    names={int(model.geom(f'tibia{c}_collision_0').id):c for c in CODES}
    f={c:0. for c in CODES}
    w=np.zeros(6)
    for i in range(data.ncon):
        contact=data.contact[i]
        a,b=int(contact.geom1),int(contact.geom2)
        gid=b if a==floor else a if b==floor else -1
        if gid in names:
            mujoco.mj_contactForce(model,data,i,w)
            f[names[gid]]+=max(0.,float(w[0]))
    return f


def initialize():
    model=mujoco.MjModel.from_xml_path(str(ROOT/'source/mjcf/scene_local30_nominal.xml'))
    # Keep the already published locally-tested friction assumption.
    mu=1.1
    model.geom_friction[model.geom('floor').id,0]=mu
    for c in CODES:
        model.geom_friction[model.geom(f'tibia{c}_collision_0').id,0]=mu
    data=mujoco.MjData(model)
    legs=load_legs(model,data,mujoco)
    static_pose(model,data,mujoco,legs,.145,.15)
    stand=data.qpos.copy()
    pre=hold_under_gravity(model,data,mujoco,stand,1.)
    assert pre['stable_by_proxy_thresholds'] and pre['last_0p4s_min_contact_legs']==6,pre
    start_qpos=data.qpos.copy(); start_qvel=data.qvel.copy(); start_time=float(data.time)
    root=int(model.body('base_link_2').id)
    start_pos=data.xpos[root].copy()
    base_up=data.xmat[root].reshape(3,3).T@np.array([0,0,1.])
    forward=np.array(forward_from_geometry(legs))
    period=2.4;duty=.18
    plans,ikerr=get_trajectory(model,start_qpos,legs,forward,.035,.020,duty)
    land={};press_delta={}
    ref=mujoco.MjData(model)
    for l in legs:
        code=l['suffix']
        ref.qpos[:]=start_qpos;ref.qvel[:]=0;mujoco.mj_forward(model,ref)
        toe0=tip_world(model,ref,l)
        land[code]=solve_waypoint(model,ref,mujoco,l,toe0+np.r_[forward*.035*.5,-.006],plans[code][min(35,int(duty*36))])[0]
        ref.qpos[:]=start_qpos; ref.qvel[:]=0; mujoco.mj_forward(model,ref)
        press=solve_waypoint(model,ref,mujoco,l,tip_world(model,ref,l)+np.array([0.,0.,-.004]),ref.qpos[l['qadrs']].copy())
        press_delta[code]=press[0]-start_qpos[l['qadrs']]
    joint=model.actuator_trnid[:,0].astype(int)
    qa=np.array([model.jnt_qposadr[j] for j in joint],dtype=int)
    da=np.array([model.jnt_dofadr[j] for j in joint],dtype=int)
    joint_idx={int(j):i for i,j in enumerate(joint)}
    act={l['suffix']:[joint_idx[j] for j in l['jids']] for l in legs}
    return locals()


def run(ctx,seed=23,damaged=None,mode='neighbor_inhibit',failure_at=3.0,seconds=8.0,retain_frames=True):
    assert mode in ('neighbor_inhibit','neighbor_recover')
    model=ctx['model'];root=ctx['root'];forward=ctx['forward'];base_up=ctx['base_up']
    qa=ctx['qa'];da=ctx['da'];act=ctx['act'];plans=ctx['plans'];land=ctx['land'];press_delta=ctx['press_delta']
    period=ctx['period'];duty=ctx['duty'];dt=float(model.opt.timestep)
    sw=period*duty;stance=period*(1.-duty)
    # Each arm starts from the very same stabilized physical state.
    for c in CODES:
        geom=int(model.geom(f'tibia{c}_collision_0').id)
        model.geom_contype[geom]=1;model.geom_conaffinity[geom]=1
    data=mujoco.MjData(model)
    data.qpos[:]=ctx['start_qpos'];data.qvel[:]=ctx['start_qvel'];data.time=ctx['start_time']
    mujoco.mj_forward(model,data)
    state0={c:float(v) for c,v in zip(CODES,np.random.default_rng(seed).uniform(.20,.76,size=6))}
    states={c:State(stance_progress=state0[c]) for c in CODES}
    priority={c:i for i,c in enumerate(np.random.default_rng(seed+729).permutation(CODES))}
    contacts_absent={c:0. for c in CODES}
    fault_applied=False;report=[];events=[];frames=[]
    fall=False;peak_tau=0.;sat_steps=0;dead_motor_request_saturation=0;no_contact=0;below3=0
    q_init=data.qpos[qa].copy();bounds=np.array(model.actuator_ctrlrange)
    pos0=data.xpos[root].copy();last_contacts=set(CODES)
    every=max(1,round(.02/dt));frame_every=max(1,round(.125/dt))
    for k in range(round(seconds/dt)):
        t=k*dt
        if damaged and t>=failure_at and not fault_applied:
            # Physical-environment change only: invisible to policy code.
            gid=int(model.geom(f'tibia{damaged}_collision_0').id)
            model.geom_contype[gid]=0;model.geom_conaffinity[gid]=0
            fault_applied=True;events.append({'t':round(t,3),'event':'fault_physics_only','leg':damaged})
            mujoco.mj_forward(model,data)
        contacts=floor_contacts(model,data,mujoco)
        for c in CODES:
            contacts_absent[c]=0. if c in contacts else contacts_absent[c]+dt
        # Own-clock/contact state, neighbors' phases; no central safety veto.
        for c in CODES:
            s=states[c]
            if s.phase=='stance':
                s.stance_progress=min(1.,s.stance_progress+dt/stance)
            else:
                s.swing_age+=dt
                if s.swing_age>=sw*.83 and c in contacts:
                    s.phase='stance';s.stance_progress=0.;s.swing_age=0.;s.touchdowns+=1
                    events.append({'t':round(t,3),'event':'touchdown','leg':c})
                elif s.swing_age>sw+.24 and s.timeouts==0:
                    s.timeouts=1
                    events.append({'t':round(t,3),'event':'touchdown_timeout','leg':c})
        ready={c:(t>=.58 and states[c].phase=='stance' and c in contacts and states[c].stance_progress>=.99) for c in CODES}
        accepted=[]
        for c in CODES:
            if not ready[c]:continue
            nbs=NEIGHBORS[c]
            if any(states[nb].phase=='swing' for nb in nbs):continue
            if any(ready[nb] and priority[nb]<priority[c] for nb in nbs):continue
            if mode=='neighbor_recover':
                # Local detection: a neighbor has not touched ground for > .55s,
                # longer than ordinary .432s swing. Require other neighbor to
                # be visibly grounded before lifting this leg.
                failed_neighbor=any(contacts_absent[nb]>.55 for nb in nbs)
                if failed_neighbor and not all((nb in contacts) or contacts_absent[nb]>.55 for nb in nbs):continue
            accepted.append(c)
        for c in accepted:
            states[c].phase='swing';states[c].swing_age=0.;states[c].lifts+=1
            events.append({'t':round(t,3),'event':'liftoff','leg':c})
        ref=q_init.copy(); press_legs=[]
        for c in CODES:
            st=states[c]
            if st.phase=='swing':
                phi=duty*min(1.,st.swing_age/sw)
                q=sample_plan(plans[c],phi)
                if st.swing_age>=sw:
                    alpha=smoothstep(min(1.,(st.swing_age-sw)/.22))
                    q=(1-alpha)*q+alpha*land[c]
            else:
                phi=duty+(1-duty)*min(.99999,st.stance_progress)
                q=sample_plan(plans[c],phi)
                if mode=='neighbor_recover' and c in contacts and any(contacts_absent[nb]>.55 for nb in NEIGHBORS[c]):
                    q=q+press_delta[c]; press_legs.append(c)
            ref[act[c]]=q
        ramp=smoothstep(min(1.,t/.58))
        ref=q_init+ramp*(ref-q_init)
        unclipped=7.*(ref-data.qpos[qa])-.32*data.qvel[da]
        tau=np.clip(unclipped,bounds[:,0],bounds[:,1])
        if damaged and fault_applied:
            active=np.array([i for i in range(len(qa)) if i not in act[damaged]],dtype=int)
            dead=np.array(act[damaged],dtype=int)
            sat_steps+=int(np.any(unclipped[active]>bounds[active,1]) or np.any(unclipped[active]<bounds[active,0]))
            dead_motor_request_saturation+=int(np.any(unclipped[dead]>bounds[dead,1]) or np.any(unclipped[dead]<bounds[dead,0]))
            tau[dead]=0.
        else:
            sat_steps+=int(np.any(unclipped>bounds[:,1]) or np.any(unclipped<bounds[:,0]))
        data.ctrl[:]=tau
        peak_tau=max(peak_tau,float(np.max(abs(tau))))
        mujoco.mj_step(model,data)
        if k%every==0 or k==round(seconds/dt)-1:
            pos=data.xpos[root].copy()
            body_up=data.xmat[root].reshape(3,3)@base_up
            tilt=math.degrees(math.acos(float(np.clip(body_up[2],-1.,1.))))
            c_now=floor_contacts(model,data,mujoco)
            fs=forces_by_leg(model,data)
            ncon=len(c_now)
            report.append({'t':round(float((k+1)*dt),3),'advance_m':round(float((pos[:2]-pos0[:2])@forward),6),
                           'drift_m':round(float((pos[:2]-pos0[:2])@(np.array([-forward[1],forward[0]]))),6),
                           'height_m':round(float(pos[2]),6),'tilt_deg':round(tilt,3),'contacts':sorted(c_now),
                           'n_contacts':ncon,'forces_N':{c:round(fs[c],3) for c in CODES},
                           'max_tau_nm':round(float(np.max(abs(tau))),4),'press_legs':press_legs})
            below3+=int(ncon<3);no_contact+=int(ncon==0)
            if pos[2]<.075 or tilt>50 or not np.all(np.isfinite(data.qpos)):
                fall=True;events.append({'t':round(t,3),'event':'fall_threshold_reached'});break
        if retain_frames and (k%frame_every==0 or k==round(seconds/dt)-1):
            frames.append({'t':round(float((k+1)*dt),3),'qpos':np.round(data.qpos,7).tolist()})
    after=[r for r in report if r['t']>=failure_at]
    before=[r for r in report if r['t']<failure_at]
    summary={
        'mode':mode,'seed':seed,'fault_leg_physics_only':damaged,'fault_time_s':failure_at if damaged else None,
        'fault_model':'tibia floor collision removed; three leg motor torques forced to zero; body/link masses retained',
        'controller_receives_fault_flag':False,'case':'CAD30 nominal 1.804868kg MuJoCo 3.14.0',
        'friction_mu_assumed':1.1,'torque_bound_Nm_assumed':.7,
        'duration_s':report[-1]['t'],'completed':not fall and len(report)>0 and k==round(seconds/dt)-1,
        'fell_by_threshold':fall,'root_advance_cm':round(100*report[-1]['advance_m'],3),
        'post_fault_advance_cm':round(100*(report[-1]['advance_m']-before[-1]['advance_m']),3) if before else None,
        'final_drift_cm':round(100*report[-1]['drift_m'],3),'max_tilt_deg':round(max(r['tilt_deg'] for r in report),3),
        'post_fault_max_tilt_deg':round(max(r['tilt_deg'] for r in after),3) if after else None,
        'min_contacts':min(r['n_contacts'] for r in report),
        'post_fault_min_contacts':min(r['n_contacts'] for r in after) if after else None,
        'fraction_below3_50hz':round(below3/len(report),4),'fraction_zero_50hz':round(no_contact/len(report),4),
        'peak_tau_nm':round(peak_tau,4),'active_motor_saturation_fraction':round(sat_steps/max(1,k+1),4),
        'failed_motor_requested_saturation_fraction':round(dead_motor_request_saturation/max(1,k+1),4),
        'liftoffs_by_leg':{c:states[c].lifts for c in CODES},'touchdowns_by_leg':{c:states[c].touchdowns for c in CODES},
        'post_fault_mean_normal_force_by_leg_N':{c:round(float(np.mean([r['forces_N'][c] for r in after])),3) for c in CODES} if after else {},
        'induced_recovery_press_samples':sum(bool(r['press_legs']) for r in after),
        'notes':['The removed contact is a physical environment intervention, not an observation from the controller.',
                 'No real servo calibration; no hardware conclusion.',
                 'Failure detection comes from local loss of contact; special recovery mode is preprogrammed.']}
    return {'summary':summary,'events':events,'trace_50hz':report,'qpos_frames':frames}


def main():
    ctx=initialize();results=[]
    for seed in (23,11,37):
        for mode,damaged in [('neighbor_inhibit',None),('neighbor_inhibit','fl'),('neighbor_recover','fl')]:
            print('RUNNING',seed,mode,damaged,flush=True)
            out=run(ctx,seed,damaged,mode)
            path=ROOT/'results'/f'failure_leg_{damaged or "intact"}_{mode}_seed{seed}_2026-11-04.json'
            path.write_text(json.dumps(out,indent=2,ensure_ascii=False)+'\n')
            print(json.dumps(out['summary'],ensure_ascii=False),flush=True)
            results.append(out['summary'])
    (ROOT/'results'/'failure_one_leg_cad30_summary_2026-11-04.json').write_text(json.dumps(results,indent=2,ensure_ascii=False)+'\n')

if __name__=='__main__':main()