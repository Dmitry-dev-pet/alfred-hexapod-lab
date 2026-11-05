#!/usr/bin/env python3
"""Six-leg contact-gated CAD30 Wave diagnostic, NO artificial root/foot motion.

Reuses existing 30-original-STL MuJoCo CAD30 model and inverse-kinematic
wave targets. Independently measures tibia-floor Fz every physics step.
The safety gate delays each new swing until all six legs are measurably loaded
and holds at the end of swing until the moved foot loads again. Optional
small stance-position feedback is in joint target space, never a force edit.
The purpose is an A/B control experiment, not a certified hardware gait.
"""
from __future__ import annotations
import argparse, json, math, sys, os, gzip
from pathlib import Path
from collections import Counter
import numpy as np
import mujoco as mj

ROOT=Path(__file__).resolve().parent
BASE=Path(os.environ.get('CAD30_WORK',str(ROOT.parent))).resolve()
sys.path.insert(0,str(BASE))
from locomotion_cad30 import CODES, get_trajectory
from hexlab.stand_v04 import static_pose, hold_under_gravity
from hexlab.stand_geometry_v04 import load_legs, tip_world, geom_mesh_world_vertices
from hexlab.step_one_leg_v04 import smoothstep,forward_from_geometry

FZ_CUTOFF=.2
DUTY=.16
# Actually time offset .161 means just before a wave swing start (fl), all legs stance at start.
START_CLOCK=0.161*3.0


def read_fz(model,data,floor,geom_to_leg):
    fz={c:0.0 for c in CODES};grounded=set()
    for i in range(data.ncon):
        contact=data.contact[i]
        a,b=int(contact.geom1),int(contact.geom2)
        if a==floor: g=b
        elif b==floor:g=a
        else:continue
        leg=geom_to_leg.get(g)
        if leg is None:continue
        force=np.zeros(6)
        mj.mj_contactForce(model,data,i,force)
        # Frame's 1st axis is local contact normal; contact solver normal positive.
        fz[leg]+=max(0.,float(force[0]));grounded.add(leg)
    return fz,grounded


def phase_index(clock,period,phases):
    active=[c for c in CODES if ((clock/period+phases[c])%1)<DUTY]
    return active


def run(mode='gated',seconds=12.,ramp_s=6.,step=.020,lift=.012,period=3.,
        touchdown_timeout=.8,preload=False, output=None, sample_hz=25., record_2ms=False):
    assert mode in ('gated','baseline','gate+feedback')
    assert seconds>0 and ramp_s>0 and period>0
    model=mj.MjModel.from_xml_path(str(BASE/'source/mjcf/scene_local30_nominal.xml'))
    model.opt.timestep=.002
    data=mj.MjData(model)
    legs=load_legs(model,data,mj)
    fit=static_pose(model,data,mj,legs,.145,.15)
    pre=hold_under_gravity(model,data,mj,data.qpos.copy(),1.0)
    assert pre['stable_by_proxy_thresholds']
    initial=data.qpos.copy()
    base_id=model.body('base_link_2').id
    root0=data.xpos[base_id].copy()
    rot0=data.xmat[base_id].reshape(3,3).copy();up_local=rot0.T@np.array([0.,0.,1.])
    forward=np.asarray(forward_from_geometry(legs))
    plan,errors=get_trajectory(model,initial,legs,forward,step,lift,DUTY)
    motor_jids=np.array(model.actuator_trnid[:,0],dtype=int)
    qadrs=np.asarray(model.jnt_qposadr[motor_jids],dtype=int)
    dadrs=np.asarray(model.jnt_dofadr[motor_jids],dtype=int)
    qref=data.qpos[qadrs].copy()
    by_joint={int(j):i for i,j in enumerate(motor_jids)}
    moving={leg['suffix']:[by_joint[j] for j in leg['jids']] for leg in legs}
    leginfo={leg['suffix']:leg for leg in legs}
    phases={c:i/6 for i,c in enumerate(CODES)}
    floor=int(model.geom('floor').id)
    gm={int(model.geom('tibia'+c+'_collision_0').id):c for c in CODES}
    bounds=np.asarray(model.actuator_ctrlrange)
    freq=int(round(1/sample_hz/model.opt.timestep))
    dt=float(model.opt.timestep)
    real_t=0.;clock=START_CLOCK if mode!='baseline' else 0.
    prev_swing=None;gate_states={};good_six_ticks=0;hold_s=0.;total_gate_hold=0.
    phase_holds=Counter();gate_events=[];low_frames=[];bad_segments=[]
    nhist=Counter();geomhist=Counter();max_tau=0.;max_raw=0.;sat_count=0;max_tilt=0.;qmax=0.;max_advance=-1e9;min_root_z=10.;n=round(seconds/dt)
    phase_reached=Counter();confirmed_landing=Counter();lifted_legs=set();started_swing=Counter()
    fz,geoms=read_fz(model,data,floor,gm)
    snap=[];body_progress=[];min_loaded=6;stop_reason=None;gate_state='none';
    raw_frames=[];named_events=[];named_stance_loss_by_mode=Counter();
    named_loss=Counter();named_low=[];lift_free=Counter();offfloor_two_mm=Counter();
    foot_start={c:tip_world(model,data,leginfo[c]).copy() for c in CODES};
    leg_foot_xy_min={c:float('inf') for c in CODES};leg_foot_xy_max={c:float('-inf') for c in CODES}
    # Feedback target downward toe penetration in mm, acts on stance legs only
    corrections={k:0.0 for k in CODES}
    # compute gradient at actual joint state each 20ms (not every step)
    zjac={}
    max_wait=1.2
    for k in range(n):
        real_t=k*dt
        held=False
        # Before changing clock, check whether leg is entering swing or landing.
        if mode!='baseline':
            active0=phase_index(clock,period,phases)
            active1=phase_index(clock+dt,period,phases)
            all_six=all(fz[c]>=FZ_CUTOFF for c in CODES)
            good_six_ticks=good_six_ticks+1 if all_six else 0
            # gate if crossing into next swing (or already held at a swing start)
            entering=next((c for c in active1 if c not in active0),None)
            exiting=next((c for c in active0 if c not in active1),None)
            if entering is not None and good_six_ticks<8:
                held=True;gate_state='before_'+entering;phase_holds['pre_swing']+=1
            elif exiting is not None and (fz[exiting]<FZ_CUTOFF or good_six_ticks<8):
                held=True;gate_state='after_'+exiting;phase_holds['touchdown']+=1
            else:
                gate_state='moving';
                if entering is not None:started_swing[entering]+=1
                if exiting is not None:confirmed_landing[exiting]+=1
            # Hold until foot can load; if unreachable report as failed rather than freeing contacts.
            if held:
                hold_s+=dt;total_gate_hold+=dt
                if hold_s>max_wait:
                    stop_reason='gate_timeout_'+gate_state
                    gate_events.append({'t':round(real_t,4),'clock':round(clock,4),'type':stop_reason,'Fz':fz.copy()})
                    break
            else:hold_s=0.
            # Prevent crossing to next phase; nominal speed otherwise.
            if not held:clock+=dt
        else:clock=real_t
        # Body-relative IK + acceleration ramp. All commands to 18 servos (+2 head unchanged).
        ramp=smoothstep(min(1.,real_t/ramp_s))
        qtar=qref.copy()
        swing_now=phase_index(clock,period,phases)
        for c in CODES:
            phase=(clock/period+phases[c])%1
            table=plan[c]
            idx=phase*len(table)
            lo=int(math.floor(idx))%len(table);hi=(lo+1)%len(table)
            frac=idx-math.floor(idx)
            qa=(1-frac)*table[lo]+frac*table[hi]
            indices=moving[c]
            qtar[indices]=qref[indices]+ramp*(qa-qref[indices])
        if mode=='gate+feedback':
            # Conservative support-load correction, only 5 stance legs, height response
            for c in CODES:
                if c in swing_now:
                    corrections[c]=max(0.,corrections[c]-dt*.006)
                    continue
                # feedforward 0.35 mm slight preload plus dynamic Fz-dependent <=2 mm
                ft=fz[c]
                desired=.0004+np.clip(1.5-ft,0,1.5)*.00105
                if ft>2.5:desired=.0002
                corrections[c]+=(np.clip(desired-corrections[c],-dt*.008,dt*.008))
                if k%10==0 or c not in zjac:
                    toe=tip_world(model,data,leginfo[c]);posj=np.zeros((3,model.nv));rotj=np.zeros((3,model.nv));mj.mj_jac(model,data,posj,rotj,toe,int(leginfo[c]['body']))
                    zjac[c]=posj[2,leginfo[c]['dadrs']].copy()
                gradient=zjac[c];norm=float(gradient@gradient)+1e-3
                delta=-corrections[c]*gradient/norm
                qtar[moving[c]]+=np.clip(delta,-.035,.035)
        unc=7*(qtar-data.qpos[qadrs])-.32*data.qvel[dadrs]
        tau=np.clip(unc,bounds[:,0],bounds[:,1]);data.ctrl[:]=tau
        if np.any(unc>bounds[:,1]) or np.any(unc<bounds[:,0]):sat_count+=1
        max_tau=max(max_tau,float(np.max(np.abs(tau))));max_raw=max(max_raw,float(np.max(np.abs(unc))))
        mj.mj_step(model,data)
        fz,geoms=read_fz(model,data,floor,gm)
        loaded=[c for c in CODES if fz[c]>=FZ_CUTOFF]
        geom_n=len(geoms);count=len(loaded)
        nhist[count]+=1;geomhist[geom_n]+=1;min_loaded=min(min_loaded,count)
        # A count of five can hide an unexpected missing stance leg plus a loaded swing foot.
        for c in CODES:
            if c in swing_now:
                if fz[c]<.08 and c not in geoms:lift_free[c]+=1
            elif fz[c]<FZ_CUTOFF:
                named_loss[c]+=1
                named_stance_loss_by_mode['swing' if swing_now else 'all_stance_gap']+=1
                named_event={'t':round(real_t+dt,4),'leg':c,'swing':swing_now,'Fz':round(fz[c],6)}
                named_events.append(named_event)
                if len(named_low)<40:named_low.append(named_event)
        qmax=max(qmax,float(np.max(np.abs(data.qpos[qadrs[:18]]))))
        if record_2ms:
            raw_frames.append({'t':round(real_t+dt,4),'phase':round(clock,5),'active':swing_now,
            'Fz':{c:round(float(fz[c]),5) for c in CODES},'touch_geoms':sorted(geoms),
            'root_xyz':np.round(data.xpos[base_id],6).tolist(),
            'qpos':np.round(data.qpos,7).tolist(),'qvel':np.round(data.qvel,6).tolist(),
            'command_tau':np.round(tau,6).tolist(),'target_q':np.round(qtar,6).tolist()})
        root=data.xpos[base_id];tilt=math.degrees(math.acos(float(np.clip((data.xmat[base_id].reshape(3,3)@up_local)[2],-1.,1.))))
        max_tilt=max(max_tilt,tilt);min_root_z=min(min_root_z,float(root[2]));adv=float((root[:2]-root0[:2])@forward);max_advance=max(max_advance,adv)
        if count<5:
            missing=[c for c in CODES if c not in loaded]
            bad={'t':round(real_t+dt,4),'n':count,'active':swing_now,'missing':missing,'unexpected':list(set(missing)-set(swing_now)),'fz':{c:round(fz[c],4) for c in CODES},'geom_n':geom_n,'advance_m':round(adv,6),'paused':held}
            low_frames.append(bad)
            if not bad_segments or real_t+dt-bad_segments[-1]['end']>dt*1.1:
                bad_segments.append({'start':real_t+dt,'end':real_t+dt,'count':1,'min_n':count})
            else:
                x=bad_segments[-1];x['end']=real_t+dt;x['count']+=1;x['min_n']=min(x['min_n'],count)
        if (k%freq==0 or k==n-1):
            for c in CODES:
                tip=tip_world(model,data,leginfo[c]);leg_foot_xy_min[c]=min(leg_foot_xy_min[c],float(tip[1]));leg_foot_xy_max[c]=max(leg_foot_xy_max[c],float(tip[1]))
                if c in swing_now and c not in geoms and k%(freq*10)==0:
                    # Sparse CAD geometry check, full frame-level postprocess is performed offline.
                    lowest=float(np.min(geom_mesh_world_vertices(model,data,leginfo[c]['geom'])[:,2]))
                    if lowest>.002:offfloor_two_mm[c]+=1
            snap.append({'t':round(real_t+dt,4),'gait_clock':round(clock,5),'base_xyz':np.round(root,6).tolist(),'advance_m':round(adv,6),'n_loaded':count,'loaded':loaded,'Fz':{c:round(fz[c],3) for c in CODES},'qpos':np.round(data.qpos,7).tolist()})
        if not np.all(np.isfinite(data.qpos)) or float(root[2])<.075 or tilt>50:
            stop_reason='fall_or_nonfinite';break
    elapsed=(k+1)*dt
    result={'schema':'cad30-six-wave-contact-gated/1','mode':mode,'run_seconds':seconds,'elapsed_seconds':round(elapsed,4),
     'gait': 'wave','phase_duty':DUTY,'ramp_s':ramp_s,'step_m':step,'lift_m':lift,'period_s':period,
     'model_source':'CAD30 30 original STL, 2 visual proxies; local scene_local30_nominal.xml','mu':1.1,
     'model_mass_kg':round(float(model.body_mass.sum()),5),'dt_s':dt,'Fz_threshold_N':FZ_CUTOFF,
     'progress_m':round(adv,6),'max_progress_m':round(max_advance,6),'min_real_Fz_loaded_legs':min_loaded,
     'num_under5_ticks':len(low_frames),'fraction_5plus':round((sum(v for c,v in nhist.items() if c>=5)/max(1,sum(nhist.values()))),7),
     'loaded_hist':dict(sorted(nhist.items())),'contact_geom_hist':dict(sorted(geomhist.items())),
     'phase_holds':dict(phase_holds),'hold_s_total':round(total_gate_hold,4),
     'gate_started_swing':dict(started_swing),'gate_verified_touchdown':dict(confirmed_landing),
     'unplanned_stance_Fz_under_0p2_ticks':int(sum(named_loss.values())),
     'unplanned_stance_losses_by_leg':dict(named_loss),'unplanned_stance_loss_examples':named_low,
     'unplanned_stance_loss_mode_counts':dict(named_stance_loss_by_mode),
     'named_stance_loss_events':named_events,
     'observed_swing_no_contact_ticks_by_leg':dict(lift_free),
     'observed_swing_2mm_mesh_clearance_sample_frames_by_leg':dict(offfloor_two_mm),
     'leg_toe_world_Y_range_mm':{c:round(1000*(leg_foot_xy_max[c]-leg_foot_xy_min[c]),2) for c in CODES},
     'max_actual_tilt_deg':round(max_tilt,4),'min_body_z_m':round(min_root_z,5),
     'max_leg_joint_abs_rad':round(qmax,5),'model_joint_limit_rad':1.7,
     'max_raw_PD_torque_Nm':round(max_raw,5),'max_clipped_torque_Nm':round(max_tau,5),'saturated_ticks':sat_count,
     'strict_5_support_pass':bool(elapsed+1e-8>=seconds and not stop_reason and not low_frames and not named_loss and sat_count==0 and qmax<=1.7 and adv>0),
     'stop_reason':stop_reason,'physics_body_teleported':False,
     'low_frames':low_frames,'bad_segments':bad_segments,'samples':snap,
     'notes':['Fz per named actual tibia floor contact from MuJoCo every 2ms.','No root qpos overwrite or outside balance forces.','Gating is causal using previous physics step; one-tick contact losses may still occur.','Nominal mass/torque/friction are not hardware certified.']}
    if output:
        path=Path(output);path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        if record_2ms:
            trace_path=path.with_suffix('.2ms.json.gz')
            with gzip.open(trace_path,'wt',encoding='utf-8',compresslevel=6) as fh:
                json.dump({'schema':'cad30-wave-gated-2ms-full/1','dt':dt,'Fz_cutoff_N':FZ_CUTOFF,
                    'model_mu':1.1,'frames':raw_frames},fh,separators=(',',':'))
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--mode',choices=['baseline','gated','gate+feedback'],default='gated')
    p.add_argument('--seconds',type=float,default=12.);p.add_argument('--ramp',type=float,default=6.)
    p.add_argument('--out',default='');p.add_argument('--record-2ms',action='store_true');a=p.parse_args()
    result=run(a.mode,a.seconds,a.ramp,output=a.out or None, record_2ms=a.record_2ms)
    print(json.dumps({k:v for k,v in result.items() if k not in ['samples','low_frames','bad_segments','notes']},ensure_ascii=False,indent=2),flush=True)
