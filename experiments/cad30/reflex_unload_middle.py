"""Limited leg-local load sharing REFLEX ABORT GATE, not full four-legged gait.

Uses virtual toe contact forces (mj_contactForce) and a virtual IMU direction;
never uses exact MuJoCo COM or support margin in control decisions. Ground-truth
COM is logged only in the independent evaluator. Front-pair pose preparation
remains the earlier scripted baseline; only the new middle-leg unloading stage
is reactive. The 3 prospective supporting feet receive a one-bit request from
candidate foot; that is a small message bus, not demonstrated Walknet.
"""
from __future__ import annotations
import argparse, json, math
from pathlib import Path
import mujoco
import numpy as np
from front_pair_trick import FRONT, SUPPORT, contact_points, margin
from hexlab.stand_geometry_v04 import load_legs, tip_world, geom_mesh_world_vertices
from hexlab.stand_v04 import static_pose, hold_under_gravity
from hexlab.step_one_leg_v04 import solve_waypoint, smoothstep
HERE=Path(__file__).resolve().parent
DT=.002

def contact_normal_forces(model,data):
    """Per-tibia load proxy in newtons from simulator contact solver."""
    floor=int(model.geom('floor').id)
    mapping={int(model.geom(f'tibia{x}_collision_0').id):x for x in SUPPORT+FRONT}
    loads={x:0. for x in SUPPORT+FRONT}; wrench=np.zeros(6)
    for k in range(data.ncon):
        c=data.contact[k];a,b=int(c.geom1),int(c.geom2)
        other=b if a==floor else a if b==floor else None
        if other not in mapping:continue
        mujoco.mj_contactForce(model,data,k,wrench)
        loads[mapping[other]]+=max(0.,float(wrench[0]))
    return loads

def clamp_rate(current,target,max_step):
    return current+float(np.clip(target-current,-max_step,max_step))

def leg_local_press(load,request,capacity_N,depth_m=.004):
    """One stance leg: press only while receiving unload request and not overloaded."""
    if not request or load>=capacity_N:return 0.
    return depth_m*float(np.clip((capacity_N-load)/(capacity_N*.5),0,1))

def ready_for_swing(my_load,other_forces,other_contacts,front_contacts,imu_tilt_deg,
                    candidate_contact=True, gyro_speed_rad_s=0., force_cutoff_N=.65, support_min_N=.7, tilt_limit_deg=6.):
    """Sensor-only candidate gate. Exact COM is deliberately excluded."""
    if any(x<0 or not np.isfinite(x) for x in [my_load,*other_forces]):return False
    return bool(my_load<force_cutoff_N and len(other_forces)==3 and
       all(x>=support_min_N for x in other_forces) and all(other_contacts)
       and candidate_contact and not any(front_contacts) and imu_tilt_deg<tilt_limit_deg
       and gyro_speed_rad_s < .3)

def experiment(mass='nominal',mu=1.1,candidate='ml',mode='reflex',press_mm=4.,
               force_cutoff_N=.65,seconds=7.8,record=False):
    if mass not in ('nominal','lower','upper'):raise ValueError(mass)
    if candidate not in ('ml','mr'):raise ValueError(candidate)
    if mode not in ('reflex','press_only','no_feedback','forced_lift'):raise ValueError(mode)
    if not(.2<=mu<=1.5 and 0.<=press_mm<=8):raise ValueError('params')
    model=mujoco.MjModel.from_xml_path(str(HERE/'source'/'mjcf'/f'scene_local30_{mass}.xml'))
    model.geom_friction[model.geom('floor').id,0]=mu
    for c in FRONT+SUPPORT:model.geom_friction[model.geom(f'tibia{c}_collision_0').id,0]=mu
    data=mujoco.MjData(model);legs=load_legs(model,data,mujoco);named={leg['suffix']:leg for leg in legs}
    static_pose(model,data,mujoco,legs,.145,.15)
    baseline=hold_under_gravity(model,data,mujoco,data.qpos.copy(),1.)
    if not baseline['stable_by_proxy_thresholds'] or baseline['last_0p4s_min_contact_legs']<6:raise RuntimeError('six-foot-stand failed')
    q0=data.qpos.copy();bid=int(model.body('base_link_2').id)
    init_up=data.xmat[bid].reshape(3,3).T@np.array([0.,0.,1.])
    jids=np.asarray(model.actuator_trnid[:,0],int); qa=np.asarray(model.jnt_qposadr[jids],int); da=np.asarray(model.jnt_dofadr[jids],int)
    iq=data.qpos[qa].copy(); idx={c:np.array([np.where(qa==a)[0][0] for a in named[c]['qadrs']]) for c in SUPPORT+FRONT}
    ref=mujoco.MjData(model); goal={};errs={}
    for c in SUPPORT+FRONT:
        ref.qpos[:]=q0; ref.qvel[:]=0; mujoco.mj_forward(model,ref)
        tip=tip_world(model,ref,named[c]).copy();goals={}
        offsets=({'guard':np.array([0,.05,.105])} if c in FRONT else
                {'shift':np.array([0,.04,0]), 'press':np.array([0,.04,-press_mm/1000.]), 'boost':np.array([0,.08,-.020]), 'boost_press':np.array([0,.08,-.020-press_mm/1000.])})
        if c==candidate:offsets['swing']=np.array([0,.04,.012])
        for name,delta in offsets.items():
            g,e=solve_waypoint(model,ref,mujoco,named[c],tip+delta,q0[named[c]['qadrs']]);goals[name]=g;errs[c+':'+name]=float(e)
        goal[c]=goals
    if max(errs.values())>.008:return {'complete':False,'reason':'ik_inaccessible','worst_ik_err_m':max(errs.values()),'ik_errors_m':errs}
    maxtau=0; sat=0; initialized=False; release=None; forward_offset=0.; ready_dwell=0.; max_dwell=0.; loads_filtered={c:0. for c in SUPPORT+FRONT}; actuator_adjust={c:0. for c in SUPPORT if c!=candidate}
    n=round(seconds/model.opt.timestep); rows=[];states=[];lost3=0;fail=None
    denom=len(actuator_adjust); body_start=data.xpos[bid].copy(); pressure_integral=0.
    for k in range(n):
        t=k*model.opt.timestep;cp=contact_points(model,data)
        f=contact_normal_forces(model,data)
        if not initialized:loads_filtered=dict(f);initialized=True
        alpha=1-math.exp(-model.opt.timestep/.05)
        for c in loads_filtered:loads_filtered[c]+=alpha*(f[c]-loads_filtered[c])
        rot=data.xmat[bid].reshape(3,3)
        tilt=math.degrees(math.acos(float(np.clip((rot@init_up)[2],-1,1))))
        request=t>=3.6
        others=[c for c in SUPPORT if c!=candidate]
        # All support legs independently pressure-adjust their own actuator goals
        # in response to request and their own filtered load.
        capacity=float(model.body_mass.sum())*9.81/3*1.23
        if mode in ('reflex','press_only'):
            for c in others:
                desire=leg_local_press(loads_filtered[c],request,capacity,press_mm/1000.)
                actuator_adjust[c]=clamp_rate(actuator_adjust[c],desire,model.opt.timestep*.008)
        if mode=='reflex' and request and release is None:
            still_heavy = loads_filtered[candidate] > force_cutoff_N
            tilt_safe = tilt < 6.
            front_clear = not any(cp[c] for c in FRONT)
            desired = .04 if (still_heavy and tilt_safe and front_clear and t<6.3) else forward_offset
            forward_offset = clamp_rate(forward_offset,desired,model.opt.timestep*.020)
        if mode in ('reflex','press_only') and release is None and t>=4.4:
            seen_ready=ready_for_swing(loads_filtered[candidate],[loads_filtered[c] for c in others],
                               [bool(cp[c]) for c in others],[bool(cp[c]) for c in FRONT],tilt,
                               candidate_contact=bool(cp[candidate]),
                               gyro_speed_rad_s=float(np.linalg.norm(data.qvel[3:6])),
                               force_cutoff_N=force_cutoff_N)
            ready_dwell=ready_dwell+model.opt.timestep if seen_ready else 0.
            max_dwell=max(max_dwell,ready_dwell)
            if ready_dwell>=.15:release=t
        if mode=='forced_lift' and release is None and t>=4.4:release=t
        cmd=iq.copy()
        support_alpha=smoothstep(min(1,t/2.))
        front_alpha=smoothstep(max(0,min(1,(t-2.)/1.2)))
        for c in SUPPORT:
            ix=idx[c]; q=iq[ix]+support_alpha*(goal[c]['shift']-iq[ix])
            if mode=='reflex' and t>=3.6:
                q+=(forward_offset/.04)*(goal[c]['boost']-goal[c]['shift'])
            if c in actuator_adjust and mode in ('reflex','press_only') and press_mm>0:
                press_delta=goal[c]['press']-goal[c]['shift']
                if mode=='reflex':
                    press_delta+=(forward_offset/.04)*((goal[c]['boost_press']-goal[c]['boost'])-press_delta)
                q+=(actuator_adjust[c]/(press_mm/1000.))*press_delta
            if c==candidate and release is not None:
                # Gentle controlled swing (no contact predicate on this segment).
                a=smoothstep(min(1,max(0,(t-release)/1.0)))
                q+=(goal[c]['swing']-goal[c]['shift'])*a
            cmd[ix]=q
        for c in FRONT:
            ix=idx[c];cmd[ix]=iq[ix]+front_alpha*(goal[c]['guard']-iq[ix])
        tau_unc=7*(cmd-data.qpos[qa])-.32*data.qvel[da]
        sat+=int(np.any(tau_unc>model.actuator_ctrlrange[:,1]+1e-8) or np.any(tau_unc<model.actuator_ctrlrange[:,0]-1e-8))
        data.ctrl[:]=np.clip(tau_unc,model.actuator_ctrlrange[:,0],model.actuator_ctrlrange[:,1]);maxtau=max(maxtau,float(np.max(abs(data.ctrl))))
        mujoco.mj_step(model,data)
        pressure_integral+=sum(actuator_adjust.values())*model.opt.timestep
        if k%10==0 or k==n-1:
            cp=contact_points(model,data);nf=contact_normal_forces(model,data)
            com=data.subtree_com[bid][:2];pts=[p for c in others for p in cp[c]]
            oracle_margin=margin(com,pts) if len(pts)>=3 else None
            mesh_z={c:float(np.min(geom_mesh_world_vertices(model,data,named[c]['geom'])[:,2])) for c in [candidate,*FRONT]}
            support=[c for c in SUPPORT if cp[c]];front=[c for c in FRONT if cp[c]]
            if t>3.6 and len(support)<3:lost3+=1
            if record and (k%50==0 or k==n-1):states.append({'t':round(t,3),'qpos':np.round(data.qpos,7).tolist()})
            rows.append({'t':round(t,3),'load_N':{c:round(loads_filtered[c],3) for c in SUPPORT},
                         'foot_contacts':support,'front_contacts':front,'z_min_m':{c:round(v,4) for c,v in mesh_z.items()},
                         'oracle_margin_m':None if oracle_margin is None else round(oracle_margin,5),
                         'torque_nm':round(float(np.max(abs(data.ctrl))),4), 'tilt_deg':round(tilt,3),
                         'body_xyz':np.round(data.xpos[bid],4).tolist(),
                         'press_mm':{c:round(1000*actuator_adjust[c],2) for c in others}})
            if data.xpos[bid][2]<.07 or tilt>35 or not np.isfinite(data.qpos).all():fail='fall_or_nonfinite';break
    evaluation=[r for r in rows if r['t']>4.45]
    post_release=[r for r in rows if release is not None and release+.2<r['t']<release+.8]
    return {'mode':mode,'mass':mass,'mu':mu,'candidate':candidate,'nominal_mass_kg':float(model.body_mass.sum()),
            'commanded_initial_shift_m':.04, 'pressure_max_allowed_mm':press_mm,
            'force_cutoff_N':force_cutoff_N,'maximum_extra_forward_shift_m':.04,'achieved_extra_forward_shift_m':round(forward_offset,5),'max_sensor_ready_dwell_s':round(max_dwell,3),'virtual_sensor':'MuJoCo contact-normal at each tibia, EMA 50ms',
            'uses_COM_in_controller':False,'release_at_s':None if release is None else round(release,3),
            'complete':fail is None and k==n-1,'failure':fail,'min_load_candidate_after_4p45_N':round(min((r['load_N'][candidate] for r in evaluation),default=float('nan')),3),
            'max_press_mm':round(max([max(r['press_mm'].values()) for r in evaluation],default=0.),2),
            'mean_oracle_margin_m':round(float(np.mean([r['oracle_margin_m'] for r in evaluation if r['oracle_margin_m'] is not None])),5) if any(r['oracle_margin_m'] is not None for r in evaluation) else None,
            'mean_candidate_load_final_1s_N':round(float(np.mean([r['load_N'][candidate] for r in rows if r['t']>=seconds-1])),3),
            'mean_oracle_margin_final_1s_m':round(float(np.mean([r['oracle_margin_m'] for r in rows if r['t']>=seconds-1 and r['oracle_margin_m'] is not None])),5) if any(r['t']>=seconds-1 and r['oracle_margin_m'] is not None for r in rows) else None,
            'oracle_positive_sample_count':sum(r['oracle_margin_m'] is not None and r['oracle_margin_m']>0 for r in evaluation),
            'min_support_contacts_after_request':min((len(r['foot_contacts']) for r in evaluation),default=-1),
            'front_any_contact_after_request':any(bool(r['front_contacts']) for r in evaluation),
            'actual_middle_mesh_clearance_max_m':round(max((r['z_min_m'][candidate] for r in evaluation),default=float('nan')),5),
            'issued_swing_command_with_three_contacts':bool(post_release and all(len(r['foot_contacts'])>=3 and not r['front_contacts'] for r in post_release)),
            'saturation_fraction':round(sat/max(1,n),5),'peak_motor_torque_Nm':round(maxtau,4),
            'pressure_integral_m_s':round(pressure_integral,5),'body_delta_xyz_m':np.round(data.xpos[bid]-body_start,4).tolist(),
            'scoring_note':'A blocked lift counts as a safe refusal, NOT walking; COM oracle for evaluation only',
            'trace':rows if record else [],'qpos_frames':states if record else []}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--mass',default='nominal');p.add_argument('--mu',type=float,default=1.1)
    p.add_argument('--candidate',default='ml');p.add_argument('--mode',choices=['reflex','press_only','no_feedback','forced_lift'],default='reflex')
    p.add_argument('--press-mm',type=float,default=4.);p.add_argument('--record',action='store_true');p.add_argument('--out')
    a=p.parse_args();out=a.out;d=vars(a);d.pop('out');r=experiment(**d);print(json.dumps({k:v for k,v in r.items() if k not in ('trace','qpos_frames')},ensure_ascii=False,indent=2));
    if out:Path(out).write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
