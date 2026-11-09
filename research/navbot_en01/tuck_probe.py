"""Bounded deterministic self-righting command study; no root writes after setup."""
from __future__ import annotations
import argparse, hashlib, json, math
from pathlib import Path
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from .getup_model import cad_floor_minimum
from .tuck_commands import Sequence
from .linkage_model import set_configuration, load_evidence

HERE = Path(__file__).resolve().parent
PROTOCOL = HERE/'tuck_protocol.json'
SIDES = ('left', 'right')


def shape_minimum(m, d):
    """Support function of every active collision proxy, excluding world floor."""
    values = []
    for g in range(m.ngeom):
        if m.geom_bodyid[g] == 0: continue
        if not (int(m.geom_conaffinity[g]) & 1 or int(m.geom_contype[g]) & 2): continue
        R = d.geom_xmat[g].reshape(3,3)
        size, typ = m.geom_size[g], int(m.geom_type[g])
        center = float(d.geom_xpos[g,2])
        if typ == mujoco.mjtGeom.mjGEOM_MESH:
            mesh = m.geom_dataid[g]
            v = m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh]+m.mesh_vertnum[mesh]]
            values.append(center+float((v@R[2]).min()))
        elif typ == mujoco.mjtGeom.mjGEOM_BOX:
            values.append(center-float(np.dot(np.abs(R[2]),size)))
        elif typ == mujoco.mjtGeom.mjGEOM_CYLINDER:
            az=min(1.,abs(float(R[2,2])))
            values.append(center-size[1]*az-size[0]*math.sqrt(max(0.,1-az*az)))
        elif typ == mujoco.mjtGeom.mjGEOM_CAPSULE:
            values.append(center-size[0]-size[1]*abs(float(R[2,2])))
        elif typ == mujoco.mjtGeom.mjGEOM_SPHERE:
            values.append(center-size[0])
        else: raise ValueError('Unhandled collision shape')
    return min(values)


def setup(m, d, orientation, mesh):
    points=load_evidence()['sides']['cad_z_positive']['points_xy_mm']
    set_configuration(m,d,points,.1)
    axis, angle = {'front':('y',90), 'back':('y',-90),
                   'left':('x',90), 'right':('x',-90), 'upright':('y',0)}[orientation]
    quat=Rotation.from_euler(axis,angle,degrees=True).as_quat()
    d.qpos[3:7]=quat[[3,0,1,2]]
    mujoco.mj_forward(m,d)
    lowest=min(shape_minimum(m,d),cad_floor_minimum(mesh,m,d))
    d.qpos[2]+=.0005-lowest
    mujoco.mj_forward(m,d)
    return {'qpos':d.qpos.tolist(), 'qvel':d.qvel.tolist(),
            'cad_min_z_m':cad_floor_minimum(mesh,m,d),
            'collision_min_z_m':shape_minimum(m,d)}


def stable_gate(tilt, loads, nonwheel, speed, angular, a):
    return (tilt <= math.radians(a['max_tilt_deg']) and
            min(loads)>=a['min_each_wheel_normal_force_n'] and
            nonwheel<=a['max_nonwheel_normal_force_n'] and
            speed<=a['max_axle_speed_m_s'] and angular<=a['max_body_angular_speed_rad_s'])


def run(out, *, orientation, strategy, model_path, mesh_path):
    p=json.loads(PROTOCOL.read_text()); a=p['acceptance']; seq=Sequence(**p['cases'][strategy])
    if orientation not in p['orientations']+['upright'] or strategy not in p['strategies']:
        raise ValueError('Undeclared case')
    if mujoco.__version__!='3.14.0': raise ValueError('Wrong physics version')
    xml=Path(model_path).read_text();m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m)
    dt=float(m.opt.timestep)
    if dt!=p['timestep_s']:raise ValueError('Wrong timestep')
    jid=lambda n:mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n)
    aid=lambda n:mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_ACTUATOR,n)
    bid=lambda n:mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,n)
    gid=lambda n:mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,n)
    sid=lambda n:mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,n)
    torso,floor=bid('torso'),gid('floor')
    wb=[bid(s+'_wheel') for s in SIDES]
    # Original CAD wheel parts, not disabled generic cylinders, carry the load.
    wg=[[gid('floorcad_'+str(pid)) for pid in ids] for ids in ((21,64),(19,65))]
    hq=[int(m.jnt_qposadr[jid('hip_'+s)]) for s in SIDES]
    hv=[int(m.jnt_dofadr[jid('hip_'+s)]) for s in SIDES]
    wq=[int(m.jnt_qposadr[jid('wheel_'+s)]) for s in SIDES]
    wv=[int(m.jnt_dofadr[jid('wheel_'+s)]) for s in SIDES]
    ha=[aid('a_hip_'+s) for s in SIDES];wa=[aid('a_wheel_'+s) for s in SIDES]
    initial=setup(m,d,orientation,mesh_path)
    body_names=[mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_BODY,b) for b in range(1,m.nbody)]
    joints={mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_JOINT,j):int(m.jnt_qposadr[j]) for j in range(m.njnt)}
    period=round(p['control_period_s_ASSUMED']/dt);steps=round(p['duration_s']/dt)
    frames=[];force=np.zeros(6);loads=[0.,0.];nonwheel=0.;captured=False;events=[];xref=last_rate=0.
    goals=[.1,.1];torques=[0.,0.];phase='settle'
    hip_min=np.full(2,np.inf);hip_max=np.full(2,-np.inf)
    peak_hip=peak_wheel=max_floor_pen=max_self_pen=max_closure=0.
    final_stable_steps=longest=current=0;first_stable=None;tilt_at_command=None
    minimum_cad_z=initial['cad_min_z_m'];previous_axle=np.mean(d.xpos[wb],axis=0)
    root_before=initial['qpos'][:7];physics_calls=0;min_active_tilt=math.pi;applied_impulse=np.zeros(2)
    for step in range(steps):
        t=float(d.time);R=d.xmat[torso].reshape(3,3)
        tilt=math.acos(float(np.clip(R[2,2],-1,1)))
        if step%period==0:
            if t>=p['settle_s'] and tilt_at_command is None:tilt_at_command=tilt
            can_balance = strategy=='balance' or (seq.pulse_at is not None and t-p['settle_s'] >= seq.pulse_at+seq.pulse_s)
            if strategy!='idle' and can_balance:
                if not captured and t>=p['settle_s'] and tilt<math.radians(20) and min(loads)>.15:
                    captured=True;xref=.0185*float(np.mean(d.qpos[wq]));events.append({'t':t,'event':'capture'})
                if captured and tilt>math.radians(35):
                    captured=False;events.append({'t':t,'event':'capture-lost'})
            if t<p['settle_s'] or strategy=='idle':
                goals,torques=[.1,.1],[0.,0.];phase='settle' if t<p['settle_s'] else 'idle'
            elif captured:
                phase='balance';goals=[.1,.1]
                pitch=math.atan2(-R[2,0],R[2,2]);gyro=float(d.sensor('imu_gyro').data[1])
                rate=float(np.mean(d.qvel[wv]));odom=.0185*float(np.mean(d.qpos[wq]))
                disabled=abs(2*rate-last_rate)>10 or abs(2*rate)>50
                if disabled:xref=odom
                last_rate=2*rate
                common=.15*pitch+.05*gyro+(0. if disabled else .008*rate+.3*(odom-xref))
                torques=[float(np.clip(common,-.03,.03))]*2
            else:
                old_pulse=seq.pulse_at
                goals,torques,phase=seq.command(t-p['settle_s'],d.qpos[hq],d.qvel[hv])
                if old_pulse is None and seq.pulse_at is not None:
                    events.append({'t':t,'event':'legs-positioned-before-pulse','q':d.qpos[hq].tolist(),
                                   'qvel':d.qvel[hv].tolist(),'target':seq.target,
                                   'tilt_deg':math.degrees(tilt),'wheel_loads_n':loads[:]})

        for i in range(2):
            q,v=float(d.qpos[hq[i]]),float(d.qvel[hv[i]])
            raw=3*(goals[i]-q)-.045*v
            limit=.14709975*(max(0.,1-abs(v)/11.6355) if raw*v>0 else 1)
            d.ctrl[ha[i]]=np.clip(raw,-limit,limit);d.ctrl[wa[i]]=torques[i]
        d.xfrc_applied[:]=0;d.qfrc_applied[:]=0
        applied_impulse+=d.ctrl[wa]*dt
        mujoco.mj_step(m,d);physics_calls+=1;mujoco.mj_forward(m,d)
        if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all():raise RuntimeError('Nonfinite physics')
        R=d.xmat[torso].reshape(3,3);tilt=math.acos(float(np.clip(R[2,2],-1,1)))
        pitch=math.atan2(-R[2,0],R[2,2]);roll=math.atan2(R[2,1],R[2,2])
        angular=float(np.linalg.norm(d.sensor('imu_gyro').data))
        axle=np.mean(d.xpos[wb],axis=0);speed=float(np.linalg.norm((axle-previous_axle)[:2])/dt);previous_axle=axle.copy()
        loads=[0.,0.];nonwheel=normal=0.;contact_names=[];floor_pen=self_pen=0.
        for ci in range(d.ncon):
            c=d.contact[ci]
            if floor not in (int(c.geom1),int(c.geom2)):
                self_pen=max(self_pen,max(0.,-float(c.dist)));continue
            other=int(c.geom2) if int(c.geom1)==floor else int(c.geom1)
            mujoco.mj_contactForce(m,d,ci,force);fn=max(0.,float(force[0]));normal+=fn
            if fn>.0001:contact_names.append(mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,other))
            if other in wg[0]:loads[0]+=fn
            elif other in wg[1]:loads[1]+=fn
            else:nonwheel+=fn
            floor_pen=max(floor_pen,max(0.,-float(c.dist)))
        closure=max(float(np.linalg.norm(d.site_xpos[sid(s+'_closure_calf')]-d.site_xpos[sid(s+'_closure_rod')])) for s in SIDES)
        hip_min=np.minimum(hip_min,d.qpos[hq]);hip_max=np.maximum(hip_max,d.qpos[hq])
        peak_hip=max(peak_hip,float(np.max(np.abs(d.ctrl[ha]))));peak_wheel=max(peak_wheel,float(np.max(np.abs(d.ctrl[wa]))))
        max_floor_pen=max(max_floor_pen,floor_pen);max_self_pen=max(max_self_pen,self_pen);max_closure=max(max_closure,closure)
        if d.time>=p['settle_s']:min_active_tilt=min(min_active_tilt,tilt)
        stable=stable_gate(tilt,loads,nonwheel,speed,angular,a)
        current=current+1 if stable and d.time>=p['settle_s'] else 0
        longest=max(longest,current)
        if current and first_stable is None:first_stable=float(d.time)
        if d.time>p['duration_s']-a['final_stable_s']:final_stable_steps+=int(stable)
        if step%80==0:
            cad_min=cad_floor_minimum(mesh_path,m,d);minimum_cad_z=min(minimum_cad_z,cad_min)
            frames.append({'t':float(d.time),'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist(),'ctrl':d.ctrl.tolist(),
                'body_poses':[[*d.xpos[b].tolist(),*d.xquat[b].tolist()] for b in range(1,m.nbody)],
                'ground_contact':bool(normal>.05),'nonwheel_ground_contact':bool(nonwheel>.05),
                'contact_geoms':contact_names,'ground_normal_force_n':normal,'pitch_rad':pitch,'roll_rad':roll,
                'tilt_rad':tilt,'closure_error_m':closure,'wheel_normal_force_n':loads[:],
                'nonwheel_normal_force_n':nonwheel,'axle_speed_m_s':speed,'body_angular_speed_rad_s':angular,
                'hip_targets_rad':goals[:],'phase':phase,'stable_gate':stable,'cad_floor_minimum_z_m':cad_min})
    warnings={str(i):int(d.warning[i].number) for i in range(len(d.warning)) if d.warning[i].number}
    checks={'fallen_at_command':orientation=='upright' or (tilt_at_command is not None and tilt_at_command>math.radians(60)),
            'final_two_seconds_stable':final_stable_steps>=round(a['final_stable_s']/dt)-1,
            'hip_bounds':bool(min(hip_min)>=a['operating_q_min_rad'] and max(hip_max)<=a['operating_q_max_rad']),
            'floor_contact_penetration':max_floor_pen<=a['max_floor_penetration_m'],
            'original_cad_floor_penetration':minimum_cad_z>=-a['max_floor_penetration_m'],
            'self_proxy_penetration':max_self_pen<=a['max_self_proxy_penetration_m'],
            'linkage_closure':max_closure<=a['max_closure_error_m'],'no_warnings':not warnings}
    summary={'schema':'en01-fourbar-diagnostic/v1','experiment':'tuck_impulse_v8','mode':'getup',
        'orientation':orientation,'strategy':strategy,'duration_s':6,'timestep_s':dt,'physics_steps':steps,
        'mujoco_version':mujoco.__version__,'model_sha256':hashlib.sha256(xml.encode()).hexdigest(),
        'geometry_source_blob':load_evidence()['source']['git_blob_sha1'],'model_profile':'cad_floor_v7_unchanged',
        'protocol_sha256':hashlib.sha256(PROTOCOL.read_bytes()).hexdigest(),
        'controller_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'total_mass_kg':float(sum(m.body_mass)),'hip_continuous_ceiling_nm':.14709975,'wheel_torque_ceiling_nm':.03,
        'peak_hip_torque_nm':peak_hip,'peak_wheel_torque_nm':peak_wheel,'hip_min_all_steps_rad':hip_min.tolist(),
        'hip_max_all_steps_rad':hip_max.tolist(),'max_floor_penetration_m':max_floor_pen,
        'max_self_proxy_penetration_m':max_self_pen,'max_closure_error_m':max_closure,
        'minimum_sampled_cad_z_m':minimum_cad_z,'tilt_at_command_deg':math.degrees(tilt_at_command or 0),
        'min_active_tilt_deg':math.degrees(min_active_tilt),'signed_wheel_impulse_nm_s':applied_impulse.tolist(),'final_tilt_deg':math.degrees(tilt),'longest_stable_s':longest*dt,'final_stable_s':current*dt,
        'final_gate_pass_fraction':final_stable_steps/round(a['final_stable_s']/dt),
        'first_stable_s':first_stable,'capture_events':events,'sequence':{'target_q_rad':seq.target,'pulse_sign':seq.sign,'pulse_duration_s':seq.pulse_s,'pulse_start_s':None if seq.pulse_at is None else seq.pulse_at+p['settle_s'],'position_timeout':seq.timed_out},'checks':checks,'getup_pass':all(checks.values()),
        'warnings':warnings,'frame_count':len(frames),'upright_6s':False,'hardware_motion_claim_allowed':False,
        'physics_integrity':{'step_calls':physics_calls,'runtime_root_resets':0,'external_forces':False}}
    record={'summary':summary,'initial_state':initial,'body_names':body_names,'joint_qpos_addresses':joints,'frames':frames}
    out=Path(out);out.parent.mkdir(parents=True,exist_ok=True)
    payload=json.dumps(record,allow_nan=False,separators=(',',':'))+'\n'
    if out.exists() and out.read_text()!=payload:raise ValueError('Immutable trace conflict')
    out.write_text(payload)
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for n in ('out','model','cad-mesh'):parser.add_argument('--'+n,type=Path,required=True)
    parser.add_argument('--orientation',required=True);parser.add_argument('--strategy',required=True)
    a=parser.parse_args()
    print(json.dumps(run(a.out,orientation=a.orientation,strategy=a.strategy,model_path=a.model,mesh_path=a.cad_mesh),indent=2))
