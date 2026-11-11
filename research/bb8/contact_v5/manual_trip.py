"""Manual trips with an explicit OPTIONAL passive floor-loss hypothesis.

CAD, inertia, constraints and motor actuators remain as in v3. The optional
external floor torque always removes energy and is NOT a motor controller.
All resistance coefficients are scenarios, not measured floor properties.
"""
from pathlib import Path
import argparse,hashlib,json,math,time
import numpy as np
import mujoco

BACK_EMF=.596/(330*35/19*math.pi/30)

def floor_torque(omega_world,normal_N,resistance_m):
    omega_t=np.array([omega_world[0],omega_world[1],0.])
    return -resistance_m*max(0.,normal_N)*omega_t/max(.02,float(np.linalg.norm(omega_t)))

def command(t,knots):
    return np.array([np.interp(t,[r[0] for r in knots],[r[j] for r in knots]) for j in [1,2]])

def run(model,initial,knots,duration=18.,dt=.001,resistance_m=0.):
    if abs(model.opt.timestep-dt)>1e-12:raise ValueError('Model timestep does not match protocol')
    if any(abs(v)>.2 for row in knots for v in row[1:]):raise ValueError('Command exceeds declared wheel effort limit')
    d=mujoco.MjData(model);d.qpos[:]=initial;mujoco.mj_forward(model,d)
    sid=model.body('shell').id;cid=model.body('chassis').id
    names=[model.geom(i).name for i in range(model.ngeom)]
    if resistance_m<0:raise ValueError('Passive resistance length must be nonnegative')
    frames=[];steps=round(duration/dt);maxpen=0.;max_tilt=0.;max_tau=0.;counts={'left':0,'right':0}
    dissipated=0.;max_loss_power=0.
    request=np.zeros(2);start=None;end=None;slip2=0.;slipn=0;clock=time.monotonic()
    for k in range(steps+1):
        t=k*dt
        if k%10==0:request=command(t,knots)
        d.ctrl[:]=np.clip(request-BACK_EMF*d.qvel[12:14],-.2,.2)
        d.xfrc_applied[:]=0.
        mujoco.mj_forward(model,d)
        # Ground-only constitutive loss, not propulsion. Normal load is measured
        # before adding this small tangential moment; the lag-free law is passive.
        ground_load=0.;ground_force=np.zeros(6)
        for j,c in enumerate(d.contact):
            pair={names[c.geom1],names[c.geom2]}
            if pair=={'floor','shell_outer'}:
                mujoco.mj_contactForce(model,d,j,ground_force)
                ground_load+=max(0.,float(ground_force[0]))
        shell_velocity=np.zeros(6)
        mujoco.mj_objectVelocity(model,d,mujoco.mjtObj.mjOBJ_BODY,sid,shell_velocity,0)
        loss=floor_torque(shell_velocity[:3],ground_load,resistance_m)
        power=float(loss@shell_velocity[:3])
        if power>1e-14:raise RuntimeError('Floor law injected energy')
        max_loss_power=max(max_loss_power,power)
        if k<steps:dissipated-=power*dt
        d.xfrc_applied[sid,3:]=loss
        if resistance_m:mujoco.mj_forward(model,d)
        tilt=math.degrees(math.acos(float(np.clip(d.xmat[cid].reshape(3,3)[2,2],-1,1))))
        max_tilt=max(max_tilt,tilt);max_tau=max(max_tau,float(np.abs(d.actuator_force).max()))
        sides=set();cad=set();normal_force={'left':0.,'right':0.,'frame':0.};force=np.zeros(6)
        for j,c in enumerate(d.contact):
            a,b=names[c.geom1],names[c.geom2]
            if not(a.startswith('cavity') or b.startswith('cavity')):continue
            maxpen=max(maxpen,-float(c.dist))
            for s in counts:
                if a.startswith(s+'_ring') or b.startswith(s+'_ring'):
                    sides.add(s)
                    if k%20==0:mujoco.mj_contactForce(model,d,j,force);normal_force[s]+=float(force[0])
            if k%20==0:
                for name in [a,b]:
                    if name.startswith('cad_'):
                        cad.add(name);mujoco.mj_contactForce(model,d,j,force);normal_force['frame']+=float(force[0])
        for s in sides:counts[s]+=1
        # mj_objectVelocity explicitly supplies WORLD coordinates for an independent floor-rolling check.
        velocity=np.zeros(6);mujoco.mj_objectVelocity(model,d,mujoco.mjtObj.mjOBJ_BODY,sid,velocity,0)
        floor_residual=velocity[3:5]-np.array([.1*velocity[1],-.1*velocity[0]])
        if 3<=t<=9:slip2+=float(floor_residual@floor_residual);slipn+=1
        if k==round(2/dt):start=d.qpos[:2].copy()
        if k==round(10/dt):end=d.qpos[:2].copy()
        if k%20==0:
            frames.append({'t':round(t,8),'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist(),'ctrl':d.ctrl.tolist(),
                'manual_effort_Nm':request.tolist(),'full_tilt_deg':tilt,'wheel_contact_sides':sorted(sides),
                'floor_loss_torque_Nm':loss.tolist(),'floor_loss_power_W':power,'floor_normal_N_before_loss':ground_load,
                'CAD_contact_parts':sorted(cad),'contact_normal_N':normal_force,
                'imu_gyro_rad_s':d.sensor('imu_gyro').data.tolist(),'imu_accel_m_s2':d.sensor('imu_accel').data.tolist(),
                'encoder_rad':[float(d.sensor(s+'_encoder').data[0]) for s in ['left','right']],
                'shell_world_velocity':velocity.tolist(),'floor_rolling_residual_m_s':floor_residual.tolist()})
        if k<steps:mujoco.mj_step(model,d)
        if not np.isfinite(d.qpos).all() or max(d.warning.number)>0 or abs(d.time-min(k+1,steps)*dt)>1e-7:
            raise RuntimeError('Non-finite state, warning, or integrator reset')
    tail=[f for f in frames if f['t']>=duration-2]
    tail_xy=np.array([f['qpos'][:2] for f in tail]);tail_v=np.array([f['qvel'][:2] for f in tail])
    displacement=end-start if end is not None and start is not None else np.zeros(2)
    return {'summary':{'drive_displacement_2_to_10s_m':displacement.tolist(),'max_full_tilt_deg':max_tilt,
             'final_xy_m':d.qpos[:2].tolist(),'final_speed_m_s':float(np.linalg.norm(d.qvel[:2])),
             'tail_speed_rms_m_s':float(np.sqrt(np.mean(np.sum(tail_v**2,axis=1)))),
             'tail_position_range_m':np.ptp(tail_xy,axis=0).tolist(),
             'wheel_contact_fraction':{s:n/(steps+1) for s,n in counts.items()},
             'max_contact_penetration_m':maxpen,'floor_rolling_residual_rms_m_s':math.sqrt(slip2/max(1,slipn)),
             'max_abs_motor_torque_Nm':max_tau,'warnings':d.warning.number.tolist(),'steps':steps,
             'floor_loss_length_m_ASSUMED':resistance_m,'floor_loss_energy_J':dissipated,'max_floor_loss_power_W':max_loss_power,
             'runtime_s':time.monotonic()-clock},'frames':frames}

def main():
    p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--profile',default='nominal');p.add_argument('--efforts',type=float,nargs='+',default=[.04,-.04])
    p.add_argument('--resistance',type=float,default=0.);p.add_argument('--seconds',type=float,default=18.)
    p.add_argument('--stop-bias',type=float,default=0.);p.add_argument('--allow-estimated-mass',action='store_true')
    a=p.parse_args()
    if not a.allow_estimated_mass:p.error('Uncalibrated hardware masses require --allow-estimated-mass')
    a.out.mkdir(parents=True,exist_ok=True)
    xml=a.baseline/(a.profile+'-model.xml');m=mujoco.MjModel.from_xml_path(str(xml))
    prior=json.loads((a.baseline/('hold-'+a.profile+'.json')).read_text())
    initial=np.array(prior['frames'][0]['qpos'])
    initial[7:10]=np.array([0.,0.,.1001])+prior['model']['initial_rigid_translation_m']
    initial[10:14]=[1,0,0,0]
    for effort in a.efforts:
        knots=[[0,0,0],[.1,effort,effort],[8,effort,effort],[9,a.stop_bias,a.stop_bias],[a.seconds,a.stop_bias,a.stop_bias]]
        r=run(m,initial,knots,duration=a.seconds,resistance_m=a.resistance)
        r.update(schema='alfred-bb8-manual-drive-floor-loss/v5',hardware_calibrated=False,head_in_dynamics=False,
                 profile=a.profile,engine_version=mujoco.__version__,knots=knots,
                 model_sha256=hashlib.sha256(xml.read_bytes()).hexdigest(),source_v3_model_metadata=prior['model'],
                 model_changes='Runtime passive floor-loss law added when resistance_m>0; base XML, all CAD contacts and inertias unchanged',
                 initial_condition='Held upright before release, not the v3 ball-center COM angle',
                 floor_loss_law='tau_xy = -b*N_ground*omega_xy/max(norm(omega_xy),0.02 rad/s); no force without floor contact',
                 floor_loss_length_m_ASSUMED=a.resistance,
                 feedback='Manual pre-recorded motor commands; physical back EMF and passive environment law, no body-state motor feedback')
        path=a.out/(a.profile+'-'+str(effort)+'-b-'+str(a.resistance)+'.json')
        path.write_text(json.dumps(r,separators=(',',':'))+'\n')
        print(json.dumps({'file':path.name,**r['summary']}),flush=True)

if __name__=='__main__':main()
