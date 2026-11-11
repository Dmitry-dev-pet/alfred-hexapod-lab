"""Component-mass contact model and a sampled IMU-feedback experiment."""
from pathlib import Path
import argparse,hashlib,json,math,sys,time
import xml.etree.ElementTree as E
import numpy as np
import mujoco
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'contact_v2'))
from model import make_model,fmt
from audit_geometry import load_parts
from masses import build,axle,WHEELS
from imu_control import Controller

def zero_angle(profile,fit):
    moment=np.zeros(3);mass=0.
    for group in ['chassis','left','right']:
        g=profile['groups'][group];pos=np.array(g['center_m'])+fit+(axle(group) if group!='chassis' else 0)
        moment+=g['mass_kg']*pos;mass+=g['mass_kg']
    com=moment/mass
    return math.atan2(com[0],-com[2]),com

def make(profile,parts,fit,level=5,dt=.001,peak=.12,collision='facets'):
    xml,meta=make_model(level=level,wall=profile['wall_m'],timestep=dt,parts=parts,fit_translation=fit,allow_assumed_mass=True)
    root=E.fromstring(xml)
    if collision=='sdf':
        # First-party MuJoCo analytic SDFs replace faceted cavity/capsule seams.
        # Bowl height=radius has width=0, giving abs(norm(p)-radius)-halfwall.
        assets=root.find('asset');shell=root.find('.//body[@name="shell"]')
        for mesh in list(assets):
            if mesh.get('name','').startswith('wedge'):assets.remove(mesh)
        for g in list(shell.findall('geom')):
            if g.get('name','').startswith('cavity'):shell.remove(g)
        ext=E.SubElement(root,'extension')
        for plugin,name,config in [('mujoco.sdf.bowl','spherical_wall',{'height':.1-profile['wall_m']/2,'radius':.1-profile['wall_m']/2,'thickness':profile['wall_m']/2}),
                                   ('mujoco.sdf.torus','source_torus',{'radius1':.025,'radius2':.0025})]:
            plug=E.SubElement(ext,'plugin',plugin=plugin);inst=E.SubElement(plug,'instance',name=name)
            for k,v in config.items():E.SubElement(inst,'config',key=k,value=str(v))
            mesh=E.SubElement(assets,'mesh',name=name);E.SubElement(mesh,'plugin',instance=name)
        g=E.SubElement(shell,'geom',name='cavity_sdf',type='sdf',mesh='spherical_wall',contype='2',conaffinity='12',rgba='.5 .65 .8 .15')
        E.SubElement(g,'plugin',instance='spherical_wall')
        from model import RINGS,AXLE
        for side in ['left','right']:
            body=root.find('.//body[@name="'+side+'_wheel"]')
            for g in list(body.findall('geom')):
                if g.get('name','').startswith(side+'_ring'):body.remove(g)
            for j,center in enumerate(RINGS[side]):
                g=E.SubElement(body,'geom',name=side+'_ring'+str(j),type='sdf',mesh='source_torus',
                               pos=fmt(np.array(center)-axle(side)),quat=fmt([math.sqrt(.5),math.sqrt(.5),0,0]),contype='4',conaffinity='2',rgba='.15 .15 .15 1')
                E.SubElement(g,'plugin',instance='source_torus')
        root.find('option').set('sdf_iterations','20');root.find('option').set('sdf_initpoints','40')
    for group,spec in profile['groups'].items():
        b=root.find('.//body[@name="'+(group+'_wheel' if group in ['left','right'] else group)+'"]')
        old=b.find('inertial');b.remove(old);I=np.array(spec['inertia_COM_kg_m2'])
        E.SubElement(b,'inertial',pos=fmt(spec['center_m']),mass=str(spec['mass_kg']),fullinertia=fmt([I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]]))
    chassis=root.find('.//body[@name="chassis"]')
    center=next(c['center_m'] for c in profile['components'] if c['id']=='c48')
    E.SubElement(chassis,'site',name='imu',pos=fmt(center),size='.001')
    sensors=E.SubElement(root,'sensor')
    E.SubElement(sensors,'gyro',name='imu_gyro',site='imu')
    E.SubElement(sensors,'accelerometer',name='imu_accel',site='imu')
    for s in ['left','right']:E.SubElement(sensors,'jointpos',name=s+'_encoder',joint=s+'_axle')
    for a in root.find('actuator'):a.set('ctrlrange',fmt([-peak,peak]))
    xml=E.tostring(root,encoding='unicode')
    meta.update(schema='alfred-bb8-IMU-model/v3',mass_profile=profile['profile'],mass_hardware_calibrated=False,
                component_mass_profile_sha256=hashlib.sha256(json.dumps(profile,sort_keys=True).encode()).hexdigest(),
                groups_mass_COM_inertia=profile['groups'],total_mass_without_head_kg=profile['total_mass_without_head_kg'],
                torque_limit_Nm_per_wheel_ASSUMED=peak,motor_stall_axle_Nm_PROXY=.596,
                motor_no_load_axle_rpm_PROXY=330*35/19,
                imu_mount_m_ESTIMATED=center,control_period_s=.01,sensor_latency_s=.01,
                command_source='simulated gyro, accelerometer and encoder jointpos only; no ground-truth body pose/velocity feedback',
                cavity_and_tire_collision=collision,
                xml_sha256=hashlib.sha256(xml.encode()).hexdigest())
    meta.pop('mass_ASSUMED_kg',None);meta.pop('motor_no_load_speed_rpm_ASSUMED',None)
    if collision=='sdf':
        meta['cavity_wedges']=0;meta['max_inner_faceting_error_m']=0.;meta['max_tire_chord_error_m']=0.
        meta['tire_segments_per_ring']=0
    return xml,meta

def quat_pitch(q):
    w,x,y,z=q;return math.atan2(2*(w*y-x*z),1-2*(x*x+y*y))

def simulate(model,meta,fit,profile,zero,case='hold',duration=6.,gains=None,enabled=True,peak=.12,initial_offset=0.,noise=False):
    d=mujoco.MjData(model);dt=model.opt.timestep;steps=round(duration/dt);period=round(.01/dt)
    theta=zero+initial_offset;R=np.array([[math.cos(theta),0,math.sin(theta)],[0,1,0],[-math.sin(theta),0,math.cos(theta)]])
    d.qpos[7:10]=np.array([0,0,.1001])+R@fit
    d.qpos[10:14]=[math.cos(theta/2),0,math.sin(theta/2),0]
    mujoco.mj_forward(model,d)
    ctl=Controller(zero=zero,enabled=enabled,**(gains or {}));request=np.zeros(2);pending=None;frames=[]
    # Initial condition: held motionless before release, as for IMU startup.
    # Only this rest accelerometer sample is synthesized; ongoing feedback uses sensors.
    rest_accel=R.T@np.array([0.,0.,9.81])
    ctl.update(np.zeros(3),rest_accel,np.array([0.,0.]))
    names=[model.geom(i).name for i in range(model.ngeom)];shell=model.body('shell').id;core=model.body('chassis').id
    counts={'left':0,'right':0};maxpen=0.;max_tilt=0.;max_tau=0.;sum2pitch=0.;max_esterror=0.;max_speed=0.;origin=None
    max_full_tilt=0.;contact_counts={};contact_forces={}
    force=np.zeros(6);rng=np.random.default_rng(20261109);t0=time.monotonic()
    for k in range(steps+1):
        t=k*dt
        if k%period==0:
            measurement=(d.sensor('imu_gyro').data.copy(),d.sensor('imu_accel').data.copy(),
                         np.array([d.sensor(s+'_encoder').data[0] for s in ['left','right']]))
            if noise:
                measurement[0][:]+=np.array([.0005,.001,-.0005])+rng.normal(0,.0005,3)
                measurement[1][:]+=rng.normal(0,.01,3)
            received=pending if pending is not None else measurement;pending=measurement
            target=(.03 if case=='forward' else -.03 if case=='reverse' else 0.) if 1<=t<3 else 0.
            yaw=(.2 if case=='yaw-left' else -.2 if case=='yaw-right' else 0.) if 1<=t<3 else 0.
            request=ctl.update(*received,target_speed=target,target_yaw=yaw)
        omega=d.qvel[12:14]
        # Bounded, hypothetical wheel-output electrical curve; explicitly not the original motor's measured curve.
        d.ctrl[:]=np.clip(request-.596/(330*35/19*math.pi/30)*omega,-peak,peak) if enabled else 0.
        d.xfrc_applied[:]=0.
        if case=='push' and 2<=t<2.05:d.xfrc_applied[core,4]=.03
        mujoco.mj_forward(model,d)
        tilt=quat_pitch(d.qpos[10:14]);error=math.atan2(math.sin(tilt-zero),math.cos(tilt-zero))
        max_full_tilt=max(max_full_tilt,math.acos(float(np.clip(d.xmat[core].reshape(3,3)[2,2],-1,1))))
        max_tilt=max(max_tilt,abs(error));sum2pitch+=error**2
        if ctl.initialized:max_esterror=max(max_esterror,abs(math.atan2(math.sin(tilt-ctl.pitch),math.cos(tilt-ctl.pitch))))
        max_tau=max(max_tau,float(np.max(np.abs(d.actuator_force))));speed=float(np.linalg.norm(d.qvel[:2]));max_speed=max(max_speed,speed)
        sides=set();contacts=[]
        for j,c in enumerate(d.contact):
            a=names[c.geom1];b=names[c.geom2]
            if a.startswith('cavity') or b.startswith('cavity'):
                maxpen=max(maxpen,-float(c.dist))
                for s in ['left','right']:
                    if a.startswith(s+'_ring') or b.startswith(s+'_ring'):sides.add(s)
                if k%20==0:
                    mujoco.mj_contactForce(model,d,j,force);contacts.append({'pair':[a,b],'penetration_m':max(0.,-float(c.dist)),'force_N':force[:3].tolist()})
        for s in sides:counts[s]+=1
        if k%20==0:
            present=set()
            for c in contacts:
                for name in c['pair']:
                    if name.startswith('cad_'):
                        present.add(name);contact_forces[name]=max(contact_forces.get(name,0.),float(np.linalg.norm(c['force_N'])))
            for name in present:contact_counts[name]=contact_counts.get(name,0)+1
        if abs(t-1)<dt/2:origin=d.qpos[:3].copy()
        if k%round(.02/dt)==0:
            frames.append({'t':round(t,8),'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist(),'ctrl':d.ctrl.tolist(),
                           'imu_control':ctl.last.copy(),'chassis_pitch_rad':tilt,'chassis_pitch_error_rad':error,
                           'wheel_contact_sides':sorted(sides),'contacts':contacts})
        if k<steps:mujoco.mj_step(model,d)
        if not np.isfinite(d.qpos).all() or max(d.warning.number)>0 or abs(d.time-min(k+1,steps)*dt)>1e-7:raise RuntimeError('Invalid simulation/time reset')
    mat=d.xmat[core].reshape(3,3);yaw=math.atan2(mat[1,0],mat[0,0])
    summary={'case':case,'mass_profile':profile['profile'],'total_mass_without_head_kg':profile['total_mass_without_head_kg'],
             'max_pitch_error_deg':math.degrees(max_tilt),'rms_pitch_error_deg':math.degrees(math.sqrt(sum2pitch/(steps+1))),
             'max_IMU_pitch_error_deg':math.degrees(max_esterror),'final_pitch_error_deg':math.degrees(error),
             'final_shell_xy_m':d.qpos[:2].tolist(),'shell_xy_from_1s_m':(d.qpos[:2]-origin[:2]).tolist() if origin is not None else None,
             'final_speed_m_s':speed,'peak_speed_m_s':max_speed,'final_yaw_rad':yaw,'final_yaw_rate_rad_s':float(d.qvel[11]),
             'max_abs_motor_torque_Nm':max_tau,'max_contact_penetration_m':maxpen,
             'wheel_contact_fraction':{s:counts[s]/(steps+1) for s in counts},'warnings':d.warning.number.tolist(),
             'steps':steps,'frames':len(frames),'runtime_s':time.monotonic()-t0,'controller_enabled':enabled}
    summary['max_full_tilt_deg']=math.degrees(max_full_tilt)
    summary['CAD_shell_contact_fraction_sampled']={name:n/len(frames) for name,n in contact_counts.items()}
    summary['CAD_max_single_contact_force_N_sampled']=contact_forces
    summary['upper_caster_contact_fraction_sampled']={name:contact_counts.get(name,0)/len(frames) for name in ['cad_c35','cad_c36','cad_c37','cad_c38']}
    return {'schema':'alfred-bb8-IMU-replay/v3','engine_version':mujoco.__version__,'model':meta,
            'case':case,'mass_profile':profile['profile'],'duration_s':duration,'control_zero_rad':zero,
            'hardware_calibrated':False,'summary':summary,'frames':frames}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--source',type=Path,required=True);ap.add_argument('--tire',type=Path,required=True)
    ap.add_argument('--fit',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);ap.add_argument('--case',default='hold');ap.add_argument('--seconds',type=float,default=6.)
    ap.add_argument('--profile',default='nominal');ap.add_argument('--level',type=int,default=4);ap.add_argument('--peak',type=float,default=.12)
    ap.add_argument('--kp',type=float,default=.5);ap.add_argument('--kd',type=float,default=.05);ap.add_argument('--kv',type=float,default=1.2);ap.add_argument('--ki',type=float,default=.3)
    ap.add_argument('--off',action='store_true');ap.add_argument('--allow-estimated-mass',action='store_true');ap.add_argument('--collision',choices=['sdf','facets'],default='facets')
    a=ap.parse_args()
    if not a.allow_estimated_mass:ap.error('Explicit --allow-estimated-mass required; hardware masses unconfirmed')
    a.out.mkdir(parents=True,exist_ok=True);parts,_=load_parts(a.source,a.tire);fit=np.array(json.loads(a.fit.read_text())['fit']['rigid_translation_from_source_pose_mm'])*.001
    profile=build(a.source,a.tire,a.profile);nominal=build(a.source,a.tire,'nominal');zero,com=zero_angle(nominal,fit)
    xml,meta=make(profile,parts,fit,a.level,peak=a.peak,collision=a.collision)
    (a.out/'model-before-integration.xml').write_text(xml)
    m=mujoco.MjModel.from_xml_string(xml)
    r=simulate(m,meta,fit,profile,zero,a.case,a.seconds,dict(kp=a.kp,kd=a.kd,kv=a.kv,ki=a.ki),not a.off,a.peak)
    tag=a.case+'-'+a.profile+('-OFF' if a.off else '')
    (a.out/(tag+'.xml')).write_text(xml);(a.out/(tag+'.json')).write_text(json.dumps(r,separators=(',',':'))+'\n')
    print('zero_deg',math.degrees(zero),'COM',com.tolist());print(json.dumps(r['summary'],indent=2))

if __name__=='__main__':main()
