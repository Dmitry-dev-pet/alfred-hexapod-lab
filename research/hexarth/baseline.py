#!/usr/bin/env python3
"""Bounded HexArth CAD-native diagnostic, not a calibrated hardware controller.

All replay poses are sampled from mj_step states. No imposed chassis motion.
OEM STEP/STL and large traces stay in a private artifact directory.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import trimesh

DT=.001
SAMPLE=.02
TORQUE=.980665
STALL=2.941995
SPEED=2*math.pi/0.222/6  # 60 degrees in 0.222 s.
RANGE=math.radians(20)
KP=24.0
KD=.07


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def nums(x):
    a=np.asarray(x,dtype=float)
    assert np.isfinite(a).all()
    return np.round(a,8).tolist()


def fmt(x):
    return ' '.join(f'{float(v):.10g}' for v in np.asarray(x).flat)


def rotation(axis,angle):
    return Rotation.from_rotvec(np.asarray(axis)*angle).as_matrix()


def fk(leg,q):
    p0,p1,p2=np.array(leg['pivots_mm'])*.001
    foot=np.array(leg['foot_center_mm'])*.001
    z,a,b=np.array(leg['axes'])
    R0=rotation(z,q[0]);R1=rotation(a,q[1]);R2=rotation(b,q[2])
    return p0+R0@(p1-p0+R1@(p2-p1+R2@(foot-p2)))


def build(assets,output,step_height):
    manifest=json.loads((assets/'cad-manifest.json').read_text())
    for filename,expected in manifest['mesh_sha256'].items():
        assert digest(assets/'meshes'/filename)==expected,filename
    mj=ET.Element('mujoco',model='HexArth CAD-native diagnostic v1')
    ET.SubElement(mj,'compiler',angle='radian',meshdir=os.path.relpath(assets/'meshes',output),inertiafromgeom='false')
    ET.SubElement(mj,'option',timestep=str(DT),integrator='implicitfast',solver='Newton',iterations='80',gravity='0 0 -9.81')
    default=ET.SubElement(mj,'default')
    ET.SubElement(default,'joint',damping='.015',armature='.00002')
    ET.SubElement(default,'geom',friction='.8 .01 .001',solref='.008 1',solimp='.9 .95 .001',margin='.0001')
    asset=ET.SubElement(mj,'asset');world=ET.SubElement(mj,'worldbody')
    ET.SubElement(world,'geom',name='floor',type='plane',size='3 3 .1',rgba='.3 .33 .36 1')
    if step_height:
        # Robot starts centered. Nearest CAD shoe is at Y~0.17m.
        ET.SubElement(world,'geom',name='step',type='box',pos=fmt([0,1.11,step_height/2]),
                      size=fmt([.6,.89,step_height/2]),rgba='.45 .45 .5 1')
    base=ET.SubElement(world,'body',name='base',pos='0 0 .0993')
    ET.SubElement(base,'freejoint',name='root')
    bodies={'base':base};pivots={'base':np.zeros(3)}
    for leg in manifest['legs']:
        prior='base'
        for j,part in enumerate(('coxa','femur','tibia')):
            group=f"leg{leg['index']}_{part}"
            pos=np.array(leg['pivots_mm'][j])*.001
            body=ET.SubElement(bodies[prior],'body',name=group,pos=fmt(pos-pivots[prior]))
            ET.SubElement(body,'joint',name=group+'_joint',axis=fmt(leg['axes'][j]),
                          type='hinge',limited='true',range=fmt([-RANGE,RANGE]))
            bodies[group]=body;pivots[group]=pos;prior=group
        ET.SubElement(bodies[prior],'site',name=f"foot{leg['index']}",
                      pos=fmt(np.array(leg['foot_center_mm'])*.001-pivots[prior]),size='.001')
    inertia_rows=[]
    for group,filename in manifest['visual_meshes'].items():
        mesh=trimesh.load(assets/'meshes'/filename,force='mesh',process=False)
        bounds=np.asarray(mesh.bounds)*.001
        center=bounds.mean(axis=0)
        extent=np.maximum(bounds[1]-bounds[0],.005)
        mass=.9 if group=='base' else (.09 if group.endswith('coxa') else .08 if group.endswith('femur') else .03)
        # Explicit diagnostic estimate. Not the STEP material density field.
        diagonal=mass/12*np.array([extent[1]**2+extent[2]**2,extent[0]**2+extent[2]**2,extent[0]**2+extent[1]**2])
        ET.SubElement(bodies[group],'inertial',mass=str(mass),pos=fmt(center-pivots[group]),diaginertia=fmt(diagonal))
        # Raw visual STL can exceed MuJoCo's 200,000-face STL parser limit.
        # Physics compiles only exterior collision parts; original unsimplified
        # visual meshes are attached to the same saved body poses by the player.
        inertia_rows.append({'body':group,'mass_kg':mass,'com_world_cad_m':nums(center),
                             'diagonal_inertia_kg_m2':nums(diagonal)})
    for n,part in enumerate(manifest['parts']):
        if not part['collision']:
            continue
        meshname=f'collision_{n}'
        ET.SubElement(asset,'mesh',name=meshname,file=part['collision_file'],scale='.001 .001 .001')
        ET.SubElement(bodies[part['body']],'geom',name=meshname,type='mesh',mesh=meshname,
                      pos=fmt(-pivots[part['body']]),group='3',rgba='.5 .4 .2 .2')
    actuators=ET.SubElement(mj,'actuator')
    for group in list(bodies)[1:]:
        ET.SubElement(actuators,'motor',name=group+'_motor',joint=group+'_joint',gear='1',
                      ctrllimited='true',ctrlrange=fmt([-TORQUE,TORQUE]))
    path=output/(f'model-step-{round(step_height*1000)}mm.xml' if step_height else 'model-flat.xml')
    ET.indent(mj);ET.ElementTree(mj).write(path,encoding='unicode')
    model=mujoco.MjModel.from_xml_path(str(path))
    assert model.nq==25 and model.nv==24 and model.nu==18
    assert abs(float(model.body_mass.sum())-2.1)<1e-8
    data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    fk_errors=[]
    for leg in manifest['legs']:
        sid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,f"foot{leg['index']}")
        reference=fk(leg,np.zeros(3))+data.qpos[:3]
        fk_errors.append(float(np.linalg.norm(data.site_xpos[sid]-reference)))
    assert max(fk_errors)<1e-8
    # Independently compare FK under nonzero joint rotations to compiled MJCF.
    data.qpos[7:]=np.tile([.07,-.08,.09],6);mujoco.mj_forward(model,data)
    for leg in manifest['legs']:
        sid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,f"foot{leg['index']}")
        reference=fk(leg,[.07,-.08,.09])+data.qpos[:3]
        fk_errors.append(float(np.linalg.norm(data.site_xpos[sid]-reference)))
    assert max(fk_errors)<1e-8
    return manifest,model,path,inertia_rows,max(fk_errors)


def gait_references(manifest,duration,walk):
    times=np.arange(0,duration+SAMPLE/2,SAMPLE)
    targets=[];max_residual=0.;range_limited=0
    old=np.zeros((6,3))
    for t in times:
        q=np.zeros((6,3))
        if walk and t>=2:
            u=t-2; phase=(u/1.6+.25)%1
            amplitude=min(1.,u/1.)
            for i,leg in enumerate(manifest['legs']):
                phase_i=(phase+(0 if i in (0,2,4) else .5))%1
                swing=phase_i<.5
                local=phase_i*2 if swing else (phase_i-.5)*2
                smooth=local*local*(3-2*local)
                dy=.020*((smooth-.5) if swing else (.5-smooth))
                dz=.012*4*local*(1-local) if swing else 0
                target=np.array(leg['foot_center_mm'])*.001+amplitude*np.array([0,dy,dz])
                # A nearly-zero warm start otherwise initializes an almost-zero
                # trust region and can terminate before a meaningful IK step.
                initial=old[i] if np.linalg.norm(old[i])>1e-5 else np.zeros(3)
                sol=least_squares(lambda a:fk(leg,a)-target,initial,bounds=(-RANGE,RANGE),
                                  ftol=1e-12,xtol=1e-12,gtol=1e-12,max_nfev=50)
                q[i]=sol.x
                max_residual=max(max_residual,float(np.linalg.norm(sol.fun)))
                range_limited+=int(np.max(np.abs(sol.x))>RANGE-.0001)
        targets.append(q.ravel());old=q
    assert max_residual<5e-6,f'IK target not reached: {max_residual}'
    return times,np.array(targets),max_residual,range_limited


def run_case(manifest,model,path,output,name,walk,duration):
    times,refs,ik_error,ik_clips=gait_references(manifest,duration,walk)
    data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    start=data.qpos[:3].copy();frames=[];peak_tau=0.;peak_speed=0.;sat_steps=0
    body_ids=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,g) for g in manifest['visual_meshes']]
    foot_ids=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,f'leg{i}_tibia') for i in range(6)]
    step_gid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'step')
    step_top=float(model.geom_pos[step_gid,2]+model.geom_size[step_gid,2]) if step_gid>=0 else 0.
    max_self_pen=0.;max_floor_pen=0.;t0=time.monotonic()
    for step in range(round(duration/DT)):
        t=data.time
        j=min(int(t/SAMPLE),len(refs)-2);f=(t-times[j])/SAMPLE
        target=(1-f)*refs[j]+f*refs[j+1]
        desired=KP*(target-data.qpos[7:])-KD*data.qvel[6:]
        envelope=np.minimum(TORQUE,STALL*np.maximum(0,1-np.sign(desired)*data.qvel[6:]/SPEED))
        torque=np.clip(desired,-envelope,envelope)
        data.ctrl[:]=torque
        peak_tau=max(peak_tau,float(np.max(np.abs(torque))))
        peak_speed=max(peak_speed,float(np.max(np.abs(data.qvel[6:]))))
        sat_steps+=int(np.any(np.abs(desired)>envelope+1e-9))
        mujoco.mj_step(model,data)
        assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
        if (step+1)%round(SAMPLE/DT):
            continue
        mujoco.mj_forward(model,data)
        contacts=[];foot_load=np.zeros(6);step_load=np.zeros(6);self_contacts=0;chassis_contact=False
        for k in range(data.ncon):
            c=data.contact[k];g1,g2=int(c.geom1),int(c.geom2)
            b1,b2=int(model.geom_bodyid[g1]),int(model.geom_bodyid[g2])
            force=np.zeros(6);mujoco.mj_contactForce(model,data,k,force)
            external=(b1==0 or b2==0)
            if external:
                max_floor_pen=max(max_floor_pen,max(0.,-float(c.dist)))
                body=b2 if b1==0 else b1
                if body in foot_ids:
                    foot_load[foot_ids.index(body)]+=max(0.,force[0])
                    ground_geom=g1 if b1==0 else g2
                    if ground_geom==step_gid and abs(c.frame[2])>.8 and c.pos[2]>=step_top-.001:
                        step_load[foot_ids.index(body)]+=max(0.,force[0])
                else:chassis_contact=True
            else:
                self_contacts+=1
                max_self_pen=max(max_self_pen,max(0.,-float(c.dist)))
            contacts.append({'g':[g1,g2],'distance_m':round(float(c.dist),8),
                             'normal_world':nums(c.frame[:3]),'normal_force_n':round(float(force[0]),6)})
        poses=[]
        for bid in body_ids:
            # Body poses, not normalized MuJoCo mesh poses: raw CAD STL remains
            # in original global coordinates; local geometry offset is -pivot.
            poses.extend(nums(data.xpos[bid]));poses.extend(nums(data.xquat[bid]))
        up=data.xmat[1].reshape(3,3)[2,2]
        frames.append({'t':round(data.time,5),'qpos':nums(data.qpos),'qvel':nums(data.qvel),
                       'ctrl':nums(torque),'target':nums(target),'contacts':contacts,
                       'foot_normal_load_n':nums(foot_load),'step_normal_load_n':nums(step_load),'body_poses':poses,
                       'tilt_deg':round(math.degrees(math.acos(np.clip(up,-1,1))),5),
                       'self_contacts':self_contacts,'nonfoot_ground_contact':chassis_contact})
    late=[r for r in frames if r['t']>=duration-1]
    result={'case':name,'duration_s':duration,'frame_count':len(frames),'frames':frames,
            'metrics':{'displacement_m':nums(data.qpos[:3]-start),
                       'forward_after_settle_m':round(float(data.qpos[1]-frames[99]['qpos'][1]),6),
                       'peak_torque_nm':peak_tau,'peak_joint_speed_rad_s':peak_speed,
                       'actuator_saturated_step_fraction':sat_steps/round(duration/DT),
                       'peak_self_penetration_m':max_self_pen,'peak_external_penetration_m':max_floor_pen,
                       'max_tilt_last_s_deg':max(r['tilt_deg'] for r in late),
                       'max_tilt_deg':max(r['tilt_deg'] for r in frames),
                       'min_base_height_last_s_m':min(r['qpos'][2] for r in late),
                       'nonfoot_ground_contact_frames':sum(r['nonfoot_ground_contact'] for r in frames),
                       'max_ik_residual_m':ik_error,'ik_near_bound_samples':ik_clips,
                       'feet_on_step_last_second':[any(r['step_normal_load_n'][i]>.1 for r in late) for i in range(6)],
                       'mujoco_warnings':[{'index':i,'number':int(w.number)} for i,w in enumerate(data.warning) if w.number],
                       'wall_s':round(time.monotonic()-t0,3)}}
    # Standing/walking criteria assess only this diagnostic configuration.
    m=result['metrics']
    result['diagnostic_stable_end']=(m['max_tilt_last_s_deg']<10 and m['min_base_height_last_s_m']>.065
                                     and not any(r['nonfoot_ground_contact'] for r in late))
    result['walk_forward_30mm']=bool(walk and result['diagnostic_stable_end'] and m['forward_after_settle_m']>.03)
    result['whole_robot_on_step']=bool('step' in name and result['diagnostic_stable_end']
                                      and all(m['feet_on_step_last_second']) and data.qpos[1]>.40)
    trace={'schema':'hexarth-cad-native-mujoco-trace/v1','mujoco_version':mujoco.__version__,
           'model_sha256':digest(path),'source_step_sha256':manifest['step_sha256'],**result}
    trace_path=output/(name+'-trace.json');trace_path.write_text(json.dumps(trace,separators=(',',':'))+'\n')
    # Same flat [time, root, geometry transforms] layout used by existing replay
    # exports. Bodies use a raw CAD offset provided by the separate manifest.
    replay={'format':'hexarth-oem-cad-mujoco-replay-v1','case':name,
            'source_trace_sha256':digest(trace_path),'source_model_sha256':digest(path),
            'duration_s':duration,'layout':{'root_pose_start':1,'visible_mesh_pose_start':8,
                                         'visible_mesh_stride':7,'visible_mesh_count':19,'frame_stride':141},
            'frames':[[r['t']]+r['qpos'][:7]+r['body_poses'] for r in frames]}
    assert all(len(r)==141 for r in replay['frames'])
    (output/(name+'-replay.json')).write_text(json.dumps(replay,separators=(',',':'))+'\n')
    return {k:v for k,v in result.items() if k!='frames'}|{'trace_sha256':digest(trace_path)}


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--assets',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True);ap.add_argument('--duration',type=float,default=10.)
    a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    summaries=[];meta=None
    for name,walk,height,duration in (('stand',False,0.,10.),('walk-flat',True,0.,10.),
                                     ('walk-step-10mm',True,.01,a.duration),('walk-step-20mm',True,.02,a.duration)):
        manifest,model,path,inertias,fk_error=build(a.assets,a.output,height)
        meta={'inertias':inertias,'max_fk_crosscheck_error_m':fk_error,'model_mass_kg':float(model.body_mass.sum()),
              'collision_geometry':'MuJoCo convex hull of each CAD exterior part; visual geometry is original tessellation'}
        summary=run_case(manifest,model,path,a.output,name,walk,duration)
        summaries.append(summary);print(json.dumps(summary),flush=True)
    asset_manifest={'schema':'hexarth-oem-cad-replay-assets/v1','step_sha256':manifest['step_sha256'],
                    'unit_scale':.001,'body_order':list(manifest['visual_meshes']),
                    'meshes':[]}
    for group,filename in manifest['visual_meshes'].items():
        pivot=np.zeros(3) if group=='base' else np.array(manifest['legs'][int(group[3])]['pivots_mm'][('coxa','femur','tibia').index(group.split('_')[1])])*.001
        asset_manifest['meshes'].append({'body':group,'file':'meshes/'+filename,
                                         'sha256':manifest['mesh_sha256'][filename],
                                         'local_translation_m':nums(-pivot)})
    (a.output/'asset-manifest.json').write_text(json.dumps(asset_manifest,indent=2)+'\n')
    receipt={'schema':'hexarth-first-trial-receipt/v1','python':platform.python_version(),
             'mujoco':mujoco.__version__,'timestep_s':DT,'sample_s':SAMPLE,'seed':None,
             'actuator':{'rated_torque_nm':TORQUE,'stall_torque_nm':STALL,'no_load_speed_rad_s':SPEED,
                         'kp_nm_per_rad':KP,'kd_nm_s_per_rad':KD,'envelope':'min(rated, stall*(1-signed_speed/no_load_speed))'},
             'joint_diagnostic_range_rad':[-RANGE,RANGE],
             'gait':{'type':'CAD-native diagnostic tripod; not an OEM firmware execution','cycle_s':1.6,
                     'stride_m':.020,'lift_m':.012,'forward_axis':'OEM CAD +Y','start_s':2.,'ramp_s':1.},
             'source_step_sha256':manifest['step_sha256'],'model':meta,'cases':summaries,
             'limitations':['Uncalibrated per-link masses, COM and box-estimated inertias; total 2.1 kg is a scenario.',
                            'Local +/-20 degree diagnostic ranges are not a validated firmware-to-hardware calibration.',
                            'Shoe-center geometry and OEM FK endpoint differ; no full firmware gait equivalence claimed.',
                            'No self-righting or autonomous obstacle perception in this baseline.',
                            'MuJoCo mesh collision uses convex hulls; compliance, backlash and thermal limits are unmeasured.']}
    (a.output/'receipt.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')


if __name__=='__main__':main()
