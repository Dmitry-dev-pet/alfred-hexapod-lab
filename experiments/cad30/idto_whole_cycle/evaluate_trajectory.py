"""Fail-closed, kinematic (not force) audit of optimized IDTO CAD30 trajectory.

Full CAD30 meshes are local owner assets only. Imported Drake trajectories are
kinematically inspected by MuJoCo mj_forward; no physical mj_step is claimed.
"""
import argparse, json, zipfile
from pathlib import Path
import numpy as np
import mujoco

LEGS=('ml','mr','bl','br','fl','fr')

def audit(qfile,vfile,tfile,model,reference_meta,math_zip):
    q=np.loadtxt(qfile,delimiter=',',ndmin=2)
    v=np.loadtxt(vfile,delimiter=',',ndmin=2)
    tau=np.loadtxt(tfile,delimiter=',',ndmin=2)
    assert q.shape[1]==27 and v.shape[1]==26 and tau.shape[1]==26, (q.shape,v.shape,tau.shape)
    assert q.shape[0]==v.shape[0] and tau.shape[0] in (q.shape[0],q.shape[0]-1)
    m=mujoco.MjModel.from_xml_path(model);d=mujoco.MjData(m)
    assert (m.nq,m.nv,m.nu)==(27,26,20)
    floor=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'floor')
    ids={leg:mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'tibia'+leg+'_collision_0') for leg in LEGS}
    gaps=[];con=[];tip_history=[]
    tip_vertices={}
    for a in q:
        d.qpos[:3]=a[4:7];d.qpos[3:7]=a[:4];d.qpos[7:]=a[7:]
        d.qvel[:]=0
        mujoco.mj_forward(m,d)
        gaps.append({leg:float(mujoco.mj_geomDistance(m,d,floor,geom,1.0,None)) for leg,geom in ids.items()})
        con.append(int(d.ncon))
        if not tip_vertices:
            for leg,gid in ids.items():
                mesh_id=int(m.geom_dataid[gid])
                start,count=int(m.mesh_vertadr[mesh_id]),int(m.mesh_vertnum[mesh_id])
                verts=np.asarray(m.mesh_vert[start:start+count])
                tibia_body=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'tibia'+leg)
                jidx=int(m.body_jntadr[tibia_body])
                knee=np.asarray(d.xanchor[jidx])
                mat=np.asarray(d.geom_xmat[gid]).reshape(3,3)
                world=verts @ mat.T + np.asarray(d.geom_xpos[gid])
                tip_vertices[leg]=np.asarray(verts[int(np.argmax(np.linalg.norm(world-knee,axis=1)))])
        toe_positions={}
        for leg,gid in ids.items():
            rot=np.asarray(d.geom_xmat[gid]).reshape(3,3)
            toe_positions[leg]=np.asarray(d.geom_xpos[gid])+rot@tip_vertices[leg]
        tip_history.append(toe_positions)
    meta=json.loads(Path(reference_meta).read_text())
    with zipfile.ZipFile(math_zip) as z: t=json.loads(z.read('math_first_four_crawl_trace.json'))
    stages=[t['nodes'][i]['phase'] for i in meta['source_trace_indices']]
    assert len(stages)==len(q)
    swing={}
    for leg in LEGS[:4]:
        selected=[k for k,ph in enumerate(stages) if ph=='swing_'+leg]
        swing[leg]={
            'sampled_nodes':len(selected),
            'max_nominal_phase_airborne_gap_mm':round(1e3*max([gaps[k][leg] for k in selected],default=float('-inf')),4),
            'all_other_working_feet_within_2mm_on_sampled_swing_nodes':all(all(abs(gaps[k][other])<0.002 for other in LEGS[:4] if other !=leg) for k in selected),
            'passes_clearance_2mm':bool(selected and max(gaps[k][leg] for k in selected)>=0.002),
        }
    qstart=q[0];qend=q[-1]; vend=v[-1];vstart=v[0]
    ychange=1e3*(qend[5]-qstart[5]);
    return {
        'n_nodes':len(q),
        'root_world_y_translation_mm':float(ychange),
        'root_y_target_mm':float(meta['nominal_root_delta_y_mm']),
        'terminal_leg_joint_closure_max_rad':float(np.max(np.abs(qend[7:25]-qstart[7:25]))),
        'terminal_velocity_closure_max_rad_per_s':float(np.max(np.abs(vend-vstart))),
        'max_abs_leg_joint_rad':float(np.max(np.abs(q[:,7:25]))),
        'max_abs_leg_tau_Nm':float(np.max(np.abs(tau[:,6:24]))),
        'max_abs_head_tau_Nm':float(np.max(np.abs(tau[:,24:26]))),
        'max_abs_floating_generalized_force':float(np.max(np.abs(tau[:,:6]))),
        'toe_end_gap_mm':{leg:round(1000*gaps[-1][leg],4) for leg in LEGS},
        'toe_landmark_world_y_delta_mm':{leg:round(1e3*(tip_history[-1][leg][1]-tip_history[0][leg][1]),3) for leg in LEGS},
        'toe_gap_max_mm':{leg:round(1000*max(x[leg] for x in gaps),4) for leg in LEGS},
        'toe_gap_min_mm':{leg:round(1000*min(x[leg] for x in gaps),4) for leg in LEGS},
        'root_prep_not_included':True,
        'swing_kinematic_screen':swing,
        'kinematic_four_swing_reference_retained':all(x['passes_clearance_2mm'] for x in swing.values()),
        'within_research_tau_cap':bool(np.max(np.abs(tau[:,6:24]))<=.7 and np.max(np.abs(tau[:,24:26]))<=.2),
        'periodic_joint_screen_pass':bool(np.max(np.abs(qend[7:25]-qstart[7:25]))<=.03),
        'periodic_velocity_screen_pass':bool(np.max(np.abs(vend-vstart))<=.1),
        'physical_gait_verified':False,
        'notes':['MuJoCo mj_forward only: NO MuJoCo dynamics, NO actual Fz, NO hard contact/drive constraints.','IDTO soft-contact force and theta-optimal generalized torques are not hardware PD commands.']
    }

if __name__=='__main__':
 p=argparse.ArgumentParser()
 for k in ['q','v','tau','model','reference_meta','math_zip','out']:p.add_argument('--'+k.replace('_','-'),required=True)
 a=p.parse_args()
 data=audit(a.q,a.v,a.tau,a.model,a.reference_meta,a.math_zip)
 Path(a.out).write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n')
 print(json.dumps({k:data[k] for k in ['root_world_y_translation_mm','max_abs_leg_tau_Nm','terminal_velocity_closure_max_rad_per_s','kinematic_four_swing_reference_retained','within_research_tau_cap','physical_gait_verified']},indent=2))
