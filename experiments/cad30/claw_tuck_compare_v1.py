"""Precomputed front-claw folding positions, physical MuJoCo check.

Starts from six grounded legs, shifts body, unloads both front tibias,
raises them and only then tests A/B/C front-leg folding poses.
No online planning; no root teleport; no prerecorded fake gait.
"""
from __future__ import annotations
import sys,json,math,hashlib,argparse
from pathlib import Path
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'experiments/cad30')]
from optimized_claw_gait import init_model,actuator_mapping,physical_contact_forces,cartesian_shift_paths,sample_ik_path,FRONT,ALL
from hexlab.stand_geometry_v04 import tip_world,geom_mesh_world_vertices
from hexlab.step_one_leg_v04 import floor_contacts,solve_waypoint,smoothstep
from support_wrench_qp import support_margin_xy

# `target_xy_factor` is fraction of initial front toe horizontal vector from hip.
# Both tucked variants additionally raise toe to stay within the conservative
# inverse-kinematics joint bounds (+/-1.7 rad) used by existing project.
POSES={
    'extended': {'target_xy_factor':1.0,'target_toe_z_m':None},
    'semi_tucked': {'target_xy_factor':0.85,'target_toe_z_m':0.18},
    'compact': {'target_xy_factor':0.75,'target_toe_z_m':0.23},
}
PD_P,PD_D=7.,0.32
SHIFT=0.038
UNLOAD_Z=0.010
HIGH_Z=0.100
MU=1.1


def simulate(pose='extended', *, record=False, mu=MU, fold_seconds=1.5,hold_seconds=1.0):
    if pose not in POSES:raise ValueError('unknown preset')
    m,d,legs=init_model(mu)
    qa,da,ids=actuator_mapping(m,d,legs)
    limits=m.actuator_ctrlrange.copy();dt=float(m.opt.timestep)
    base0=d.qpos[qa].copy()
    start0=d.qpos.copy()
    paths, path_err=cartesian_shift_paths(m,d,legs,FRONT,SHIFT)
    total=0;max_torque=0.;peak_front=0.;saturated=0;loss_stance=0
    phases=[];qframes=[];support_log=[];step_idx=0
    com0=(d.xipos*m.body_mass[:,None]).sum(axis=0)/m.body_mass.sum()
    baseid=m.body('base_link_2').id
    ref_up=d.xmat[baseid].reshape(3,3)[:,2].copy()
    def recorder(phase):
        nonlocal max_torque,peak_front
        contacts=floor_contacts(m,d,mujoco)
        forces,points=physical_contact_forces(m,d)
        com=(d.xipos*m.body_mass[:,None]).sum(axis=0)/m.body_mass.sum()
        four=[points[c][:2] for c in ALL if c not in FRONT and points[c] is not None]
        margin=float(support_margin_xy(np.asarray(four),com[:2])) if len(four)>=3 else -0.5
        up=float(np.arccos(np.clip(np.dot(d.xmat[baseid].reshape(3,3)[:,2],ref_up),-1,1))*180/np.pi)
        meshz={c:float(geom_mesh_world_vertices(m,d,next(ll for ll in legs if ll['suffix']==c)['geom'])[:,2].min()) for c in FRONT}
        return {'time_s':round(step_idx*dt,3),'phase':phase,'four_contacts':sum(c in contacts for c in ALL if c not in FRONT),
                'front_contacts':sum(c in contacts for c in FRONT),'front_force_N':float(sum(forces[c][2] for c in FRONT)),
                'COM_xyz_m':np.round(com,7).tolist(), 'COM_margin_four_mm':round(margin*1e3,3),
                'front_min_mesh_z_mm':round(min(meshz.values())*1e3,3),'tilt_deg':round(up,3),
                'torque_max_nm':round(float(np.max(np.abs(d.ctrl))),4),'contacts':sorted(contacts),
                'root_z_m':round(float(d.xpos[baseid,2]),6),
                'joint_limit_reserve_front_deg':round(float((1.7-max(abs(float(d.qpos[qad])) for ll in legs if ll['suffix'] in FRONT for qad in ll['qadrs']))*180/np.pi),3)}
    def step(cmd,phase):
        nonlocal step_idx,max_torque,peak_front,saturated,loss_stance,total
        raw=PD_P*(cmd-d.qpos[qa])-PD_D*d.qvel[da]
        if np.any(raw>limits[:,1]+1e-7) or np.any(raw<limits[:,0]-1e-7):saturated+=1
        max_torque=max(max_torque,float(np.max(np.abs(raw))))
        peak_front=max(peak_front,float(np.max(np.abs(raw[np.r_[ids['fl'],ids['fr']]]))))
        d.ctrl[:]=np.clip(raw,limits[:,0],limits[:,1])
        mujoco.mj_step(m,d)
        step_idx+=1
        if step_idx % 10==0:
            record_row=recorder(phase)
            support_log.append(record_row)
        if step_idx % 50==0 and record:
            qframes.append({'t':round(step_idx*dt,3),'phase':phase,
                            'qpos':np.round(d.qpos,8).tolist(),
                            'contacts':sorted(floor_contacts(m,d,mujoco))})
        if step_idx>round(4.1/dt):
            loss_stance+=int(sum(c in floor_contacts(m,d,mujoco) for c in ALL if c not in FRONT)<4)
    def advance(duration, phase, fn):
        n=int(round(duration/dt))
        for k in range(n):step(fn(smoothstep((k+1)/n)),phase)
    advance(2.0,'six_stance_transfer',lambda a: _shift(base0,ids,paths,a))
    cmd_shift=_shift(base0,ids,paths,1.)
    advance(.6,'six_stance_settle',lambda a: cmd_shift)
    prep=d.qpos.copy()
    unload_cmd=cmd_shift.copy();front_targets={}
    for leg in legs:
        if leg['suffix'] not in FRONT:continue
        ref=mujoco.MjData(m);ref.qpos[:]=prep;ref.qvel[:]=0;mujoco.mj_forward(m,ref)
        goal=tip_world(m,ref,leg)+np.array([0.,0.,UNLOAD_Z])
        q,err=solve_waypoint(m,ref,mujoco,leg,goal,prep[leg['qadrs']])
        if err>.005:raise RuntimeError(f'unload IK failure {leg["suffix"]}: {err}')
        unload_cmd[ids[leg['suffix']]]=q
    advance(1.5,'unload_front',lambda a: cmd_shift*(1-a)+unload_cmd*a)
    advance(.7,'hold_four',lambda a: unload_cmd)
    high_start=d.qpos.copy()
    high_cmd=unload_cmd.copy();high_err={}
    for leg in legs:
        if leg['suffix'] not in FRONT:continue
        ref=mujoco.MjData(m);ref.qpos[:]=high_start;ref.qvel[:]=0;mujoco.mj_forward(m,ref)
        goal=tip_world(m,ref,leg)+np.array([0.,0.,HIGH_Z])
        q,err=solve_waypoint(m,ref,mujoco,leg,goal,high_start[leg['qadrs']])
        high_err[leg['suffix']]=err
        # Existing high-claw baseline is a commanded z displacement; the achieved
        # pose is measured in the following MuJoCo steps. Keep IK residual visible.
        high_cmd[ids[leg['suffix']]]=q
    advance(1.5,'raise_claws',lambda a: unload_cmd*(1-a)+high_cmd*a)
    advance(.5,'hold_claws_high',lambda a: high_cmd)
    if record:qframes.append({'t':round(step_idx*dt,3),'phase':'before_tuck', 'qpos':np.round(d.qpos,8).tolist(),
                              'contacts':sorted(floor_contacts(m,d,mujoco))})
    baseline=recorder('before_tuck')
    # Compute symmetric front positions at dynamically achieved high-claws pose.
    config=POSES[pose];fold_cmd=high_cmd.copy();IK={};target={};hip_tip={}
    ref=mujoco.MjData(m);ref.qpos[:]=d.qpos;ref.qvel[:]=0;mujoco.mj_forward(m,ref)
    for leg in legs:
        c=leg['suffix']
        if c not in FRONT:continue
        hip=np.asarray(ref.xanchor[leg['jids'][0]]).copy()
        tip=tip_world(m,ref,leg).copy()
        hip_tip[c]={'initial_hip':np.round(hip,6).tolist(),'initial_tip':np.round(tip,6).tolist()}
        z=tip[2] if config['target_toe_z_m'] is None else float(config['target_toe_z_m'])
        goal=np.r_[hip[:2]+config['target_xy_factor']*(tip[:2]-hip[:2]),z]
        init=ref.qpos[leg['qadrs']].copy()
        if pose=='extended':q=init;err=0.
        else:q,err=solve_waypoint(m,ref,mujoco,leg,goal,init)
        if err>.005:raise ValueError(f'Pose {pose}/{c} cannot reach goal: {err*1000:.1f}mm')
        fold_cmd[ids[c]]=q
        IK[c]={'error_mm':round(err*1e3,3),'target_xyz':np.round(goal,5).tolist(),
               'q_target':np.round(q,5).tolist()}
        ref.qpos[leg['qadrs']]=init;mujoco.mj_forward(m,ref)
    advance(fold_seconds,'fold_'+pose,lambda a: high_cmd*(1-a)+fold_cmd*a)
    advance(hold_seconds,'hold_'+pose,lambda a: fold_cmd)
    last=recorder('last')
    # Compare full interval in the fold, not just the end.
    foldrows=[row for row in support_log if row['time_s']>6.8-1e-6]
    tracked=[row for row in support_log if row['time_s']>=4.1]
    result={'pose':pose,'model_friction_mu':mu,'source_model':'CAD30 nominal ~1.805 kg, 30 STL local, torque cap +/-0.7Nm',
            'pd_gains':[PD_P,PD_D],'start':'initial six grounded legs; 1s physical stabilization',
            'phase_s':[2.0,.6,1.5,.7,1.5,.5,fold_seconds,hold_seconds],
            'duration_s':round(step_idx*dt,3),'initial_COM_xyz_m':np.round(com0,6).tolist(),
            'before_tuck':baseline,'after_tuck':last,'ik_targets':IK,'hip_tip':hip_tip,
            'min_support_four_during_tuck':min((r['four_contacts'] for r in foldrows),default=None),
            'min_support_four_after_unload':min((r['four_contacts'] for r in tracked),default=None),
            'min_front_mesh_height_mm_during_tuck':min((r['front_min_mesh_z_mm'] for r in foldrows),default=None),
            'min_support_margin_during_tuck_mm':min((r['COM_margin_four_mm'] for r in foldrows),default=None),
            'max_tilt_during_tuck_deg':max((r['tilt_deg'] for r in foldrows),default=None),
            'max_control_torque_unclipped_nm':round(max_torque,5),
            'max_front_control_torque_unclipped_nm':round(peak_front,5),
            'saturated_steps_fraction':round(saturated/step_idx,8),
            'stance_loss_after_front_unload_fraction':round(loss_stance/max(1,step_idx-round(4.1/dt)),8),
            'support_samples_50hz':support_log if record else None,'qpos_samples_10hz':qframes if record else None}
    result['strict_pass']=bool(result['min_support_four_after_unload']>=4
              and result['min_support_margin_during_tuck_mm']>0.0
              and result['min_front_mesh_height_mm_during_tuck']>0.0
              and result['saturated_steps_fraction']==0.0
              and result['max_tilt_during_tuck_deg']<8.0)
    return result

def _shift(base0,idx,paths,a):
    c=base0.copy()
    for code,path in paths.items():c[idx[code]]=sample_ik_path(path,a)
    return c

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--pose',choices=[*POSES,'all'],default='all')
    parser.add_argument('--record',action='store_true')
    parser.add_argument('--out',type=Path,default=ROOT/'results/claw_tuck_compare_2026-11-04.json')
    opts=parser.parse_args()
    names=[opts.pose] if opts.pose!='all' else list(POSES)
    results={}
    for name in names:
        print('Evaluating',name,flush=True)
        r=simulate(name,record=opts.record)
        results[name]=r
        print(json.dumps({k:v for k,v in r.items() if k not in ('support_samples_50hz','qpos_samples_10hz')},ensure_ascii=False),flush=True)
    opts.out.parent.mkdir(parents=True,exist_ok=True)
    opts.out.write_text(json.dumps(results,ensure_ascii=False,separators=(',',':'))+'\n',encoding='utf-8')

if __name__=='__main__':main()
