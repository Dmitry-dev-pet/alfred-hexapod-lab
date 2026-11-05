"""Physical *diagnostic* of an IDTO CAD30 solution, no 6->4 preparation.

Loads only the mathematically virtual, four-foot initial configuration, then
never writes qpos/qvel. Uses actual MuJoCo 2ms forward dynamics with bounded
joint PD torques. A PASS here would *not* prove a reachable 6->4 transition or
periodic three-cycle walking.
"""
from __future__ import annotations
import argparse, gzip, json, math, zipfile
from pathlib import Path
import numpy as np
import mujoco

LEGS=('ml','mr','bl','br','fl','fr')


def check_physics(model_file, traj_q, reference_meta, math_zip, dt, prefix, torque_file=None):
    q=np.loadtxt(traj_q,delimiter=',',ndmin=2)
    assert q.shape[1]==27 and len(q)>=9
    predicted_tau=(np.loadtxt(torque_file,delimiter=',',ndmin=2) if torque_file is not None else None)
    if predicted_tau is not None:
        assert predicted_tau.shape in ((len(q),26),(len(q)-1,26))
    meta=json.loads(Path(reference_meta).read_text())
    with zipfile.ZipFile(math_zip) as z:
        src=json.loads(z.read('math_first_four_crawl_trace.json'))['nodes']
    phases=[src[i]['phase'] for i in meta['source_trace_indices']]
    assert len(phases)==len(q)
    m=mujoco.MjModel.from_xml_path(model_file)
    d=mujoco.MjData(m)
    assert (m.nq,m.nv,m.nu)==(27,26,20)
    assert abs(m.opt.timestep-.002)<1e-12
    d.qpos[:3]=q[0,4:7];d.qpos[3:7]=q[0,:4];d.qpos[7:]=q[0,7:]
    d.qvel[:]=0.
    mujoco.mj_forward(m,d)
    floor=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'floor')
    ids={leg:mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'tibia'+leg+'_collision_0') for leg in LEGS}
    id_to_leg={i:leg for leg,i in ids.items()}
    motor_q=[];motor_v=[]
    for i in range(m.nu):
        j=int(m.actuator_trnid[i,0]);motor_q.append(int(m.jnt_qposadr[j]));motor_v.append(int(m.jnt_dofadr[j]))
    assert motor_q==list(range(7,27))
    period=(len(q)-1)*dt
    n=round(period/m.opt.timestep)
    q0=d.qpos.copy();initialY=float(d.qpos[1])
    support_stats={leg:{'frames':0,'good_trio':0,'actual_airborne_s':0.,'max_sampled_gap_mm':float('-inf'),'first_bad_t':None} for leg in LEGS[:4]}
    min_all_loaded=6; min_active_trio=3
    max_real_joint=0;max_cmd_torque=0;max_raw_pd=0;sat_frames=0;body_lowest=float('inf'); body_highest=float('-inf')
    first_low_support=None; first_trio_loss=None; sampled=0
    last_hold_four=0;last_hold_total=0; bad_body_collision=0; footdrop=[]
    prefix=Path(prefix);prefix.parent.mkdir(parents=True,exist_ok=True)
    with gzip.open(str(prefix)+'_physical_trace.jsonl.gz','wt') as f:
        for t in range(n):
            now=t*m.opt.timestep
            s=min(now/dt,len(q)-1-1e-10)
            k=min(int(s),len(q)-2)
            alpha=s-k
            goal=(1-alpha)*q[k]+alpha*q[k+1]
            # Absolute angles: q[7:27] in both kinematic models.
            desired=goal[7:]
            current=np.asarray(d.qpos)[motor_q]
            speed=np.asarray(d.qvel)[motor_v]
            ff=(np.zeros(20) if predicted_tau is None else predicted_tau[k,6:26])
            raw=ff+7*(desired-current)-.32*speed
            limits=np.array([.7]*18+[.2]*2)
            applied=np.clip(raw,-limits,limits)
            d.ctrl[:]=applied
            max_raw_pd=max(max_raw_pd,float(np.max(np.abs(raw[:18]))))
            max_cmd_torque=max(max_cmd_torque,float(np.max(np.abs(applied[:18]))))
            if np.any(np.abs(raw[:18])>limits[:18]+1e-7):sat_frames+=1
            mujoco.mj_step(m,d)
            if not np.all(np.isfinite(d.qpos)):
                raise RuntimeError('NaN physics state at frame '+str(t))
            forces={leg:0. for leg in LEGS}
            for ci in range(d.ncon):
                ct=d.contact[ci]
                g1,g2=int(ct.geom1),int(ct.geom2)
                if floor not in (g1,g2):continue
                g=g2 if g1==floor else g1
                leg=id_to_leg.get(g)
                if leg is None:continue
                out=np.zeros(6)
                mujoco.mj_contactForce(m,d,ci,out)
                normal=np.asarray(ct.frame).reshape(3,3)[0]
                forces[leg]+=max(0.,float(out[0]*abs(normal[2])))
            active=[leg for leg in LEGS if forces[leg]>=.2]
            min_all_loaded=min(min_all_loaded,len(active))
            if len(active)<3 and first_low_support is None:first_low_support={'t_s':round(float(d.time),3),'active_legs':active,'all_Fz':forces.copy()}
            swing=(phases[k] if phases[k].startswith('swing_') else '')
            active_swing=swing[6:] if swing else None
            if active_swing in LEGS[:4]:
                stats=support_stats[active_swing];stats['frames']+=1
                trio=[other for other in LEGS[:4] if other!=active_swing]
                nloaded=sum(forces[i]>=.2 for i in trio)
                min_active_trio=min(min_active_trio,nloaded)
                if nloaded==3:stats['good_trio']+=1
                if nloaded<3 and stats['first_bad_t'] is None:
                    stats['first_bad_t']=round(float(d.time),3)
                    if first_trio_loss is None:first_trio_loss={'t_s':round(float(d.time),3),'swing':active_swing,'Fz':forces.copy()}
                # One-step force threshold; clearance inspected at 20ms sample only.
                if forces[active_swing]<.08:
                    stats['actual_airborne_s']+=m.opt.timestep
            if t%(round(.02/m.opt.timestep))==0:
                sampled+=1
                for leg in LEGS[:4]:
                    if active_swing==leg:
                        gap=mujoco.mj_geomDistance(m,d,floor,ids[leg],.1,None)
                        support_stats[leg]['max_sampled_gap_mm']=max(support_stats[leg]['max_sampled_gap_mm'],1000*float(gap))
            if d.time>=period-.4:
                last_hold_total+=1
                if all(forces[x]>=.2 for x in LEGS[:4]):last_hold_four+=1
            max_real_joint=max(max_real_joint,float(np.max(np.abs(d.qpos[7:25]))))
            body_lowest=min(body_lowest,float(d.qpos[2])); body_highest=max(body_highest,float(d.qpos[2]))
            if t%10==0:
                record={'time_s':round(float(d.time),5),'root_xyz':list(map(float,d.qpos[:3])),
                    'qpos':list(map(float,d.qpos)), 'qvel':list(map(float,d.qvel)),
                    'Fz_N':forces,'phase':phases[k],'joint_torque':list(map(float,applied))}
                f.write(json.dumps(record,separators=(',',':'))+'\n')
    phase_report={leg:{**st,'min_all_expected_trio_loaded':int(min_active_trio)} for leg,st in support_stats.items()}
    for st in phase_report.values():st['actual_airborne_s']=round(st['actual_airborne_s'],3)
    endY=float(d.qpos[1]);
    result={
        'schema':'cad30-idto-g1-independent-physical-diagnostic-v1',
        'starting_state':'virtual post-preparation four-foot geometric qpos from archived PR43 trace, NOT physically reachable 6->4 proof',
        'motion':'continuous free-base MuJoCo mj_step (500Hz) from one initial qpos; PD applied to 18 leg joint motors; real contacts',
        'period_s':period, 'physics_steps':n,'real_root_y_delta_mm':1000*(endY-initialY),
        'starting_root_y_m':initialY, 'ending_root_y_m':endY,
        'min_loaded_legs_any_step':min_all_loaded,'min_loaded_three_required_leg_trio_during_swing':min_active_trio,
        'first_total_support_loss_below_three':first_low_support, 'first_required_trio_failure':first_trio_loss,
        'per_swing_leg':phase_report,
        'max_abs_leg_joint_rad':max_real_joint,'max_raw_pd_torque_Nm':max_raw_pd,
        'max_applied_leg_torque_Nm':max_cmd_torque,'pd_saturated_steps':sat_frames,
        'body_root_z_range_m':[body_lowest,body_highest],
        'last_0p4s_four_support_fraction':last_hold_four/max(1,last_hold_total),
        'terminal_leg_joint_max_diff_from_initial_rad':float(np.max(np.abs(d.qpos[7:25]-q0[7:25]))),
        'kinematic_idto_source':'Drake optimized q[t] converted into PD targets, no root rewriting',
        'feedforward_idto_generalized_actuated_torques_enabled':predicted_tau is not None,
        'full_gait_accepted':False, 'reason':'No physically reached 6->4 start; requires 3 continuous cycles and net positive full-chain travel even if diagnostic local period passes'
    }
    Path(str(prefix)+'_physical_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser()
    for x in ['model','q','reference-meta','math-zip','prefix']:p.add_argument('--'+x,required=True)
    p.add_argument('--dt',type=float,default=.9)
    p.add_argument('--feedforward-tau',default=None)
    a=p.parse_args()
    r=check_physics(a.model,a.q,a.reference_meta,a.math_zip,a.dt,a.prefix,torque_file=a.feedforward_tau)
    print(json.dumps({k:r[k] for k in ['real_root_y_delta_mm','min_loaded_legs_any_step','first_required_trio_failure','max_abs_leg_joint_rad','pd_saturated_steps','last_0p4s_four_support_fraction','full_gait_accepted']},ensure_ascii=False,indent=2))
