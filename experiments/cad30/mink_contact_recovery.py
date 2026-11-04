"""Contact-triggered Mink micro-correction after one simulated rear-foot swing.

A self-contained *diagnostic recovery* from recorded MuJoCo qpos/qvel. Not an
MPC and not a hardware control law. Compare unchanged goal and several small
corrections from EXACTLY the same physical state, then score contact forces
at every 2-ms timestep. This never teleports or fixes the floating base.
"""
from __future__ import annotations
import argparse,json
from pathlib import Path

import mink
import mujoco as mj
import numpy as np
from scipy.spatial import ConvexHull

from mink_contact_preflight import STANCE,PREFIX,load_context,StepPreflight,com_xyz


def assess_recovery(ctx, physical_result, push_mm, duration_s=.8):
    """Measured state and original motor goal are identical for all trials."""
    if push_mm<0 or push_mm>2.:raise ValueError('Diagnostic permitted foot adjustment: 0..2 mm')
    source=physical_result
    feet=set(source.get('end_loaded_legs',[]))
    missing=[k for k in STANCE if k not in feet]
    if len(missing)!=1 or len(feet)!=3:
        raise ValueError('Expected exactly one unloaded support foot after swing')
    unloaded=missing[0]
    step_leg='bl' if unloaded=='br' else 'br'
    if not source.get('end_qpos') or not source.get('end_qvel'):
        raise ValueError('Need saved physical qpos and qvel from independent rollout')
    # Rebuild original target from same quasi-static plan, NOT measured angles:
    # resetting all motor goals to measured angles would create a new artifact.
    math_plan=StepPreflight(ctx).attempt_rear_step(step_leg,(0.,0.,.024))
    if math_plan['status']!='ONE_STEP_STATIC_NODES_PASS':raise RuntimeError('Static step changed')
    original_goal=np.asarray(math_plan['nodes'][-1]['qpos'],float)
    m=ctx.model
    m.opt.timestep=.002
    m.geom_friction[m.geom('floor').id,0]=1.1
    for k in (*STANCE,'fl','fr'):
        m.geom_friction[m.geom(f'tibia{k}_collision_0').id,0]=1.1
    q=np.asarray(source['end_qpos'],float);v=np.asarray(source['end_qvel'],float)
    ids=np.asarray(m.actuator_trnid[:,0],int)
    qa=np.asarray(m.jnt_qposadr[ids],int);dofs=np.asarray(m.jnt_dofadr[ids],int)
    caps=np.asarray(m.actuator_ctrlrange[:,1]);initial_goal=original_goal[qa].copy()
    conf=mink.Configuration(m);conf.update(q)
    targets={leg:conf.get_transform_frame_to_world(PREFIX+leg,'site').as_matrix() for leg in STANCE}
    corr_goal=initial_goal.copy()
    if push_mm>0:
        tasks=[]
        for leg in STANCE:
            target=targets[leg].copy()
            if leg==unloaded:target[2,3]-=push_mm/1000.
            frame=mink.FrameTask(PREFIX+leg,'site',position_cost=10.,orientation_cost=0.)
            frame.set_target(mink.SE3.from_matrix(target));tasks.append(frame)
        posture=mink.PostureTask(m,cost=1e-5);posture.set_target_from_configuration(conf)
        error=float('inf')
        for k in range(70):
            inc=mink.solve_ik(conf,tasks+[posture],dt=1.,solver='daqp',damping=1e-8,
                              limits=ctx.limits,constraints=ctx.freeze)
            conf.update(conf.integrate(inc,dt=1.))
            err=[np.linalg.norm(conf.get_transform_frame_to_world(PREFIX+leg,'site').translation()-
                 np.asarray(targets[leg][:3,3])-np.array([0.,0.,-push_mm/1000.])*(leg==unloaded)) for leg in STANCE]
            error=float(max(err))
            if error<.0001:break
        if error>.0002:raise RuntimeError(f'Mink correction IK error {error:.6g}m')
        jointids=[int(m.body(part+unloaded).jntadr[0]) for part in ('coxa','femur','tibia')]
        jaddr=[int(m.jnt_qposadr[j]) for j in jointids]
        for idx in jaddr:
            actuator_idx=int(np.flatnonzero(qa==idx)[0])
            corr_goal[actuator_idx]+=conf.q[idx]-q[idx]
        if np.max(abs(conf.q[ctx.joint_qpos_addrs]))>1.700001:
            raise RuntimeError('IK correction exceeds research angle limit')
    d=mj.MjData(m);d.qpos[:]=q;d.qvel[:]=v;mj.mj_forward(m,d)
    floor=int(m.geom('floor').id)
    geomleg={int(m.geom(f'tibia{k}_collision_0').id):k for k in STANCE}
    min_legs=4;min_margin=float('inf');max_tau=0.;max_q=0.;saturations=0
    observations=[]
    for index in range(round(duration_s/.002)):
        frac=min(1.,(index+1)/125);ease=frac*frac*(3-2*frac)
        goal=initial_goal+(corr_goal-initial_goal)*ease
        u=7.*(goal-d.qpos[qa])-.32*d.qvel[dofs]
        saturations+=int(np.any(abs(u)>caps+1e-9))
        d.ctrl[:]=np.clip(u,-caps,caps)
        mj.mj_step(m,d)
        fz={k:0. for k in STANCE};pts={k:[] for k in STANCE}
        for ix in range(d.ncon):
            c=d.contact[ix]
            gid=int(c.geom2) if int(c.geom1)==floor else int(c.geom1) if int(c.geom2)==floor else -1
            if gid not in geomleg:continue
            leg=geomleg[gid];w=np.zeros(6);mj.mj_contactForce(m,d,ix,w)
            ff=c.frame.reshape(3,3).T@w[:3]
            fz[leg]+=float(ff[2]);pts[leg].append((c.pos.copy(),max(1e-8,float(ff[2]))))
        loaded=[leg for leg in STANCE if fz[leg]>.08]
        min_legs=min(min_legs,len(loaded))
        if len(loaded)>=3:
            points=np.array([np.average([p for p,w in pts[k]],axis=0,
                       weights=[w for p,w in pts[k]])[:2] for k in loaded])
            h=ConvexHull(points).equations
            com=com_xyz(m,d)[:2]
            margin=float(np.min(-(h[:,:2]@com+h[:,2]))*1000)
            min_margin=min(min_margin,margin)
        max_tau=max(max_tau,float(max(abs(d.ctrl))))
        max_q=max(max_q,float(max(abs(d.qpos[ctx.joint_qpos_addrs]))))
        observations.append({'loaded':list(loaded),'fz':fz})
    final=observations[-1]
    tail=observations[-round(.2/.002):]
    all4_tail=all(len(v['loaded'])==4 for v in tail)
    sustained=bool(all4_tail and min(final['fz'].values())>=.2)
    physical_gate=bool(sustained and min_legs>=3 and min_margin>0 and
                       max_q<=1.700001 and saturations==0)
    return {'push_mm':push_mm,'unloaded_leg':unloaded,
            'loaded_legs_at_end':final['loaded'],'Fz_at_end_N':final['fz'],
            'min_legs_all_timesteps':min_legs,'positive_margin_min_mm':min_margin,
            'min_0p2s_all_four_loaded':all4_tail,
            'all_four_force_bearing_at_end_ge_0p2N':min(final['fz'].values())>=.2,
            'max_joint_abs_rad':max_q,'max_motor_command_Nm':max_tau,
            'saturated_timesteps':saturations,'fraction_of_steps_four_loaded':
                sum(len(v['loaded'])==4 for v in observations)/len(observations),
            'recovery_passed_local_dynamics':physical_gate,
            'autonomous_second_step_authorized':False}


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--model',type=Path,required=True)
    p.add_argument('--safe-state-json',type=Path,required=True)
    p.add_argument('--dynamic-result',type=Path,required=True)
    p.add_argument('--out',type=Path)
    args=p.parse_args();ctx=load_context(args.model,args.safe_state_json)
    physical=json.loads(args.dynamic_result.read_text())
    cases={str(x):assess_recovery(ctx,physical,x) for x in (0.,.25,.5,1.,2.)}
    report={'schema':'cad30_contact_recovery_mink_feedback_v1','cases':cases,
            'contact_feedback_proven_in_nominal_simulation':any(x['recovery_passed_local_dynamics'] for x in cases.values()),
            'complete_four_leg_gait_proven':False,
            'limits':'One sample only; very small PWM-servo proxy; start derived from historical unsafe B; no full horizon and no hardware calibration'}
    payload=json.dumps(report,ensure_ascii=False,indent=2)
    if args.out:args.out.parent.mkdir(parents=True,exist_ok=True);args.out.write_text(payload+'\n')
    print(payload)

if __name__=='__main__':main()