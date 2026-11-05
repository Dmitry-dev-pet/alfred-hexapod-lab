"""One middle-leg step: optimize *commands* with independent MuJoCo rollouts.

Objective: take the recorded physically obtained four-loaded state just before
ML swing, move COM with actual PD servo commands, and genuinely lift, swing and
land ML while MR/BL/BR remain physically load-bearing. The result is one local
step only, NOT a full four-leg gait or a hardware-ready controller.

Input (private and local): original CAD30 MJCF+meshes, 218-node math trace,
2-ms independent physics replay from PR #44 (saved qpos/qvel and phase IDs).
Code carries no Freenove proprietary meshes. Never commands real hardware.

Upstream numerical tools: MuJoCo 3.14 physics, SciPy SLSQP (2D target) and
SciPy differential_evolution (physics-in-the-loop command search), inherited
CAD30 IK from PR #43. No root qpos is changed after initialization.
"""
from __future__ import annotations
import argparse
import gzip
import hashlib
import json
import sys
from pathlib import Path
from dataclasses import dataclass

import numpy as np
import mujoco as mj
from scipy.optimize import minimize, differential_evolution
from scipy.spatial import ConvexHull

LEG_CODES = ('ml', 'mr', 'bl', 'br', 'fl', 'fr')
TRIPOD = ('mr', 'bl', 'br')
WALKING = ('ml', 'mr', 'bl', 'br')
LOADED_N = .2
FULL_MODEL_DT = .002
MODEL_TAU_CAP = .7
MODEL_Q_CAP = 1.7


def load_helpers(cad_root: Path, repo_root: Path):
    """Reuse the exact earlier CAD IK and contact readers, don't copy models."""
    sys.path.insert(0, str(cad_root))
    sys.path.insert(0, str(repo_root/'experiments/cad30/pupper_baseline'))
    sys.path.insert(0, str(repo_root/'experiments/cad30/math_first_dynamic'))
    from math_first_four_crawl_v1 import ModelStudy
    from dynamic_replay_math_first import read_contact, model_com
    from hexlab.stand_geometry_v04 import load_legs, tip_world, geom_mesh_world_vertices
    return ModelStudy,read_contact,model_com,load_legs,tip_world,geom_mesh_world_vertices


def load_checkpoint(gzip_path: Path, node: int = 117):
    with gzip.open(gzip_path, 'rt') as f:
        payload=json.load(f)
    arr=[x for x in payload['frames'] if x.get('i')==node]
    if not arr: raise ValueError(f'No node {node} in the physical 2ms trace')
    result=arr[-1]
    if result.get('phase')!='before_ml_body_shift':
        raise ValueError(f'Unexpected node phase: {result.get("phase")}')
    if not all(k in result for k in ('qpos','qvel','fz_N')):raise ValueError('Missing actual physical state')
    return result


def triangle_margin_mm(xy, com_xy, names=TRIPOD):
    if not all(n in xy for n in names): return float('-inf')
    pts=np.asarray([xy[n] for n in names],float)
    try: equations=ConvexHull(pts).equations
    except Exception:return float('-inf')
    return float(np.min(-(equations[:,:2]@com_xy+equations[:,2]))*1000.)


def minimal_com_translation(com_xy, xy, required_margin_mm):
    """Minimal planar COM shift to the interior of the current contact triangle.

    This target is geometric. Actual force/COM are checked by MuJoCo, not
    assumed to be achieved. SLSQP is used only on linear triangle inequalities.
    """
    if not all(n in xy for n in TRIPOD):raise ValueError('No current three-contact footprint')
    eq=ConvexHull([xy[k] for k in TRIPOD]).equations
    def inequalities(delta):
        return -(eq[:,:2]@(np.asarray(com_xy)+delta)+eq[:,2])-required_margin_mm/1000.
    result=minimize(lambda a:float(np.dot(a,a)),np.zeros(2),method='SLSQP',
        constraints=[{'type':'ineq','fun':inequalities}],options={'ftol':1e-12,'maxiter':100})
    if not result.success or min(inequalities(result.x)) < -1e-7:
        raise RuntimeError('COM target projection infeasible')
    return np.asarray(result.x)


def smoothstep(t):
    a=float(np.clip(t,0.,1.));return a*a*(3-2*a)


@dataclass(frozen=True)
class Command:
    com_margin_mm: float
    step_command_mm: float
    swing_height_command_mm: float


class PhysicalTrial:
    def __init__(self, cad_root: Path, repo_root: Path, math_trace: Path, checkpoint: Path,
                 mu: float=1.1, checkpoint_node: int=117, full_log: bool=False):
        self.cad_root=Path(cad_root);self.repo_root=Path(repo_root)
        self.ModelStudy,self.read_contact,self.model_com,self.load_legs,self.tip_world,self.mesh_vertices=load_helpers(self.cad_root,self.repo_root)
        with open(math_trace) as f:self.math_nodes=json.load(f)['nodes']
        self.measured=load_checkpoint(Path(checkpoint), checkpoint_node)
        self.model_path=self.cad_root/'source/mjcf/scene_local30_nominal.xml'
        self.mu=mu;self.node=checkpoint_node;self.full_log=full_log

    def make(self):
        m=mj.MjModel.from_xml_path(str(self.model_path));m.opt.timestep=FULL_MODEL_DT
        d=mj.MjData(m);d.qpos[:]=self.measured['qpos'];d.qvel[:]=self.measured['qvel']
        floor=int(m.geom('floor').id)
        geom_map={int(m.geom(f'tibia{k}_collision_0').id):k for k in LEG_CODES}
        for gid in (*geom_map.keys(),floor):m.geom_friction[gid,0]=self.mu
        mj.mj_forward(m,d)
        jid=np.asarray(m.actuator_trnid[:,0],dtype=int)
        ja=np.asarray(m.jnt_qposadr[jid],dtype=int);va=np.asarray(m.jnt_dofadr[jid],dtype=int)
        assert len(ja)==20
        base_goals=np.asarray(self.math_nodes[self.node]['qpos'],float)[ja]
        legs={leg['suffix']:leg for leg in self.load_legs(m,d,mj)}
        return m,d,floor,geom_map,ja,va,base_goals,legs

    def rollout(self, action: Command, prebalance_s: float=1.0, swing_node_s: float=.22,
                hold_s:float=.4, live_model_data=None):
        if live_model_data is None:
            m,d,floor,geom_map,ja,va,commands,legs=self.make()
        else:
            m,d=live_model_data
            floor=int(m.geom('floor').id)
            geom_map={int(m.geom(f'tibia{k}_collision_0').id):k for k in LEG_CODES}
            jids=np.asarray(m.actuator_trnid[:,0],int)
            ja=np.asarray(m.jnt_qposadr[jids],int);va=np.asarray(m.jnt_dofadr[jids],int)
            commands=np.asarray(self.math_nodes[self.node]['qpos'],float)[ja]
            legs={leg['suffix']:leg for leg in self.load_legs(m,d,mj)}
        start_com=self.model_com(m,d).copy();start_root=d.qpos[:3].copy()
        initial_fz,xy,_=self.read_contact(m,d,floor,geom_map)
        initial_triangle=triangle_margin_mm(xy,start_com[:2])
        offset=minimal_com_translation(start_com[:2],xy,action.com_margin_mm)
        # Four stance-foot targets anchored in *observed* world-frame geometry.
        study=self.ModelStudy(self.model_path,d.qpos.copy())
        original_toes={k:v.copy() for k,v in study.original_feet.items()}
        q_target,ikerrors=study.ik(d.qpos[:2]+offset,{k:original_toes[k] for k in WALKING},d.qpos.copy(),anchored=WALKING)
        pre_command=q_target[ja].copy();pre_command[18:]=commands[18:]
        # No direct write to the root after this point. Dynamics only.
        current=commands.copy();min_tripod_fz=1000.;min_margin_during_swing=1000.
        min_margin_entire=1000.;peak_tau=0.;peak_raw_tau=0.;peak_q=0.;min_contact_count=6
        preload_fz=None;preload_margin=None;first_fail=None;frames=[];swing_metrics=[]
        end_hold_4_loaded=0;end_hold_total=0
        ml_id=int(legs['ml']['geom']);zero_pair_t=0.;air_mesh_t=0.;air_toe_t=0.;max_mesh_z=-1000.;max_ml_tip_z=-1000.;air_forward_y=[]
        tripod_references=None;max_stance_marker_drift_mm=0.
        def drive(target,seconds,phase):
            nonlocal current,min_tripod_fz,min_margin_during_swing,min_margin_entire,peak_tau,peak_raw_tau,peak_q
            nonlocal min_contact_count,first_fail,zero_pair_t,air_mesh_t,air_toe_t,max_mesh_z,max_ml_tip_z,end_hold_4_loaded,end_hold_total,tripod_references,max_stance_marker_drift_mm
            start=current.copy();n=max(1,round(seconds/FULL_MODEL_DT))
            for tick in range(n):
                a=smoothstep((tick+1)/n);motor_goal=start+a*(target-start)
                raw=7*(motor_goal-d.qpos[ja])-.32*d.qvel[va]
                applied=np.clip(raw,m.actuator_ctrlrange[:,0],m.actuator_ctrlrange[:,1]);d.ctrl[:]=applied
                mj.mj_step(m,d)
                fz,contactxy,bad=self.read_contact(m,d,floor,geom_map)
                com=self.model_com(m,d);tri_margin=triangle_margin_mm(contactxy,com[:2])
                min_margin_entire=min(min_margin_entire,tri_margin)
                if phase=='swing_ml':
                    min_tripod_fz=min(min_tripod_fz,*(fz[k] for k in TRIPOD))
                    min_margin_during_swing=min(min_margin_during_swing,tri_margin)
                peak_tau=max(peak_tau,float(max(abs(applied[:18]))))
                peak_raw_tau=max(peak_raw_tau,float(max(abs(raw[:18]))))
                peak_q=max(peak_q,float(max(abs(d.qpos[ja[:18]]))))
                count=sum(fz[k]>.08 for k in WALKING);min_contact_count=min(min_contact_count,count)
                zero_pairs = not any((int(c.geom1)==floor and int(c.geom2)==ml_id) or
                                     (int(c.geom2)==floor and int(c.geom1)==ml_id) for c in d.contact[:d.ncon])
                if phase=='swing_ml':
                    if tripod_references is None:tripod_references={k:self.tip_world(m,d,legs[k])[:2].copy() for k in TRIPOD}
                    for k in TRIPOD:
                        if fz[k]>LOADED_N:
                            dist=float(np.linalg.norm(self.tip_world(m,d,legs[k])[:2]-tripod_references[k])*1000)
                            max_stance_marker_drift_mm=max(max_stance_marker_drift_mm,dist)
                    if zero_pairs:zero_pair_t+=FULL_MODEL_DT
                    ml_xyz=self.tip_world(m,d,legs['ml'])
                    if zero_pairs and float(ml_xyz[2])>=.002:
                        air_toe_t+=FULL_MODEL_DT;air_forward_y.append(float(ml_xyz[1]))
                    max_ml_tip_z=max(max_ml_tip_z,float(ml_xyz[2]))
                    if tick%10==0:
                        zmin=float(self.mesh_vertices(m,d,ml_id)[:,2].min())
                        max_mesh_z=max(max_mesh_z,zmin)
                        if zero_pairs and zmin>=.002:air_mesh_t+=FULL_MODEL_DT*10
                if phase=='hold_after_touchdown':
                    end_hold_total+=1
                    if all(fz[k]>.2 for k in WALKING):end_hold_4_loaded+=1
                if phase=='swing_ml':
                    violated=min(fz[k] for k in TRIPOD)<.2 or tri_margin<.005*1000
                else:
                    violated=min(fz[k] for k in TRIPOD)<.2 or tri_margin<.005*1000
                reason=None
                if violated:reason='loss_of_required_support_or_margin'
                elif peak_raw_tau>MODEL_TAU_CAP+1e-5:reason='raw_torque_limit'
                elif peak_q>MODEL_Q_CAP+1e-5:reason='joint_limit'
                elif not np.all(np.isfinite(d.qpos)):reason='nonfinite_state'
                if first_fail is None and reason:
                    first_fail={'phase':phase,'physics_t_s':float(d.time),'reason':reason,'tripod_fz':{k:fz[k] for k in TRIPOD},'triangle_margin_mm':tri_margin}
                if self.full_log or tick%25==0 or reason:
                    row={'time_s':round(float(d.time),5),'phase':phase,'Fz_N':{k:round(float(v),5) for k,v in fz.items()},
                         'tripod_margin_mm':round(tri_margin,4),'ml_floor_contact_pairs':0 if zero_pairs else 1,
                         'COM_world_xyz_m':com.tolist(),'qpos':d.qpos.tolist() if self.full_log else None,
                         'qvel':d.qvel.tolist() if self.full_log else None}
                    frames.append(row)
                # Do not stop: record every negative counterfactual to terminal for analysis.
            current=target.copy()

        drive(pre_command,prebalance_s,'physical_prebalance')
        drive(pre_command,.2,'prebalance_hold')
        pre_fz,pre_xy,_=self.read_contact(m,d,floor,geom_map)
        preload_fz=pre_fz.copy();preload_margin=triangle_margin_mm(pre_xy,self.model_com(m,d)[:2])
        actual_prebalance_q=d.qpos.copy();actual_prebalance_com=self.model_com(m,d).copy()
        # Resolve swing IK against observed physical pose, retaining stance PD targets.
        study2=self.ModelStudy(self.model_path,d.qpos.copy())
        foot_original=study2.original_feet['ml'].copy()
        ml_qidx=[int(m.body(a+'ml').jntadr[0]) for a in ('coxa','femur','tibia')]
        ml_act=[i for i in range(m.nu) if int(m.actuator_trnid[i,0]) in ml_qidx]
        if len(ml_act)!=3:raise RuntimeError('Unexpected motor map for ML')
        plan_seed=actual_prebalance_q.copy();target_cmd=pre_command.copy();swing_ik_errors=[]
        for i in range(1,10):
            s=i/9.
            toe_goal=foot_original.copy()
            toe_goal[1]+=action.step_command_mm/1000*s
            toe_goal[2]+=action.swing_height_command_mm/1000*np.sin(np.pi*s)
            qnode,errs=study2.ik(actual_prebalance_q[:2],{'ml':toe_goal},plan_seed,anchored=('ml',))
            plan_seed=qnode.copy();swing_ik_errors.extend(errs.values())
            target_cmd[ml_act]=qnode[ja[ml_act]]
            drive(target_cmd,swing_node_s,'swing_ml')
        drive(target_cmd,hold_s,'hold_after_touchdown')
        final_fz,final_xy,_=self.read_contact(m,d,floor,geom_map)
        final_com=self.model_com(m,d).copy();final_ml_tip=self.tip_world(m,d,legs['ml']).copy()
        actual_forward_mm=float((final_ml_tip[1]-foot_original[1])*1000.)
        actual_air_forward_mm=float((max(air_forward_y)-min(air_forward_y))*1000) if air_forward_y else 0.
        results={'status':'NOT_EVALUATED','input':'measured physical checkpoint PR#44 node 117, no artificial root motion',
          'command':vars(action),'mu':self.mu,'physics_dt_s':FULL_MODEL_DT,'checkpoint_node':self.node,
          'initial_fz_N':initial_fz,'initial_tripod_margin_mm':initial_triangle,
          'virtual_com_shift_target_xy_mm':(offset*1000).tolist(),'max_ik_toe_error_mm':max((*ikerrors.values(),*swing_ik_errors)),
          'max_stance_toe_marker_xy_drift_mm':max_stance_marker_drift_mm,
          'real_com_prebalance_xy_mm':((actual_prebalance_com[:2]-start_com[:2])*1000).tolist(),
          'prebalance_actual_tripod_margin_mm':preload_margin,'prebalance_Fz_N':preload_fz,
          'swing_min_tripod_Fz_N':min_tripod_fz,'swing_min_tripod_margin_mm':min_margin_during_swing,
          'minimum_tripod_margin_whole_test_mm':min_margin_entire,'ml_zero_contact_pairs_total_s':zero_pair_t,
          'ml_toe_height_over_2mm_no_contact_s':air_toe_t,'ml_mesh_height_over_2mm_no_contact_s_sampled':air_mesh_t,
          'ml_mesh_peak_height_mm_sampled':max_mesh_z*1000,'ml_toe_peak_height_mm':max_ml_tip_z*1000,
          'ml_in_air_forward_mm':actual_air_forward_mm,'actual_ml_world_forward_mm':actual_forward_mm,
          'final_Fz_N':final_fz,'final_tripod_margin_mm':triangle_margin_mm(final_xy,final_com[:2]),
          'final_four_loaded_last_hold_fraction':end_hold_4_loaded/max(1,end_hold_total),
          'max_leg_tau_Nm':peak_tau,'max_raw_leg_tau_Nm':peak_raw_tau,'max_leg_joint_abs_rad':peak_q,
          'min_four_legs_loaded_count':min_contact_count,'first_guard_violation':first_fail,
          'root_world_y_delta_mm':float((d.qpos[1]-start_root[1])*1000),
          'COM_world_y_delta_mm':float((final_com[1]-start_com[1])*1000),
          'source_has_hardware_calibrated_limits':False,
          'warning':'One middle-leg step from checkpoint only; not a complete gait; contact parameters provisional.'}
        results['accepted']=accepted(results)
        results['status']='LOCAL_ML_LIFT_SWING_TOUCHDOWN_PASS' if results['accepted'] else 'FAIL'
        if self.full_log:
            results['_raw_log']=frames
        self.last_model_data=(m,d)
        self.last_motor_goals=target_cmd.copy()
        return results


def accepted(res):
    if res.get('first_guard_violation') is not None:return False
    if res['swing_min_tripod_Fz_N']<.2:return False
    if res['swing_min_tripod_margin_mm']<10:return False
    if res['ml_mesh_height_over_2mm_no_contact_s_sampled']<.25:return False
    if res['ml_in_air_forward_mm']<10:return False
    if res['max_ik_toe_error_mm']>2:return False
    if res['max_stance_toe_marker_xy_drift_mm']>5:return False
    if res['actual_ml_world_forward_mm']<20:return False
    if res['final_four_loaded_last_hold_fraction']<.99:return False
    if any(res['final_Fz_N'][k]<.2 for k in WALKING):return False
    if res['max_raw_leg_tau_Nm']>MODEL_TAU_CAP or res['max_leg_joint_abs_rad']>MODEL_Q_CAP:return False
    return True


def objective(res):
    """Minimize physical COM shift while enforcing measured contact viability.

    All penalties come from real MuJoCo data, not virtual LP contact forces.
    """
    a=res['command'];score=abs(a['com_margin_mm'])
    score+=.015*a['step_command_mm']+.01*a['swing_height_command_mm']
    score+=15*max(0,20-res['actual_ml_world_forward_mm'])
    score+=20*max(0,10-res['ml_in_air_forward_mm'])
    score+=60*max(0,.25-res['ml_mesh_height_over_2mm_no_contact_s_sampled'])
    score+=5*max(0,10-res['swing_min_tripod_margin_mm']) if np.isfinite(res['swing_min_tripod_margin_mm']) else 500
    score+=150*max(0,.2-res['swing_min_tripod_Fz_N'])
    score+=20*max(0,.99-res['final_four_loaded_last_hold_fraction'])
    score+=4*max(0,res['max_ik_toe_error_mm']-1)
    score+=2*max(0,res['max_stance_toe_marker_xy_drift_mm']-3)
    score+=500*max(0,res['max_leg_joint_abs_rad']-MODEL_Q_CAP)
    score+=500*max(0,res['max_raw_leg_tau_Nm']-MODEL_TAU_CAP)
    if res['first_guard_violation'] is not None:score+=500
    if not res['accepted']:score+=100
    return float(score)


def optimize(trial, maxiter=3, popsize=4, seed=17):
    tested=[];cache={}
    bounds=[(20,28),(28,39),(13,22)]
    def evaluate(p):
        param=Command(*map(float,p));key=tuple(round(a,5) for a in p)
        if key in cache:return cache[key]
        res=trial.rollout(param)
        score=objective(res);res['optimization_objective']=score
        tested.append({k:v for k,v in res.items() if k!='_raw_log'});cache[key]=score
        if len(tested)%3==0:print(f'optimizer trials={len(tested)} feasible={sum(r["accepted"] for r in tested)} score={score:.2f}',flush=True)
        return score
    # Deterministic safe seed plus broad neighboring candidates. Best is not a promise of full gait.
    start=np.array([[25,35,18],[24.5,35,18],[25.5,35,18],[26,34,18],
                    [25,34,16],[25,35,19],[25,36,20],[24,36,18],
                    [23,32,18],[27,34,18],[25,38,17],[26,36,21]],dtype=float)
    dres=differential_evolution(evaluate,bounds,init=start,maxiter=maxiter,popsize=popsize,
                               seed=seed,polish=False,workers=1,updating='immediate',tol=0)
    passed=[r for r in tested if r['accepted']]
    best=min(passed,key=lambda r:r['optimization_objective']) if passed else min(tested,key=lambda r:r['optimization_objective'])
    return {'search_algorithm':'SciPy differential_evolution, MuJoCo free-base nested rollout','seed':seed,
      'candidate_trials':len(tested),'feasible_trials':len(passed),'status':'FEASIBLE_LOCAL_SWING' if passed else 'NO_PASS_FOUND',
      'best':best,'scipy_result':{'fun':float(dres.fun),'nit':int(dres.nit)},'trials':tested}


def sha256(path:Path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for data in iter(lambda:f.read(2**20),b''):h.update(data)
    return h.hexdigest()


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--cad-root',type=Path,required=True)
    p.add_argument('--repo-root',type=Path,default=Path(__file__).resolve().parents[3])
    p.add_argument('--math-trace',type=Path,required=True)
    p.add_argument('--physical-trace',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--maxiter',type=int,default=3)
    p.add_argument('--mu',type=float,default=1.1)
    args=p.parse_args();trial=PhysicalTrial(args.cad_root,args.repo_root,args.math_trace,args.physical_trace,mu=args.mu)
    result=optimize(trial,maxiter=args.maxiter)
    chosen=result['best']['command']
    again=PhysicalTrial(args.cad_root,args.repo_root,args.math_trace,args.physical_trace,mu=args.mu,full_log=True)
    verification=again.rollout(Command(**chosen))
    raw=verification.pop('_raw_log',[])
    args.out.parent.mkdir(parents=True,exist_ok=True)
    gzfile=args.out.with_name(args.out.stem+'_2ms.json.gz')
    with gzip.open(gzfile,'wt') as f:json.dump({'schema':'cad30-contact-optimization-2ms-replay-v1','frames':raw},f)
    result['independent_rerun']=verification
    result['provenance']={'math_sha256':sha256(args.math_trace),'checkpoint_sha256':sha256(args.physical_trace),
                          'raw_2ms_sha256':sha256(gzfile),'cad_model_path':'private CAD30, not uploaded'}
    args.out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('trials',)},ensure_ascii=False,indent=2))

if __name__=='__main__':main()
