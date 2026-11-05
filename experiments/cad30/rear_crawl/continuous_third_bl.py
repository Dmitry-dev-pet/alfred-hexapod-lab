"""Reproduce a measured CAD30 6-support -> ML -> MR -> ML-back-prep -> BL continuous MuJoCo sequence.

No robot actuation, no root teleport or reset between phases. Test/diagnostic only.
Uses the existing PR #45 optimizer and PR #49 MR Fz-feedback as a baseline.
"""
from __future__ import annotations
import argparse, gzip, hashlib, json, sys
from pathlib import Path
import numpy as np
from scipy.optimize import minimize
from scipy.spatial import ConvexHull


def compact(r):
    return {k: v for k, v in r.items() if k not in ('final_qpos', 'final_qvel', 'final_motor_goals')}


def required_com_shift_for_lift(xy, com_xy, leg, margin_mm=10):
    """Geometry-only minimum COM shift in the horizontal plane for 3 remaining contacts.

    This is NOT a dynamic prediction or a feasible servo trajectory.
    """
    others = tuple(k for k in ('ml', 'mr', 'bl', 'br') if k != leg)
    if not all(k in xy for k in others):
        return {'geometry_pass': False, 'reason': 'missing_contact'}
    eq = ConvexHull(np.array([xy[k] for k in others])).equations
    target = margin_mm/1000.
    constraints = [{'type':'ineq','fun':lambda a, n=n, c=c: -np.dot(n,com_xy+a)-c-target} for n,c in zip(eq[:,:2],eq[:,2])]
    res = minimize(lambda a: float(np.dot(a,a)), np.zeros(2), method='SLSQP', constraints=constraints,
                   options={'ftol':1e-13,'maxiter':300})
    mar0 = float(np.min(-(eq[:,:2] @ com_xy + eq[:,2]))*1000)
    return {'geometry_pass':bool(res.success), 'initial_signed_margin_mm':mar0,
            'target_signed_margin_mm':margin_mm, 'required_COM_dx_dy_mm':(res.x*1000).tolist(),
            'required_COM_displacement_mm':float(np.linalg.norm(res.x)*1000)}


def run(cad_root:Path, repo_root:Path, six_file:Path, math_trace:Path, negative_physics_trace:Path,
        optimized_ml:Path, out_dir:Path, record_raw=False, ml_back_mm=85.):
    cad_root=Path(cad_root).resolve();repo_root=Path(repo_root).resolve()
    sys.path[:0]=[str(cad_root),str(repo_root/'experiments/cad30/contact_optimization'),
                  str(repo_root/'experiments/cad30/next_middle'),str(Path(__file__).resolve().parent)]
    from contact_aware_shooting import PhysicalTrial, Command
    from replay_six_to_ml import replay_six_to_contact
    from mr_contact_continuation import Trial as OriginalTrial
    from lateral_foot_trial import Trial as LateralTrial
    out_dir=Path(out_dir);out_dir.mkdir(parents=True, exist_ok=True)
    study=PhysicalTrial(cad_root,repo_root,Path(math_trace),Path(negative_physics_trace),mu=1.1,full_log=False)
    (m,d),pre=replay_six_to_contact(study, Path(six_file), through_node=117,node_s=.3,keep_frames=False)
    if pre['status']!='PASS_TO_PRE_ML': raise RuntimeError('prelude did not reproduce: '+str(pre['status']))
    com_start=float(study.model_com(m,d)[1])-pre['COM_y_delta_mm_from_saved_six']/1000
    root_start=float(d.qpos[1])-pre['root_y_delta_mm_from_saved_six']/1000
    best=Command(**json.loads(Path(optimized_ml).read_text())['best']['command'])
    ml=study.rollout(best,live_model_data=(m,d))
    if not ml['accepted']: raise RuntimeError('ML did not reproduce: '+str(ml.get('status')))
    mr_t=OriginalTrial(cad_root,repo_root,Path(six_file),mu=1.1,leg='mr')
    mr,mr_frames=mr_t.run(step_mm=33,clearance_mm=18,swing_node_s=.35,pre_shift_xy_mm=(0.,20.),
        ml_servo_bias_rad=(0.,-.075,.055),hold_s=.6,live_model_data=(m,d,study.last_motor_goals),
        save_raw=record_raw,touchdown_feedback=True,touchdown_gain=4.,touchdown_damping=20.,
        touchdown_target_N=.8,touchdown_max_goal_rad=.06)
    if not mr['accepted']: raise RuntimeError('MR did not reproduce: '+str(mr.get('status')))
    ml_prep_t=LateralTrial(cad_root,repo_root,Path(six_file),mu=1.1,leg='ml')
    ml_prep,ml_prep_frames=ml_prep_t.run(step_mm=-float(ml_back_mm),lateral_step_mm=0,clearance_mm=18,
        swing_node_s=.24,hold_s=.8,initial_hold_s=.2,live_model_data=(m,d,np.asarray(mr['final_motor_goals'])),
        touchdown_feedback=True,save_raw=record_raw)
    prep_pass=(not ml_prep['failures'] and ml_prep['trio_bad_force_frames']==0 and
        ml_prep['min_margin_trio_mm']>=10 and ml_prep['max_leg_joint_rad']<=1.7 and
        ml_prep['max_raw_tau_Nm']<=.7 and ml_prep['last_hold_fraction_four_loaded']>=.99 and
        abs(ml_prep['physical_toe_forward_mm']+ml_back_mm)<10 and
        ml_prep['mesh_above_2mm_no_contact_sampled_s']>=.25)
    if not prep_pass: raise RuntimeError('ML prep did not satisfy contact/safety gate')
    bl_t=OriginalTrial(cad_root,repo_root,Path(six_file),mu=1.1,leg='bl')
    bl,bl_frames=bl_t.run(step_mm=30,clearance_mm=15,swing_node_s=.28,hold_s=.8,initial_hold_s=.2,
        live_model_data=(m,d,np.asarray(ml_prep['final_motor_goals'])),touchdown_feedback=True,save_raw=record_raw)
    floor=int(m.geom('floor').id)
    leg_geoms={int(m.geom(f'tibia{k}_collision_0').id):k for k in ('ml','mr','bl','br','fl','fr')}
    fz,xy,_=bl_t.read_contact(m,d,floor,leg_geoms)
    br_geom=required_com_shift_for_lift(xy,study.model_com(m,d)[:2],'br',margin_mm=10.)
    result={'schema':'CAD30_contiguous_6_ML_MR_MLprep_BL_v1','model':'Freenove FNK0052 original CAD30, MuJoCo 3.14',
        'mu':1.1,'time_step_s':.002,'nominal_tau_cap_Nm':.7,'nominal_joint_bound_rad':1.7,
        'input_six':'saved Pupper six-contact state AFTER prebalance (not original raw start)',
        'prelude':{'status':pre['status'],'physics_steps':pre['physics_steps']},
        'first_ml':compact(ml),'second_mr':compact(mr), 'additional_ml_backward_prep':compact(ml_prep),
        'ml_prep_accepted':prep_pass, 'third_bl':compact(bl),'third_bl_accepted':bool(bl['accepted']),
        'br_after_bl_geometry':br_geom, 'net_COM_Y_mm_from_saved_six':(float(study.model_com(m,d)[1])-com_start)*1000,
        'net_root_Y_mm_from_saved_six':(float(d.qpos[1])-root_start)*1000,
        'all_phases_same_MjData':True, 'root_qpos_not_overwritten_during_simulation':True,
        'all_four_legs_swing_forward':False,'repeatable_positive_net_gait':False,
        'private_CAD_meshes_in_artifact':False}
    (out_dir/'continuous_third_bl_summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    (out_dir/'after_bl_physical_state.json').write_text(json.dumps({'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist(),
        'motor_goals':bl['final_motor_goals']},indent=2)+'\n')
    if record_raw:
        for name,frames in [('mr',mr_frames),('ml_preparation',ml_prep_frames),('bl',bl_frames)]:
            dest=out_dir/f'continuous_{name}_2ms.json.gz'
            with gzip.open(dest,'wt') as f:json.dump({'schema':'CAD30_2ms_actual_physical','name':name,'frames':frames},f)
            result.setdefault('trace_sha256',{})[name]=hashlib.sha256(dest.read_bytes()).hexdigest()
        (out_dir/'continuous_third_bl_summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--cad-root',required=True,type=Path)
    p.add_argument('--repo-root',required=True,type=Path);p.add_argument('--six-file',required=True,type=Path)
    p.add_argument('--math-trace',required=True,type=Path);p.add_argument('--negative-physics-trace',required=True,type=Path)
    p.add_argument('--optimized-ml',required=True,type=Path);p.add_argument('--out-dir',required=True,type=Path)
    p.add_argument('--raw',action='store_true');p.add_argument('--ml-back-mm',type=float,default=85.)
    args=p.parse_args();r=run(args.cad_root,args.repo_root,args.six_file,args.math_trace,args.negative_physics_trace,
                           args.optimized_ml,args.out_dir,record_raw=args.raw,ml_back_mm=args.ml_back_mm)
    print(json.dumps({'third_bl_accepted':r['third_bl_accepted'], 'net_COM_Y_mm_from_saved_six':r['net_COM_Y_mm_from_saved_six'],
                      'bl_forward_mm':r['third_bl']['physical_toe_forward_mm'],
                      'bl_min_actual_tripod_margin_mm':r['third_bl']['min_margin_trio_mm'],
                      'br_required_com_shift_mm':r['br_after_bl_geometry']['required_COM_displacement_mm']},indent=2))
