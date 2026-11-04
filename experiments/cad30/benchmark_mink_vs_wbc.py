"""Head-to-head *kinematic* Mink 1.4 / local WBC comparison on one CAD30 pose.

No dynamics/physics integration; base is moved virtually by the exact same
specified 6D increment for both methods. Baseline stance targets are copied
from actual recorded physical contact markers before virtual perturbation.
All front/head joints and floating base are frozen in the Mink optimization.
Both methods have the same stance-joint per-step ±0.02rad envelope and model
joint bounds. Different optimization algorithms/weights are disclosed.
"""
from __future__ import annotations
from pathlib import Path
import argparse, json, platform, sys, time, statistics, hashlib
from importlib.metadata import version
import numpy as np
import mujoco
import mink

from mink_cad30_probe import contact_sites, STANCE, FRONT, MARKER_PREFIX, actual_violations
from wbc_contact_correction import micro_correct_stance

CASES={
  "forward_0p35mm": [0.,0.00035,0.,0.,0.,0.],
  "backward_0p35mm": [0.,-0.00035,0.,0.,0.,0.],
  "lateral_0p35mm": [0.00035,0.,0.,0.,0.,0.],
  "lift_body_0p25mm": [0.,0.,0.00025,0.,0.,0.],
  "lower_body_0p25mm": [0.,0.,-0.00025,0.,0.,0.],
  "forward_down": [0.,0.00035,-0.00010,0.,0.,0.],
  "roll_0p3mrad": [0.,0.,0.,0.0003,0.,0.],
  "pitch_0p3mrad": [0.,0.,0.,0.,0.0003,0.],
  "yaw_0p3mrad": [0.,0.,0.,0.,0.,0.0003],
  "combined_twist": [0.,0.00035,-0.00010,0.0003,-0.0002,0.00015],
}

def tracked_positions(model, q, sites):
  d=mujoco.MjData(model);d.qpos[:]=q;mujoco.mj_forward(model,d)
  return np.vstack([np.asarray(d.site_xpos[sites[leg]]).copy() for leg in STANCE])

def q_integrate(model,q,v):
  after=q.copy();mujoco.mj_integratePos(model,after,v,1.)
  return after


def setup_problem(model,q,ref_sites):
  d=mujoco.MjData(model); d.qpos[:]=q;d.qvel[:]=0.;mujoco.mj_forward(model,d)
  sites={leg:int(model.site(MARKER_PREFIX+leg).id) for leg in STANCE}
  anchors=tracked_positions(model,q,sites)
  leg_jids=[]
  for leg in (*STANCE,*FRONT):
    for part in ('coxa','femur','tibia'):
      body=model.body(f'{part}{leg}')
      jid=int(body.jntadr[0]);leg_jids.append(jid)
  if len(leg_jids)!=18 or len(set(leg_jids))!=18:raise AssertionError('18 leg joints expected')
  dofs=np.asarray([int(model.jnt_dofadr[j]) for j in leg_jids],int)
  qa=np.asarray([int(model.jnt_qposadr[j]) for j in leg_jids],int)
  lo=np.asarray([float(model.jnt_range[j][0]) for j in leg_jids]);hi=np.asarray([float(model.jnt_range[j][1]) for j in leg_jids])
  J=np.zeros((12,model.nv))
  for i,leg in enumerate(STANCE):
    jp=np.zeros((3,model.nv));jr=np.zeros((3,model.nv))
    mujoco.mj_jacSite(model,d,jp,jr,sites[leg]);J[3*i:3*i+3]=jp
  all_hinge_dofs=[int(model.jnt_dofadr[j]) for j in range(model.njnt) if model.jnt_type[j]==mujoco.mjtJoint.mjJNT_HINGE]
  # Freeze floating base, front pair and head in Mink; only 12 stance hinges allowed.
  stance_dofs=set(int(x) for x in dofs[:12]);frozen=sorted(set(range(6))|(set(all_hinge_dofs)-stance_dofs))
  velocity={str(model.joint(j).name):np.array([0.02]) for j in leg_jids[:12]}
  return {'data':d,'sites':sites,'anchors':anchors,'dofs':dofs,'qa':qa,'lo':lo,'hi':hi,'J':J,'frozen':frozen,'velocity':velocity}


def solve_custom(model,q,delta,common):
  tic=time.perf_counter_ns()
  res=micro_correct_stance(joint_positions_rad=q[common['qa']],lower_limits_rad=common['lo'],upper_limits_rad=common['hi'],stance_jacobian_base=common['J'][:,:6],stance_jacobian_joints=common['J'][:,common['dofs']],desired_base_delta=delta,desired_joint_delta=np.zeros(18),max_joint_delta_rad=.02,max_contact_linear_residual_m=2e-5,joint_reference_weight=1e-4)
  elapsed=time.perf_counter_ns()-tic
  if not res.approved_kinematic:return {'ok':False,'reason':res.reason,'time_ns':elapsed}
  v=np.zeros(model.nv);v[:6]=delta;v[common['dofs']]=res.joint_delta_rad
  after=q_integrate(model,q,v)
  drift=np.linalg.norm(tracked_positions(model,after,common['sites'])-common['anchors'],axis=1)
  return {'ok':True,'time_ns':elapsed,'q':after,'max_foot_drift_mm':float(np.max(drift)*1e3),'rms_foot_drift_mm':float(np.sqrt(np.mean(drift**2))*1e3),'max_joint_delta_rad':float(np.max(abs(np.asarray(res.joint_delta_rad)))),'foot_movement_mm':(drift*1e3).tolist()}


def solve_mink(model,q,delta,common):
  # Choose a single commanded base pose BEFORE both algorithms change joints.
  shift=np.zeros(model.nv);shift[:6]=delta
  shifted=q_integrate(model,q,shift)
  conf=mink.Configuration(model);conf.update(shifted)
  ref=mink.Configuration(model);ref.update(q)
  tasks=[]
  for leg in STANCE:
    task=mink.FrameTask(MARKER_PREFIX+leg,'site',position_cost=1.,orientation_cost=0.)
    task.set_target_from_configuration(ref)
    tasks.append(task)
  posture=mink.PostureTask(model,cost=1e-4)
  posture.set_target_from_configuration(conf)
  tasks.append(posture)
  limits=[mink.ConfigurationLimit(model),mink.VelocityLimit(model,common['velocity'])]
  freeze=[mink.DofFreezingTask(model,dof_indices=common['frozen'])]
  tic=time.perf_counter_ns()
  try: v=mink.solve_ik(conf,tasks,dt=1.,solver='daqp',damping=1e-9,limits=limits,constraints=freeze)
  except Exception as exc: return {'ok':False,'reason':f'{type(exc).__name__}: {exc}','time_ns':time.perf_counter_ns()-tic}
  elapsed=time.perf_counter_ns()-tic
  after=conf.integrate(v,dt=1.)
  drift=np.linalg.norm(tracked_positions(model,after,common['sites'])-common['anchors'],axis=1)
  # Guarantee exactly the same prescribed 6D base as custom.
  base_dof=v[:6].copy()
  return {'ok':True,'time_ns':elapsed,'q':after,'max_foot_drift_mm':float(np.max(drift)*1e3),'rms_foot_drift_mm':float(np.sqrt(np.mean(drift**2))*1e3),'max_joint_delta_rad':float(np.max(abs(after[common['qa']]-q[common['qa']]))),'max_frozen_base_velocity':float(np.max(abs(base_dof))),'foot_movement_mm':(drift*1e3).tolist()}


def benchmark(model_path,pose_zip,n_repeat=20):
  model,qpos,_,markers_error=contact_sites(model_path,pose_zip)
  common=setup_problem(model,qpos,None)
  bth={}
  for name,vec in CASES.items():
    delta=np.array(vec,dtype=float)
    local=solve_custom(model,qpos,delta,common)
    upstream=solve_mink(model,qpos,delta,common)
    # Timings include solver but exclude environment setup or MuJoCo mj_forward.
    if local['ok']:
      local_times=[]; local_total=[]
      for _ in range(n_repeat):
        tic=time.perf_counter_ns()
        candidate=solve_custom(model,qpos,delta,common)
        local_total.append(time.perf_counter_ns()-tic)
        local_times.append(candidate['time_ns'])
      local['median_solver_ms']=float(np.median(local_times)/1e6)
      local['median_call_ms']=float(np.median(local_total)/1e6)
      local.pop('time_ns',None);local.pop('q',None)
    if upstream['ok']:
      # Mink task setup is excluded; direct solver time only, along with config unchanged.
      upstream_times=[]; upstream_total=[]
      for _ in range(n_repeat):
        tic=time.perf_counter_ns()
        candidate=solve_mink(model,qpos,delta,common)
        upstream_total.append(time.perf_counter_ns()-tic)
        upstream_times.append(candidate['time_ns'])
      upstream['median_solver_ms']=float(np.median(upstream_times)/1e6)
      upstream['median_call_ms']=float(np.median(upstream_total)/1e6)
      upstream.pop('time_ns',None);upstream.pop('q',None)
    bth[name]={'requested_body_delta_world':vec,'custom':local,'mink':upstream}
  passed=all(z['custom'].get('ok') and z['mink'].get('ok') and z['custom']['max_foot_drift_mm']<0.02 and z['mink']['max_foot_drift_mm']<0.02 and z['mink']['max_frozen_base_velocity']<1e-9 for z in bth.values())
  custom=[v['custom']['median_solver_ms'] for v in bth.values() if v['custom'].get('ok')]
  upstream=[v['mink']['median_solver_ms'] for v in bth.values() if v['mink'].get('ok')]
  return {'schema':'cad30-mink-vs-custom-WBC-1',
          'status':'KINEMATIC_MATCHED_TASK_PASS' if passed else 'REVIEW_FAILED_CASES',
          'source':'FNK0052 CAD30 with four massless sites at original physical contacts; archived semi_tucked B t=9.3s',
          'versions':{'mink':version('mink'),'mujoco':mujoco.__version__,'numpy':version('numpy'),'scipy':version('scipy'),'daqp':version('daqp')},
          'geometry':{'model_meshes':int(model.nmesh),'qpos':int(model.nq),'nvel':int(model.nv),'contact_site_reconstruction_m':markers_error},
          'model_joint_limits':'XML ±pi rad; two pose-B joints exceed provisional research ±1.7 rad',
          'cases':bth,'summary':{'cases_total':len(CASES),'both_pass':sum(c['custom'].get('ok') and c['mink'].get('ok') for c in bth.values()),
                               'max_custom_drift_mm':max((c['custom'].get('max_foot_drift_mm',float('inf')) for c in bth.values())),
                               'max_mink_drift_mm':max((c['mink'].get('max_foot_drift_mm',float('inf')) for c in bth.values())),
                               'median_case_solver_custom_ms':float(np.median(custom)) if custom else None,
                               'median_case_solver_mink_ms':float(np.median(upstream)) if upstream else None,
                               'median_case_call_custom_ms':float(np.median([v['custom']['median_call_ms'] for v in bth.values() if v['custom'].get('ok')])),
                               'median_case_call_mink_ms':float(np.median([v['mink']['median_call_ms'] for v in bth.values() if v['mink'].get('ok')]))},
          'limitations':['Only one archived pose, 10 tiny goals; no physical simulation, no torque/contact stability, no hardware approval',
                         'Both use same fixed exact base increment and foot markers, with hinge delta limited ±0.02rad, front/head frozen',
                         'Two solvers use different residual objective/scaling and optimize from slightly different linearization points',
                         'Solver timing excludes per-run task setup; call time includes it but not Jacobian construction or contacts. Neither is a real-time guarantee',
                         'Results do not establish 4-leg gait, MPC, or superiority under full task distribution']}


def main():
  ap=argparse.ArgumentParser();ap.add_argument('--model',type=Path,required=True);ap.add_argument('--replay-zip',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);ap.add_argument('--repeats',type=int,default=20)
  a=ap.parse_args();r=benchmark(a.model,a.replay_zip,a.repeats);a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(r,indent=2,ensure_ascii=False)+'\n');print(json.dumps({'status':r['status'],**r['summary']},indent=2))
  if r['status']!='KINEMATIC_MATCHED_TASK_PASS':
    for name,v in r['cases'].items():
      if not(v['custom'].get('ok') and v['mink'].get('ok')):print(name,'custom',v['custom'].get('reason'),'mink',v['mink'].get('reason'))
    raise SystemExit(1)
if __name__=='__main__':main()