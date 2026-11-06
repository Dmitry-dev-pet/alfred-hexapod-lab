#!/usr/bin/env python3
"""Independent S2 perturbation screen over pinned XGO-Lite3 ONNX/MuJoCo CPU.

Uses original `lite3_onnx_joint_audit.py` observation, command, mapping,
controller and integrator without replacing the actor or editing vendor assets.
Overrides only model mass/inertias, friction, and actuator force limits *before*
the physical rollout. Observes true contact forces and the relative tangential
speed at each physical 4ms step, with no simulated sensor/actuator calibration.
Not native MJLab, not an inferred hardware mass distribution, not robot control.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import time
from dataclasses import dataclass
from typing import Optional
import numpy as np
import mujoco
import lite3_onnx_joint_audit as base

@dataclass(frozen=True)
class Variation:
    name: str
    friction: Optional[float] = None
    mass_scale: float = 1.0
    force_cap: Optional[float] = None
    rationale: str = ''

VARIATIONS = [
    Variation('original_baseline', rationale='No edits to source model (floor mu=1.0, shank mu=0.6, 0.345 kg, model cap 0.31)'),
    Variation('friction_035', friction=0.35, rationale='Both floor and four shanks set mu=0.35'),
    Variation('friction_020', friction=0.20, rationale='Both floor and four shanks set mu=0.20'),
    Variation('uniform_mass_0610', mass_scale=0.610/0.345, rationale='All body mass+inertia multiplied uniformly: NOT measured real distribution'),
    Variation('actuator_cap_02256', force_cap=0.2256, rationale='Model torque cap reduced to nominal manufacturer kg.cm equivalent; NOT continuous servo calibration'),
    Variation('combined_stress', friction=0.35, mass_scale=0.610/0.345, force_cap=0.2256, rationale='Three simultaneous assumptions; not physical robot validation'),
]


def modify_model(m, variation: Variation):
    if variation.friction is not None:
        for gid in range(m.ngeom):
            n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, gid)
            if n == 'floor' or (n and n.endswith('_shank_collision')):
                m.geom_friction[gid,0] = variation.friction
    if variation.mass_scale != 1.0:
        m.body_mass[:] *= variation.mass_scale
        m.body_inertia[:] *= variation.mass_scale
        mujoco.mj_setConst(m, mujoco.MjData(m))
    if variation.force_cap is not None:
        assert np.all(m.actuator_forcelimited == 1)
        m.actuator_forcerange[:,0] = -variation.force_cap
        m.actuator_forcerange[:,1] = variation.force_cap
    return m

class StepAudit:
    def __init__(self):
        self.n = 0
        self.steps_no_foot_contact = 0
        self.steps_lt2_loaded = 0
        self.steps_lt3_loaded = 0
        self.steps_all_four_loaded = 0
        self.contact_velocity_samples = 0
        self.samples_slip_gt_005ms = 0
        self.samples_slip_gt_010ms = 0
        self.max_contact_slip_ms = 0.0
        self.sum_contact_slip_ms = 0.0
        self.max_normal_force_N = 0.0
        self.min_foot_support_count = 4
        self.numerically_unstable = False

    def observe(self,m,d,pre_step_qvel):
        self.n += 1
        loads = {f:0. for f in base.FEET}
        for ci in range(d.ncon):
            c=d.contact[ci]
            pairs=(int(c.geom1),int(c.geom2))
            foot_geo = -1
            for gid in pairs:
                n=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,gid)
                if n in {f+'_shank_collision' for f in base.FEET}:
                    foot_geo=gid
                    foot=n[:2]
                    break
            if foot_geo < 0:continue
            force=np.zeros(6)
            mujoco.mj_contactForce(m,d,ci,force)
            fn=max(0.,float(force[0]))
            loads[foot] += fn
            self.max_normal_force_N=max(self.max_normal_force_N,fn)
            if fn >= 0.2:
                body_id=int(m.geom_bodyid[foot_geo])
                jacp=np.zeros((3,m.nv));jacr=np.zeros((3,m.nv))
                mujoco.mj_jac(m,d,jacp,jacr,c.pos,body_id)
                velocity=jacp@pre_step_qvel
                normal=np.asarray(c.frame[:3])
                tangent=velocity-np.dot(velocity,normal)*normal
                vt=float(np.linalg.norm(tangent))
                self.contact_velocity_samples+=1
                self.max_contact_slip_ms=max(self.max_contact_slip_ms,vt)
                self.sum_contact_slip_ms+=vt
                self.samples_slip_gt_005ms+=int(vt>0.05)
                self.samples_slip_gt_010ms+=int(vt>0.10)
        loaded=sum(v>=0.2 for v in loads.values())
        self.steps_no_foot_contact+=int(loaded==0)
        self.steps_lt2_loaded+=int(loaded<2)
        self.steps_lt3_loaded+=int(loaded<3)
        self.steps_all_four_loaded+=int(loaded==4)
        self.min_foot_support_count=min(loaded,self.min_foot_support_count)
        self.numerically_unstable|=not bool(np.isfinite(d.qpos).all())

    def as_dict(self):
        n=max(self.n,1);ns=max(self.contact_velocity_samples,1)
        return {
            'physics_steps':self.n, 'min_loaded_feet_per_physics_step':self.min_foot_support_count,
            'fraction_steps_zero_loaded':self.steps_no_foot_contact/n,
            'fraction_steps_less_2_loaded':self.steps_lt2_loaded/n,
            'fraction_steps_less_3_loaded':self.steps_lt3_loaded/n,
            'fraction_steps_four_loaded':self.steps_all_four_loaded/n,
            'loaded_contact_samples':self.contact_velocity_samples,
            'peak_loaded_contact_tangential_speed_m_s':self.max_contact_slip_ms,
            'mean_loaded_contact_tangential_speed_m_s':self.sum_contact_slip_ms/ns,
            'fraction_loaded_contact_samples_speed_gt_005_m_s':self.samples_slip_gt_005ms/ns,
            'fraction_loaded_contact_samples_speed_gt_010_m_s':self.samples_slip_gt_010ms/ns,
            'max_contact_normal_force_N':self.max_normal_force_N,
            'numerically_unstable':self.numerically_unstable,
        }


def run_perturbed(session,joints,seed,variation,protocol='midstride'):
    audit=StepAudit()
    original_get_model=base.get_model
    original_mj_step=mujoco.mj_step
    def get_perturbed_model():return modify_model(original_get_model(),variation)
    def observe_step(m,d,*a,**kw):
        # MuJoCo contact/force/jacobian caches correspond to the pre-integration
        # configuration while qvel has advanced on return from mj_step.
        pre_step_qvel=d.qvel.copy()
        original_mj_step(m,d,*a,**kw)
        audit.observe(m,d,pre_step_qvel)
    base.get_model=get_perturbed_model
    mujoco.mj_step=observe_step
    try:
        run=base.run_case(session,joints,'onnx',seed,9.5 if protocol=='midstride' else 8.,protocol=protocol)
    finally:
        base.get_model=original_get_model
        mujoco.mj_step=original_mj_step
    data={k:v for k,v in run.items() if k!='trace'}
    data['variation']=variation.name
    data['physics_contact_audit']=audit.as_dict()
    data['observed_protocol']='Every 4ms: loaded-contact Fz, cached pre-integration contact frames/Jacobian and matching pre-integration qvel; 50Hz ONNX closed loop'
    return data,run['trace']


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--seeds',type=int,default=2)
    p.add_argument('--start-seed',type=int,default=20261106)
    p.add_argument('--variations',nargs='+',choices=[v.name for v in VARIATIONS],default=[v.name for v in VARIATIONS])
    p.add_argument('--output-dir',type=Path,default=Path('/tmp/lite3_s2'))
    a=p.parse_args()
    a.output_dir.mkdir(parents=True,exist_ok=True)
    policy,joints,meta=base.make_policy()
    chosen=[x for x in VARIATIONS if x.name in a.variations]
    print('S2 original policy:',base.sha256(base.ONNX),'variations:',len(chosen),'seeds:',a.seeds,flush=True)
    report={'name':'Lite3 S2 ONNX independent CPU robustness screen','version':'S2_PRELIM_2026-11-06',
            'main_untouched':True,'native_mjlab':False,'real_robot_test':False,
            'source':'research/xgo_lite3/lite3_onnx_joint_audit.py from draft PR #69',
            'baseline_policy_sha256':base.sha256(base.ONNX),'baseline_model_xml':base.sha256(base.SCENE),
            'engine_versions':{'mujoco':mujoco.__version__,'onnxruntime':base.ort.__version__},
            'joint_names':joints,'assumptions':[v.__dict__ for v in chosen],
            'protocol':'midstride 9.5 seconds; same seed perturbations and ONNX actions; 250Hz physical integration / 50Hz ONNX; original flat contact configuration; body mass+inertia scaled uniformly in 610g variants; not calibrated',
            'runs':[]}
    for variant in chosen:
        for seed in range(a.start_seed,a.start_seed+a.seeds):
            try:
                data,trace=run_perturbed(policy,joints,seed,variant)
                outfile=a.output_dir/(variant.name+'_'+str(seed)+'.json')
                outfile.write_text(json.dumps({'run':data,'trace':trace},separators=(',',':'))+'\n')
                report['runs'].append(data)
                print(variant.name,seed,'tilt',round(data['max_tilt_deg'],2),'fall',bool(data['first_fall']),
                      'sat',data['torque_saturation_ticks'], 'slip_gt0.05',round(data['physics_contact_audit']['fraction_loaded_contact_samples_speed_gt_005_m_s'],4),flush=True)
            except Exception as e:
                report['runs'].append({'variation':variant.name,'seed':seed,'error':str(e)})
                print('FAILED',variant.name,seed,str(e),flush=True)
    (a.output_dir/'S2_PRELIM_SUMMARY.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Saved',a.output_dir)

if __name__=='__main__':main()