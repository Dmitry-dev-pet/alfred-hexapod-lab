#!/usr/bin/env python3
"""Turn 60 executed S2 Lite3 MuJoCo rollouts into compact reproducible metrics."""
import json, math, hashlib, statistics, os
from collections import defaultdict
from pathlib import Path
from lite3_s2_robustness import VARIATIONS
from lite3_onnx_joint_audit import ONNX, SCENE, sha256

RESULTS=Path(os.environ.get('LITE3_S2_RESULTS_DIR', './s2_final'))
SECONDS=9.5
N_SEEDS=10

def mean(xs):return sum(xs)/len(xs)

def round6(x):return round(float(x),6)

def main():
    rows=[];summary={};all_runs=[]
    for v in VARIATIONS:
        runs=[]
        for seed in range(20261106,20261106+N_SEEDS):
            p=RESULTS/(v.name+'_'+str(seed)+'.json')
            r=json.loads(p.read_text())['run'];runs.append(r)
            assert r['seed']==seed and r['variation']==v.name
            assert r['physics_contact_audit']['physics_steps']==2375
        all_runs.extend(runs)
        def metric(key):return [float(r[key]) for r in runs]
        def audit(key):return [float(r['physics_contact_audit'][key]) for r in runs]
        def local_xyz(s):
            theta=math.radians(s['yaw_start_deg']);dx=s['dx_m'];dy=s['dy_m'];return (dx*math.cos(theta)+dy*math.sin(theta),-dx*math.sin(theta)+dy*math.cos(theta))
        track=[]
        for rr in runs:
            seg=rr['segments']
            assert len(seg)==6
            forward=local_xyz(seg[1])[0]*1e3
            lateral=local_xyz(seg[2])[1]*1e3
            yaw=seg[3]['yaw_end_deg']-seg[3]['yaw_start_deg']
            reverse=local_xyz(seg[4])[0]*1e3
            stop=math.hypot(seg[5]['dx_m'],seg[5]['dy_m'])*1e3
            track.append((forward,lateral,yaw,reverse,stop))
        data={
            'label':v.name,'n':len(runs),'falls':sum(bool(r['first_fall']) for r in runs),
            'max_tilt_deg_max':round6(max(metric('max_tilt_deg'))),
            'min_root_height_m_min':round6(min(metric('min_root_height_m'))),
            'saturation_steps_avg':round6(mean(metric('torque_saturation_ticks'))),
            'fraction_sim_steps_any_actuator_at_force_cap_mean':round6(mean(metric('torque_saturation_ticks'))/2375),
            'contact_slip_speed_gt_0p05_m_s_mean_fraction':round6(mean(audit('fraction_loaded_contact_samples_speed_gt_005_m_s'))),
            'contact_slip_speed_gt_0p10_m_s_mean_fraction':round6(mean(audit('fraction_loaded_contact_samples_speed_gt_010_m_s'))),
            'loaded_contact_speed_mean_m_s':round6(mean(audit('mean_loaded_contact_tangential_speed_m_s'))),
            'peak_loaded_contact_speed_m_s_max':round6(max(audit('peak_loaded_contact_tangential_speed_m_s'))),
            'zero_loaded_feet_steps_mean_fraction':round6(mean(audit('fraction_steps_zero_loaded'))),
            'less_than_three_loaded_feet_steps_mean_fraction':round6(mean(audit('fraction_steps_less_3_loaded'))),
            'forward_mm_mean':round6(mean([t[0] for t in track])),
            'lateral_mm_mean':round6(mean([t[1] for t in track])),
            'yaw_deg_mean':round6(mean([t[2] for t in track])),
            'reverse_mm_mean':round6(mean([t[3] for t in track])),
            'stop_drift_mm_mean':round6(mean([t[4] for t in track])),
        }
        summary[v.name]=data
        print(v.name,data)
    assert len(all_runs)==60
    out={
        'study':'XGO-Lite3 S2 preliminary robustness screen',
        'date_utc':'2026-11-06',
        'status':'60_ACTUAL_CPU_MUJOCO_RUNS_INDEPENDENT_NOT_NATIVE_MJLAB_NOT_HARDWARE',
        'model_and_policy':{'onnx_sha256':sha256(ONNX),'original_scene_sha256':sha256(SCENE),'mass_kg':0.345,'nominal_product_mass_kg':0.610},
        'episodes':60,'seeds_per_case':10,'seed_start':20261106,'seed_end':20261115,
        'total_physics_steps':60*2375,'physics_step_seconds':0.004,'policy_frequency_hz':50,
        'duration_seconds_per_episode':SECONDS,
        'environment':'flat; four shank contacts only; no arm/body self-collision; ONNX naturally mapped 14 joints; no external disturbance',
        'fall_threshold':'tilt >60deg OR base z <0.03m; an incomplete safety metric',
        'contact_tangential_velocity':'computed from pre-integration qvel and actual cached MuJoCo contact Jacobian/normal and Fz, every 4ms; counting only loaded Fn≥0.2N contact samples; speed>0.05 m/s is a descriptive slipping indicator, not a validated real traction threshold',
        'model_mass_variant':'uniformly scale all inertial body masses and body inertias by 0.610/0.345; distribution uncalibrated',
        'motor_variant':'set model actuator force cap ±0.2256 N·m, a nominal approximate kg.cm conversion, not motor continuous torque',
        'conditions':summary,
        'known_uncertainties':['Only 10 seeds with small ±1deg roll/pitch ±2.29deg yaw initial variations','No terrain/obstacles/collision beyond four shank meshes','No battery, thermal, joint-speed, motor backlash, controller latency, encoder, IMU-noise calibration','Not native MJLab; policy.onnx differs from model_1499.pt checkpoint','Per-contact time samples correlated; percentages are descriptive and not statistically independent observations','No actual hardware performance can be inferred']
    }
    data=json.dumps(out,ensure_ascii=False,indent=2)+'\n'
    (RESULTS/'S2_60RUN_AGGREGATE.json').write_text(data)
    return out

if __name__=='__main__':main()