#!/usr/bin/env python3
"""Review-only CPU battery: matched XML PD, BAM M6, and BAM + backlash.

Uses pinned upstream PolicyInference and BAM without torch/warp or retraining.
The backlash adapter supplies the same encoder view as upstream training:
policy pos/vel = motor + passive; firmware P position = motor + passive;
motor velocity remains motor-side for back-EMF and friction.
"""
from __future__ import annotations

import argparse
import contextlib
import hashlib
import io
import json
import math
from pathlib import Path
import time

import mujoco
import numpy as np
import onnxruntime as ort
from scipy.spatial.transform import Rotation

from standup_cpu import (
    CASES, CONTROL_DT, DT, POLICY_SHA256, UPSTREAM_COMMIT,
    is_stable, sample, sha256, upstream_inference,
)

BAM_COMMIT = "62bd8ce12154340be97e06f7f41a0ca8f116d967"
MODES = ("xml_pd", "bam", "bam_backlash")
SEEDS = (20261109, 20261110, 20261111)
_SESSION = ort.InferenceSession


def cpu_session(path, *args, **kwargs):
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    return _SESSION(path, sess_options=options, providers=["CPUExecutionProvider"])


def source_gate(root: Path, bam_root: Path, manifest: dict) -> dict:
    """Verify every downloaded upstream/BAM blob, not just a user-supplied SHA label."""
    counts = {}
    for label, path in (("microduck_rl", root), ("bam", bam_root)):
        entries = manifest[label]["blobs"]
        for name, expected in entries.items():
            p = path / name
            if not p.is_file():
                raise ValueError(f"Missing source: {label}/{name}")
            b = p.read_bytes()
            actual = hashlib.sha1(f"blob {len(b)}\0".encode() + b).hexdigest()
            if actual != expected:
                raise ValueError(f"Unpinned source: {label}/{name}")
        counts[label] = len(entries)
    if manifest["microduck_rl"]["commit"] != UPSTREAM_COMMIT:
        raise ValueError("Wrong RL source commit")
    if manifest["bam"]["commit"] != BAM_COMMIT:
        raise ValueError("Wrong BAM source commit")
    # Ensure the installed BAM Python/parameter bytes are the validated source.
    import bam
    installed = Path(bam.__file__).parent
    for name in manifest["bam"]["blobs"]:
        if name.startswith("bam/") and (name.endswith(".py") or name.endswith(".json")):
            if (installed / name[4:]).read_bytes() != (bam_root / name).read_bytes():
                raise ValueError(f"Installed BAM differs: {name}")
    return {"verified_git_blobs": counts, "manifest_sha256": sha256(Path(manifest["_path"]))}


def backlash_indices(model):
    servo = model.actuator_trnid[:, 0]
    passive = []
    for jid in servo:
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(jid))
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"passive_{name}_backlash")
        if bid < 0:
            raise ValueError(f"Missing backlash partner for {name}")
        passive.append(bid)
    return model.jnt_qposadr[passive], model.jnt_dofadr[passive]


def install_encoder_view(policy, ctrl, model, data):
    """Adapt official CPU interfaces to training's output-encoder semantics."""
    bq, bv = backlash_indices(model)
    sq, sv = np.array(policy.joint_qpos_indices), np.array(policy.joint_qvel_indices)
    policy.get_joint_pos_relative = lambda: (
        (data.qpos[sq] + data.qpos[bq]).astype(np.float32) - policy.default_pose
    )
    policy.get_joint_vel = lambda: (data.qvel[sv] + data.qvel[bv]).astype(np.float32)
    original = ctrl.model.actuator.compute_control

    def encoder_control(target, q, dq, dt):
        # Only firmware position feedback is remapped; rotor dq is unchanged.
        return original(target, q + data.qpos[bq], dq, dt)

    ctrl.model.actuator.compute_control = encoder_control
    return bq, bv


def perturbation(case, seed):
    # Each mode gets identical draws regardless of actuator implementation.
    rng = np.random.default_rng(np.random.SeedSequence([seed, CASES.index(case)]))
    return {
        "joint_jitter_rad": rng.uniform(-0.02, 0.02, 14).tolist(),
        "root_rpy_jitter_rad": rng.uniform(-math.radians(10), math.radians(10), 3).tolist(),
        "root_xy_m": rng.uniform(-0.01, 0.01, 2).tolist(),
        "drop_height_m": float(rng.uniform(0.17, 0.19)),
        "push_velocity_mps": float(rng.uniform(0.6, 0.8)),
        "battery_voltage_v": float(rng.uniform(6.5, 8.2)),
        "voltage_sag_gain": float(rng.uniform(0.0, 0.2)),
    }


def contact_metrics(model, data, floor):
    total = np.zeros(3)
    peak = 0.0
    for j in range(data.ncon):
        c = data.contact[j]
        if floor not in (c.geom1, c.geom2):
            continue
        wrench = np.zeros(6)
        mujoco.mj_contactForce(model, data, j, wrench)
        # Force vector sign is irrelevant for the norm; accumulate in world axes.
        sign = 1 if c.geom1 == floor else -1
        total += sign * (np.asarray(c.frame).reshape(3, 3).T @ wrench[:3])
        peak = max(peak, float(np.linalg.norm(wrench[:3])))
    return float(np.linalg.norm(total)), peak


def run_episode(upstream, root, policy_path, mode, case, seed, seconds):
    p = perturbation(case, seed)
    scene_name = "scene_backlash.xml" if mode == "bam_backlash" else "scene.xml"
    scene = root / "src/mjlab_microduck/robot/microduck" / scene_name
    ctrl = None
    if mode == "xml_pd":
        model = mujoco.MjModel.from_xml_path(str(scene))
        model.opt.timestep = DT
        data = mujoco.MjData(model)
    else:
        b = upstream.load_bam_model(upstream.BAM_KP_FW, p["battery_voltage_v"], upstream.BAM_MAX_CURRENT)
        model, data, ctrl, _ = upstream.load_mujoco_with_bam(
            str(scene), b, DT, p["voltage_sag_gain"], upstream.BAM_VIN_MIN
        )
    key = case.upper() if case in ("sit", "fold") else "STAND"
    kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, key)
    if kid < 0:
        raise ValueError(f"Missing keyframe {key}")
    mujoco.mj_resetDataKeyframe(model, data, kid)
    fid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "trunk_base_freejoint")
    qa, va = int(model.jnt_qposadr[fid]), int(model.jnt_dofadr[fid])
    servo = model.actuator_trnid[:, 0]
    sq, sv = model.jnt_qposadr[servo], model.jnt_dofadr[servo]
    data.qpos[sq] += p["joint_jitter_rad"]
    data.qpos[qa:qa + 2] = p["root_xy_m"]
    bases = {"roll180": (math.pi, 0, 0), "pitch180": (0, math.pi, 0), "side90": (math.pi / 2, 0, 0)}
    r = Rotation.from_euler("xyz", bases.get(case, (0, 0, 0))) * Rotation.from_euler("xyz", p["root_rpy_jitter_rad"])
    q = r.as_quat()
    data.qpos[qa + 3:qa + 7] = q[[3, 0, 1, 2]]
    if case in bases:
        data.qpos[qa + 2] = p["drop_height_m"]
    data.qvel[:] = 0
    if ctrl is not None:
        ctrl.reset(data.qpos)
        ctrl.last_ts = float(data.time)
    else:
        data.ctrl[:] = data.qpos[sq]
    mujoco.mj_forward(model, data)
    policy = upstream.PolicyInference(model, data, walking_onnx_path=str(policy_path),
                                      bam_ctrl=ctrl, use_projected_gravity=True, new_cmd_obs=True)
    if mode == "bam_backlash":
        install_encoder_view(policy, ctrl, model, data)
        ctrl.q_target[:] = data.qpos[sq] + data.qpos[backlash_indices(model)[0]]
    policy.set_vel_cmd(0, 0, 0)
    if policy.get_observations().shape != (61,) or model.nu != 14:
        raise ValueError("Wrong 61D/14D policy contract")
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    peak_force = peak_single = torque_peak = 0.0
    torque_sq_dt = 0.0
    absolute_work = 0.0
    saturation_steps = 0
    physical_steps = 0
    reset_detected = False
    frames = []
    settling_frames = []
    targets = []

    def physics_step():
        nonlocal peak_force, peak_single, torque_peak, torque_sq_dt, absolute_work
        nonlocal saturation_steps, physical_steps, reset_detected
        if ctrl is not None:
            ctrl.update()
        before = float(data.time)
        mujoco.mj_step(model, data)
        physical_steps += 1
        reset_detected |= abs(float(data.time) - before - DT) > 1e-8
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise FloatingPointError("Nonfinite physics state")
        force, single = contact_metrics(model, data, floor)
        peak_force, peak_single = max(peak_force, force), max(peak_single, single)
        tau = np.asarray(data.actuator_force)
        torque_peak = max(torque_peak, float(np.max(np.abs(tau))))
        torque_sq_dt += float(tau @ tau) * DT
        absolute_work += float(np.sum(np.abs(tau * data.qvel[sv]))) * DT
        limits = model.actuator_forcerange[:, 1]
        saturation_steps += int(np.any(np.abs(tau) >= .99 * limits))

    # All variants use the same 0.6s motor-held settling recipe for synthetic drops.
    if case in bases:
        for k in range(round(.6 / DT)):
            physics_step()
            if k % 4 == 3:
                mujoco.mj_forward(model, data)
                settling_frames.append({"t_physics":float(data.time), **sample(model,data,qa,va)})
    start = data.qpos[qa:qa+3].copy()
    initial = {"qpos":data.qpos.tolist(),"qvel":data.qvel.tolist(),"t_physics":float(data.time)}
    start_t = float(data.time)
    for k in range(round(seconds / CONTROL_DT)):
        if case == "push" and k == round(1 / CONTROL_DT):
            data.qvel[va] = p["push_velocity_mps"]
        mujoco.mj_forward(model, data)
        action = policy.infer()
        policy.apply_action(action)
        targets.append((ctrl.q_target if ctrl is not None else data.ctrl).tolist())
        for _ in range(round(CONTROL_DT / DT)):
            physics_step()
        mujoco.mj_forward(model, data)
        frames.append({"t":round(float(data.time)-start_t,6), **sample(model,data,qa,va)})
    eligible = [f for f in frames if f["t"] > (1.0 if case == "push" else 0.0)]
    window = round(1 / CONTROL_DT)
    first = next((eligible[i]["t"] - (1 if case == "push" else 0)
                  for i in range(window-1,len(eligible))
                  if all(map(is_stable,eligible[i-window+1:i+1]))),None)
    warnings = {str(i):int(s.number) for i,s in enumerate(data.warning) if s.number}
    stable = len(eligible)>=window and all(map(is_stable,eligible[-window:]))
    passed = stable and not warnings and not reset_detected
    xy = np.array([f["qpos"][qa:qa+2] for f in frames])
    summary = {
        "mode":mode,"case":case,"seed":seed,"status":"PASS" if passed else "FAIL",
        "stable_at_end_1s":stable,"warnings":warnings,"physics_reset_detected":reset_detected,
        "physical_steps":physical_steps,"time_to_first_1s_stable_s":first,
        "final_displacement_m":float(np.linalg.norm(xy[-1]-start[:2])),
        "max_displacement_m":float(np.max(np.linalg.norm(xy-start[:2],axis=1))),
        "path_length_m":float(np.linalg.norm(np.diff(np.vstack([start[:2],xy]),axis=0),axis=1).sum()),
        "peak_floor_resultant_force_n":peak_force,"peak_single_contact_force_n":peak_single,
        "peak_actuator_torque_nm":torque_peak,"sum_torque_squared_dt_nm2_s":torque_sq_dt,
        "sum_abs_mechanical_work_j":absolute_work,"saturation_step_fraction":saturation_steps/physical_steps,
        "final_z_m":frames[-1]["trunk_z_m"],"final_tilt_deg":frames[-1]["tilt_deg"],
        "nq":model.nq,"nv":model.nv,"nu":model.nu,
    }
    trace = {**summary,"perturbation":p,"initial_after_settle":initial,
             "settling_frames":settling_frames,"frames":frames,"position_targets":targets}
    return summary, trace


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--rl-root",type=Path,required=True)
    parser.add_argument("--bam-root",type=Path,required=True)
    parser.add_argument("--source-manifest",type=Path,required=True)
    parser.add_argument("--policy",type=Path,required=True)
    parser.add_argument("--output-dir",type=Path,required=True)
    parser.add_argument("--seconds",type=float,default=6.0)
    parser.add_argument("--modes",nargs="+",choices=MODES,default=list(MODES))
    parser.add_argument("--cases",nargs="+",choices=CASES,default=list(CASES))
    parser.add_argument("--seeds",type=int,nargs="+",default=list(SEEDS))
    args=parser.parse_args()
    if not 2<=args.seconds<=30:
        parser.error("seconds must be in [2,30]")
    if sha256(args.policy)!=POLICY_SHA256:
        raise ValueError("Unpinned ONNX")
    manifest=json.loads(args.source_manifest.read_text());manifest["_path"]=str(args.source_manifest)
    verified=source_gate(args.rl_root,args.bam_root,manifest)
    upstream=upstream_inference(args.rl_root)
    upstream.ort.InferenceSession=cpu_session
    args.output_dir.mkdir(parents=True,exist_ok=True)
    summary={"schema":"microduck-bam-backlash-cpu-v2","publication_policy":"review_only",
             "rl_commit":UPSTREAM_COMMIT,"bam_commit":BAM_COMMIT,"policy_sha256":POLICY_SHA256,
             "mujoco_version":mujoco.__version__,"onnxruntime_version":ort.__version__,
             "numpy_version":np.__version__,"source_verification":verified,
             "physics_dt_s":DT,"policy_dt_s":CONTROL_DT,"seconds_per_case":args.seconds,
             "seeds":args.seeds,"episodes":[]}
    began=time.monotonic()
    for mode in args.modes:
        for case in args.cases:
            for seed in args.seeds:
                try:
                    with contextlib.redirect_stdout(io.StringIO()):
                        metrics,trace=run_episode(upstream,args.rl_root,args.policy,mode,case,seed,args.seconds)
                except Exception as exc:
                    # A failed episode remains an explicit outcome; it never
                    # silently reduces the denominator or becomes PASS.
                    metrics={"mode":mode,"case":case,"seed":seed,"status":"ERROR",
                             "error_type":type(exc).__name__,"error":str(exc)}
                    trace={**metrics,"perturbation":perturbation(case,seed),"frames":[]}
                name=f"{mode}-{case}-{seed}.json"
                path=args.output_dir/name
                path.write_text(json.dumps(trace,separators=(",",":"),allow_nan=False)+"\n")
                summary["episodes"].append({**metrics,"trace_file":name,"trace_sha256":sha256(path)})
                if metrics['status']=='ERROR':
                    print(f"{mode:13} {case:9} {seed}: ERROR {metrics['error']}",flush=True)
                else:
                    print(f"{mode:13} {case:9} {seed}: {metrics['status']} drift={metrics['final_displacement_m']:.3f}m peak={metrics['peak_floor_resultant_force_n']:.1f}N",flush=True)
                (args.output_dir/"summary.json").write_text(json.dumps(summary,indent=2,allow_nan=False)+"\n")
    summary["wall_seconds"]=time.monotonic()-began
    (args.output_dir/"summary.json").write_text(json.dumps(summary,indent=2,allow_nan=False)+"\n")
    print(f"Finished {len(summary['episodes'])} episodes in {summary['wall_seconds']:.1f}s")


if __name__=="__main__":
    main()
