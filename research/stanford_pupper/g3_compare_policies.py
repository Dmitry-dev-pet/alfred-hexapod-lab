#!/usr/bin/env python3
"""G3: three-policy, matched-command, measured MuJoCo CPU physical evaluation.

Sources: existing independently reproduced Stanford Pupper v3 MuJoCo physics
and deploy inference in this research branch (PRs #73-#75).
Candidates: original pretrained policy, independent G2 exported native RSL-RL
policy, and a true zero-action correction (same neutral PD stance).

All episodes begin in identical joint/base states; no root scripting during
the rollout. This IS NOT native MJLab/C++ numerical parity and not hardware
qualification. G2 policy doesn't contain jump/hop or lift references, so
G3 excludes tricks, high-speed running and any claims about those functions.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
from full_controls import rollout
from infer_pupper import load_policy, POLICY_SHA256 as ORIGINAL_SHA

G2_POLICY_SHA = "228c70944c26e697cb93f42325386a47d20713699c22fb8bc24ca9fb9728562c"
PINNED_MODEL_BLOB = "7771e176c42b3aed1d48cc62434771b0fceb7c8c"

SCENARIOS = [
    ("stand", 7.0, [{"t":0},{"t":1},{"t":6}]),
    ("forward", 9.0, [{"t":0},{"t":1,"vx":0.20},{"t":8}]),
    ("reverse", 9.0, [{"t":0},{"t":1,"vx":-0.12},{"t":8}]),
    ("strafe", 9.0, [{"t":0},{"t":1,"vy":0.12},{"t":8}]),
    ("turn", 9.0, [{"t":0},{"t":1,"wz":0.20},{"t":8}]),
    ("diagonal", 9.0, [{"t":0},{"t":1,"vx":0.14,"vy":0.08},{"t":8}]),
    ("switch", 12.0, [
        {"t":0},{"t":1,"vx":0.20},{"t":3,"vy":0.12},
        {"t":5,"wz":0.20},{"t":7,"vx":-0.12},
        {"t":9,"vx":0.14,"vy":-0.08,"wz":-0.15},{"t":11},
    ]),
]
NATIVE_EXTRA = {"forward","reverse","switch"}
POLICIES = ("zero","stanford","g2")
PHYSICS_DT = 0.004
FOOT_LOAD_THRESHOLD_N = 0.2


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def policy_weights(policy: dict[str,Any]) -> list[tuple[np.ndarray,np.ndarray,str]]:
    assert policy["in_shape"] == [None,960]
    dim=960
    result=[]
    for item in policy["layers"]:
        assert item["type"]=="dense"
        weights=np.array(item["weights"][0],dtype=np.float32)
        biases=np.array(item["weights"][1],dtype=np.float32)
        assert item["shape"]==[None,biases.size] and weights.shape==(dim,biases.size)
        assert np.isfinite(weights).all() and np.isfinite(biases).all()
        result.append((weights,biases,item["activation"]))
        dim=biases.size
    assert dim==12 and len(result)==5
    return result


def verify_policy_contract(orig: dict,g2: dict) -> dict:
    shared=["contract_version","use_imu","control_orientation",
            "observation_history","single_observation_size","obs_spec",
            "action_scale","kp","kd","default_joint_pos",
            "joint_upper_limits","joint_lower_limits","in_shape"]
    assert all(orig[k]==g2[k] for k in shared), "Mismatch in common observation/servo contract"
    assert orig["contract_version"]==2
    assert orig["observation_history"]==20 and orig["single_observation_size"]==48
    assert g2["obs_spec"]["frame_order"]=="newest_first"
    assert orig["obs_spec"]["components"]==[
        {"name":"base_ang_vel","size":3},
        {"name":"projected_gravity","size":3},
        {"name":"command","size":3},
        {"name":"desired_world_z","size":3},
        {"name":"joint_pos_rel","size":12},
        {"name":"last_action","size":12},
        {"name":"gait_reference","size":12},
    ]
    assert orig["kp"]==5.5 and orig["kd"]==0.25 and orig["action_scale"]==0.75
    assert orig["gait_reference"]["joint_names"]==g2["gait_reference"]["joint_names"]
    assert "jump_slot" in orig and "jump_slot" not in g2
    assert "lift_table" in orig["gait_reference"]
    assert "lift_table" not in g2["gait_reference"]
    assert orig["heading_hold"]!=g2["heading_hold"]
    assert orig["command_clip"]!=g2["command_clip"]
    # For experimental controlled mode all commands stay within both export
    # contracts and an additional Stanford deploy walk-speed cap.
    for _,_,schedule in SCENARIOS:
        for s in schedule:
            assert not s.get("jump",False) and not s.get("run",False)
            assert abs(s.get("vx",0.0))<=0.20
            assert abs(s.get("vy",0.0))<=0.12
            assert abs(s.get("wz",0.0))<=0.20
    return {
        "shared_keys_validated":shared,
        "g2_has_jump_slot":False,
        "g2_has_lift_reference":False,
        "native_heading_hold_differs":True,
        "native_command_clip_differs":True,
        "gallop_speed_orig":orig["gait_reference"]["gallop_speed"],
        "gallop_speed_g2":g2["gait_reference"]["gallop_speed"],
        "g2_compatibility_bridge":
            "Only copy two nonbinding walk/run velocity caps for the existing "
            "non-jump evaluator; trained G2 gait/observations/weights are unchanged",
    }


def calculate_metrics(frames: list[dict], duration: float) -> dict[str,Any]:
    errors_xy=[]
    errors_yaw=[]
    commanded=[]
    measured=[]
    feet_counter={"0":0,"1":0,"2":0,"3":0,"4":0}
    elapsed=0.0
    for i,frame in enumerate(frames):
        feet_counter[str(frame["feet_loaded"])]+=1
        if i==0:
            continue
        prev=frames[i-1]
        t=float(frame["t"])
        dt=t-float(prev["t"])
        assert 0.001<dt<0.03, (i,t,dt)
        q0=np.array(prev["qpos"],dtype=np.float64)
        q1=np.array(frame["qpos"],dtype=np.float64)
        assert np.isfinite(q0).all() and np.isfinite(q1).all()
        yaw0=math.atan2(2*(q0[3]*q0[6]+q0[4]*q0[5]),1-2*(q0[5]**2+q0[6]**2))
        yaw1=math.atan2(2*(q1[3]*q1[6]+q1[4]*q1[5]),1-2*(q1[5]**2+q1[6]**2))
        wx,wy=(q1[:2]-q0[:2])/dt
        c,s=math.cos(yaw0),math.sin(yaw0)
        vx=c*wx+s*wy
        vy=-s*wx+c*wy
        dyaw=((yaw1-yaw0+math.pi)%(2*math.pi)-math.pi)/dt
        cmd=np.array(frame["cmd"],dtype=np.float64)
        if t>=1.04 and t<=duration-0.98:
            errors_xy.extend([(vx-cmd[0])**2,(vy-cmd[1])**2])
            errors_yaw.append((dyaw-cmd[2])**2)
            commanded.append(cmd.tolist())
            measured.append([float(vx),float(vy),float(dyaw)])
            elapsed+=dt
    assert errors_xy and errors_yaw
    return {
        "tracking_xy_rmse_mps":round(math.sqrt(sum(errors_xy)/len(errors_xy)),6),
        "tracking_yaw_rmse_rps":round(math.sqrt(sum(errors_yaw)/len(errors_yaw)),6),
        "tracking_window_seconds":round(elapsed,5),
        "tracking_samples":len(errors_yaw),
        "loaded_feet_histogram":feet_counter,
        "loaded_2plus_fraction":round(sum(feet_counter[str(i)] for i in (2,3,4))/len(frames),6),
        "contact_threshold_N":FOOT_LOAD_THRESHOLD_N,
    }


def run(output: Path,original_path: Path,g2_path: Path) -> int:
    if output.exists():
        raise FileExistsError("Refusing to overwrite existing G3 output")
    original_bytes=original_path.read_bytes()
    g2_bytes=g2_path.read_bytes()
    assert sha256(original_bytes)==ORIGINAL_SHA, "Wrong pinned Stanford policy"
    assert sha256(g2_bytes)==G2_POLICY_SHA, "Wrong exact G2 extended exported policy"
    orig,original_weights=load_policy(original_path)
    g2=json.loads(g2_bytes)
    contract=verify_policy_contract(orig,g2)
    g2_weights=policy_weights(g2)
    # No trick requested. Existing PR #75's controller only reads the velocity
    # caps from jump_slot for flat non-jump commands: supply the same caps to
    # avoid silently claiming G2 can jump or run.
    g2_runtime=dict(g2)
    g2_runtime["jump_slot"]={
        "walk_speed_cap":orig["jump_slot"]["walk_speed_cap"],
        "run_speed_cap":orig["jump_slot"]["run_speed_cap"],
    }
    assert all(abs(float(x.get('vx',0)))<g2_runtime["jump_slot"]["walk_speed_cap"]
               for _,_,sch in SCENARIOS for x in sch)
    assert all(abs(float(x.get('vy',0)))<g2_runtime["jump_slot"]["walk_speed_cap"]
               for _,_,sch in SCENARIOS for x in sch)
    zero_weights=list(original_weights)
    w,b,activation=zero_weights[-1]
    zero_weights[-1]=(np.zeros_like(w),np.zeros_like(b),activation)
    assignments={
        "zero":(orig,zero_weights),
        "stanford":(orig,original_weights),
        "g2":(g2_runtime,g2_weights),
    }

    output.mkdir(parents=True,exist_ok=False)
    (output/"traces").mkdir()
    index={
        "schema":"alfred-pupper-g3-physical-ab-v1",
        "physics":"MuJoCo 3.14 CPU, original Pupper inertias/foot contacts, 13 visual meshes omitted",
        "simulator_version":mujoco.__version__,
        "numpy_version":np.__version__,
        "model_blob_sha1":PINNED_MODEL_BLOB,
        "original_policy_sha256":ORIGINAL_SHA,
        "g2_policy_sha256":G2_POLICY_SHA,
        "zero_baseline":"12 zero neural action corrections; same default pose/PD servo physics",
        "comparison":"matched schedules, flat ground, no jump, no run, no teleport",
        "controlled_mode":"heading_hold disabled, same emitted vx/vy/wz",
        "native_mode":"policy-specific heading_hold enabled, different effective commanded yaw",
        "policy_bridge":contract,
        "timestep_s":PHYSICS_DT,
        "policy_hz":50,
        "servo_delay_physics_steps":4,
        "fall_guard":"tilt>45deg OR torso_height<0.05m; not a full hardware safety criterion",
        "fixed_initial_state":True,
        "randomized_trials":False,
        "results":[],
        "status":"IN_PROGRESS",
    }
    try:
        for mode in ("controlled","native"):
            for name,duration,schedule in SCENARIOS:
                if mode=="native" and name not in NATIVE_EXTRA:
                    continue
                emitted_signatures=[]
                for kind in POLICIES:
                    policy,weights=assignments[kind]
                    start=time.perf_counter()
                    data=rollout(policy,weights,schedule,duration,
                                 hold_enabled=mode=="native",delay=4,frameskip=5)
                    data["policy_sha256"]= ORIGINAL_SHA if kind!="g2" else G2_POLICY_SHA
                    data["source"] = "Pinned Stanford source-MuJoCo independent deploy reconstruction"
                    data["policy_label"]=kind
                    assert data["summary"]["complete"],f"{kind}/{name}: incomplete physics"
                    assert data["summary"]["executed_steps"]==int(round(duration/PHYSICS_DT))
                    assert len(data["frames"])>50
                    assert data["summary"]["jump_scheduled_count"]==0
                    metrics=calculate_metrics(data["frames"],duration)
                    trace=f"traces/{mode}_{name}_{kind}.json"
                    raw=(json.dumps(data,separators=(",",":"),ensure_ascii=False)+"\n").encode()
                    (output/trace).write_bytes(raw)
                    result={
                        "mode":mode,"scenario":name,"policy":kind,
                        "duration_s":duration,"trace":trace,
                        "trace_sha256":sha256(raw),"wall_seconds":round(time.perf_counter()-start,4),
                        **data["summary"],**metrics,
                    }
                    index["results"].append(result)
                    # Compare exact applied commands for the controlled mode.
                    if mode=="controlled":
                        emitted_signatures.append(json.dumps(
                            [(row["t"],row["emitted"]) for row in data["policy_commands"]],
                            sort_keys=True,separators=(",",":")))
                    print("G3_CASE",mode,name,kind,"guard",result["fall_guard_steps"],
                          "dx",result["world_dx_m"],"yaw",result["net_yaw_deg"],
                          "xy_rmse",result["tracking_xy_rmse_mps"],flush=True)
                if mode=="controlled":
                    assert len(set(emitted_signatures))==1,(
                        "Controlled command mismatch",name)
        assert len(index["results"])==3*(len(SCENARIOS)+len(NATIVE_EXTRA))
        index["status"]="PHYSICAL_EVALUATION_COMPLETE"
    except Exception as ex:
        index["status"]="FAILED_OR_INCOMPLETE"
        index["error_type"]=type(ex).__name__
        # Avoid serializing arbitrary exception content from third-party code.
        print("G3_FAIL",type(ex).__name__,flush=True)
    finally:
        summary=index["results"]
        index["policy_overview"]={
            p:{
                "runs":len([r for r in summary if r["policy"]==p]),
                "falling_runs":sum(r["fall_guard_steps"]>0 for r in summary if r["policy"]==p),
                "controlled_falling":sum(r["fall_guard_steps"]>0 for r in summary if r["policy"]==p and r["mode"]=="controlled"),
            } for p in POLICIES
        }
        (output/"index.json").write_text(json.dumps(index,ensure_ascii=False,indent=2)+"\n")
        print("G3_STATUS",index["status"],"runs",len(summary),flush=True)
    return 0 if index["status"]=="PHYSICAL_EVALUATION_COMPLETE" else 2


if __name__=="__main__":
    ap=argparse.ArgumentParser()
    ap.add_argument("--original-policy",type=Path,required=True)
    ap.add_argument("--g2-policy",type=Path,required=True)
    ap.add_argument("--output",type=Path,required=True)
    args=ap.parse_args()
    sys.exit(run(args.output,args.original_policy,args.g2_policy))
