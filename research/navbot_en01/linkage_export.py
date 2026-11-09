"""Validate measured qpos/body poses, then adapt to the existing replay pipeline."""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
from .linkage_model import create
from .linkage import PINNED_BLOB

FORMAT="navbot-en01-fourbar-physical-replay-v2"
CAD_PATH="hexapod/en01-original-cad-v1/original-meshes.json"


def part_groups():
    groups={i:"torso" for i in range(73)}
    for side,upper,calf,wheel,rod in [
        ("right",[22,24,25,31,33,34],[18,37,48,58,62,69],[19,65],[28]),
        ("left",[23,26,27,30,32,35],[20,39,50,57,63,71],[21,64],[29])]:
        for ids,body in [(upper,"upper"),(calf,"calf"),(wheel,"wheel"),(rod,"rod")]:
            for i in ids:groups[i]=side+"_"+body
    return groups


def build(src,dst):
    import mujoco
    import numpy as np
    raw=Path(src).read_bytes();record=json.loads(raw);s=record["summary"]
    if (s["schema"]!="en01-fourbar-diagnostic/v1" or s["mujoco_version"]!="3.14.0"
            or s["geometry_source_blob"]!=PINNED_BLOB or s["duration_s"]!=6):
        raise ValueError("Unexpected trace provenance")
    xml=create(timestep=s["timestep_s"])
    if hashlib.sha256(xml.encode()).hexdigest()!=s["model_sha256"]:
        raise ValueError("Trace model hash mismatch")
    m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    names=record["body_names"]
    expected=[mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_BODY,b) for b in range(1,m.nbody)]
    if names!=expected:raise ValueError("Unknown physical body layout")
    rest=[[float(x) for x in d.xpos[b]-d.xpos[1]] for b in range(1,m.nbody)]
    frames=[];max_residual=0.;previous=-1.
    for f in record["frames"]:
        q=np.asarray(f["qpos"],dtype=float);v=np.asarray(f["qvel"],dtype=float)
        if q.shape!=(m.nq,) or v.shape!=(m.nv,) or not np.isfinite(q).all() or not np.isfinite(v).all():
            raise ValueError("Invalid recorded state")
        if not previous<f["t"] or not math.isfinite(f["t"]):raise ValueError("Non-monotone trace")
        previous=f["t"];d.qpos[:]=q;d.qvel[:]=v;mujoco.mj_forward(m,d)
        poses=np.asarray(f["body_poses"],dtype=float)
        rebuilt=np.concatenate([d.xpos[1:],d.xquat[1:]],axis=1)
        if poses.shape!=rebuilt.shape or not np.isfinite(poses).all():raise ValueError("Invalid body poses")
        residual=float(np.max(np.abs(poses-rebuilt)));max_residual=max(max_residual,residual)
        if residual>1e-10:raise ValueError("Visual poses disagree with measured qpos")
        if type(f["ground_contact"]) is not bool or type(f["nonwheel_ground_contact"]) is not bool:
            raise ValueError("Invalid contact flags")
        tail=[int(f["ground_contact"]),int(f["nonwheel_ground_contact"]),f["ground_normal_force_n"],f["pitch_rad"],f["roll_rad"],f["closure_error_m"]]
        if not all(math.isfinite(v) for v in tail):raise ValueError("Nonfinite telemetry")
        frames.append([f["t"],*poses.flatten().tolist(),*tail])
    if len(frames)!=300 or abs(frames[-1][0]-5.981)>1e-8:
        raise ValueError("Incomplete six-second physical trace")
    status="failed" if s["mode"] in ("jump","fall") or not s["upright_6s"] else "research"
    doc={"schema":FORMAT,"format":FORMAT,"physics_engine":"MuJoCo","physics_version":"3.14.0",
         "duration_s":6,"status":status,"body_names":names,"rest_body_positions":rest,
         "part_body_map":{str(k):v for k,v in part_groups().items()},"source":{
             "trace_sha256":hashlib.sha256(raw).hexdigest(),"model_sha256":s["model_sha256"],
             "step_git_blob_sha1":PINNED_BLOB},"result":s,"frame_count":len(frames),
         "sample_stride":len(frames[0]),"frames":frames,
         "limitations":["CAD-derived planar four-bar; lateral load planes simplified",
             "Unmeasured mass distribution, springs omitted, assumed motor curves and servo zero",
             "Original CAD visual geometry is not collision geometry; self-collision is disabled",
             "Review only; no hardware, policy or successful jump validation"],
         "verification":{"qpos_to_body_pose_max_residual":max_residual},
         "cad_asset":CAD_PATH}
    dst=Path(dst);dst.parent.mkdir(parents=True,exist_ok=True)
    payload=json.dumps(doc,allow_nan=False,separators=(",",":"))+"\n"
    if dst.exists() and dst.read_text()!=payload:raise ValueError("Immutable replay conflict")
    dst.write_text(payload)
    return {"frames":len(frames),"status":status,"trace_sha256":doc["source"]["trace_sha256"],
            "replay_sha256":hashlib.sha256(payload.encode()).hexdigest(),"qpos_body_residual":max_residual}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--trace",type=Path,required=True);p.add_argument("--out",type=Path,required=True)
    a=p.parse_args();print(json.dumps(build(a.trace,a.out),indent=2))


if __name__=="__main__":main()
