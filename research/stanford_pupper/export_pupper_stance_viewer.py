#!/usr/bin/env python3
"""Export source-locked Pupper CPU stance to a physics-only tablet replay.

Reads real qpos/contact forces from reproduce_pupper.py's mj_step output.
Only mj_forward is used here to recover exact original body transforms; this
script does not create learned commands, gait cycles or simulated forces.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from reproduce_pupper import read_pinned_xml, load_model, FOOT_GEOMS

HERE = Path(__file__).resolve().parent
PIN = "aa475e9a393a209b29fd5de7e0ee10f3de643230"
ORIGIN = "https://raw.githubusercontent.com/G-Levine/pupper_v3_description/1a3c8590189d3a406219d155f4177afc491a7d77/description/meshes/stl/"


def values(raw: str | None, defaults: list[float]) -> list[float]:
    return [float(v) for v in raw.split()] if raw is not None else defaults


def mesh_descriptors(xml: bytes) -> list[dict]:
    root=ET.fromstring(xml)
    assets={m.get("name"):m for m in root.findall("asset/mesh")}
    manifest=json.loads((HERE/"upstream/STL_SOURCE_MANIFEST.json").read_text())
    by_name={f["name"]:f for f in manifest["files"]}
    out=[]
    for body in root.findall(".//body"):
        for geom in body.findall("geom"):
            if geom.get("type")!="mesh":
                continue
            assert geom.get("density")=="0" and geom.get("class") is None
            asset=assets[geom.get("mesh")]
            name=asset.get("file")
            ref=by_name[name]
            out.append({
                "body":body.get("name"),"mesh":name,
                "size":ref["size_bytes"],"git_blob_sha1":ref["git_blob_sha"],
                "geom_pos":values(geom.get("pos"),[0.,0.,0.]),
                "geom_quat_wxyz":values(geom.get("quat"),[1.,0.,0.,0.]),
                "scale":values(asset.get("scale"),[1.,1.,1.]),
            })
    assert len(out)==13 and len({v["mesh"] for v in out})==13
    return out


def export(xmlpath: Path, tracepath: Path, outpath: Path) -> dict:
    xml=read_pinned_xml(xmlpath)
    model=load_model(xml)
    raw=tracepath.read_bytes()
    trace=json.loads(raw)
    assert trace["schema"]=="alfred-pupper-physics-only-v1"
    assert trace["policy_inference"]=="NOT_RUN" and trace["rl_training"]=="NOT_RUN"
    assert trace["source"]["commit"]==PIN
    assert trace["experiment"]["mj_step_count"]==1250
    assert trace["experiment"]["stored_frames"]==250
    body_names=["base_link", *(f"leg_{p}_{s}_{i}" for p,s in
                 (("front","r"),("front","l"),("back","r"),("back","l")) for i in (1,2,3))]
    foot_names=[f"leg_{p}_{s}_3_foot_site" for p,s in
                 (("front","r"),("front","l"),("back","r"),("back","l"))]
    bodyids=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,n) for n in body_names]
    footids=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,n) for n in foot_names]
    assert min(bodyids)>=0 and min(footids)>=0
    data=mujoco.MjData(model)
    frames=[]
    for row in trace["experiment"]["frames"]:
        mujoco.mj_resetData(model,data)
        q=row["base_xyz"]+row["base_quat_wxyz"]+row["joint_q"]
        assert len(q)==19
        data.qpos[:]=q
        mujoco.mj_forward(model,data)
        assert np.max(np.abs(data.xpos[bodyids[0]]-np.array(row["base_xyz"]))) < 2e-8
        f=[float(row["foot_normal_N"][g]) for g in FOOT_GEOMS]
        assert sum(force>0.2 for force in f)==row["feet_loaded"]
        pose=[]
        for body in bodyids:
            pose.extend([round(float(v),8) for v in
                         (*data.xpos[body],*data.xquat[body])])
        feet=[]
        for site in footids:
            feet.extend(round(float(v),8) for v in data.site_xpos[site])
        fr=[row["t"],*pose,*feet,*f,int(row["feet_loaded"])]
        assert len(fr)==109
        frames.append(fr)
    assert len(frames)==250 and abs(frames[-1][0]-5)<1e-8
    doc={
        "schema":"pupper-v3-mujoco-cpu-stance-replay-v1",
        "physics":"MuJoCo 3.14.0 CPU, 0.004 s timestep, no training or inference",
        "is_walk":False,
        "source":{
            "repo":"cs123-stanford/pupper-mjlab","commit":PIN,
            "original_xml_git_blob_sha1":"7771e176c42b3aed1d48cc62434771b0fceb7c8c",
            "trace_sha256":hashlib.sha256(raw).hexdigest(),
            "trace_bytes":len(raw),
            "visual_source_prefix":ORIGIN,
        },
        "duration_s":5.0,"physics_steps":1250,"frame_rate_hz":50,
        "layout":{
            "stride":109,"t_index":0,"body_pose_start":1,"body_pose_stride":7,
            "body_names":body_names,"foot_xyz_start":92,"foot_site_names":foot_names,
            "foot_fz_start":104,"foot_count_index":108,"body_pose_quaternion_order":"wxyz",
        },
        "visual_meshes":mesh_descriptors(xml),
        "summary":{
            "mass_kg":3.17,"initial_base_z_m":0.18,
            "final_base_z_m":trace["experiment"]["final_base_height_m"],
            "final_loaded_feet":4,
            "peak_abs_servo_torque_Nm":trace["experiment"]["peak_abs_actuator_force_Nm"],
        },
        "frames":frames,
    }
    outpath.parent.mkdir(parents=True,exist_ok=True)
    outpath.write_bytes(json.dumps(doc,ensure_ascii=False,separators=(",",":")).encode("utf8"))
    print("PUPPER_VIEWER_EXPORT_OK",len(frames),"frames",len(doc["visual_meshes"]),"original STL")
    return doc


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--xml",type=Path,default=HERE/"upstream/pupper_v3.xml")
    parser.add_argument("--trace",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    export(args.xml,args.trace,args.output)
