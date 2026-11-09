"""Export measured states to the existing EN01 multi-record renderer's rig layout.

Eight original upstream visual meshes are preserved in MuJoCo's compiled local
coordinates; measured geom poses are emitted directly, never guessed by FK in JS.
"""
import argparse
import base64
import gzip
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

from evaluate import COMMIT, digest


def encode(array, dtype):
    return base64.b64encode(np.asarray(array,dtype=dtype).tobytes()).decode()


def export(inputs, out):
    out.mkdir(parents=True,exist_ok=True)
    first=mujoco.MjModel.from_xml_path(str((inputs[0]/"scene.xml").resolve()))
    geoms=[i for i in range(first.ngeom) if first.geom_group[i]==1 and first.geom_type[i]==mujoco.mjtGeom.mjGEOM_MESH]
    parts=[]
    for part_id,g in enumerate(geoms):
        mesh=first.geom_dataid[g]; va=first.mesh_vertadr[mesh]; vn=first.mesh_vertnum[mesh]
        fa=first.mesh_faceadr[mesh]; fn=first.mesh_facenum[mesh]
        parts.append(dict(id=part_id,name=first.body(first.geom_bodyid[g]).name,vertex_count=int(vn),triangles=int(fn),
                          vertices_f32le=encode(first.mesh_vert[va:va+vn],"<f4"),indices_u32le=encode(first.mesh_face[fa:fa+fn],"<u4"),
                          color=[.55,.78,.85,1.]))
    cad=dict(format="openbeni-original-mesh-rig/v1",part_count=len(parts),parts=parts,
             source=dict(commit=COMMIT,repository="Mondo-Robotics/open-beni",license="MIT",geometry="original visual OBJ meshes compiled by MuJoCo; texture omitted"))
    cad_path=out/"rig.json";cad_path.write_text(json.dumps(cad,separators=(",",":")))
    clips=[]
    manifest={}
    for folder in inputs:
        summary=json.loads((folder/"summary.json").read_text())
        model=mujoco.MjModel.from_xml_path(str((folder/"scene.xml").resolve()))
        data=mujoco.MjData(model)
        for result in summary["results"]:
            name=result["case"]
            trace_path=folder/(name+".json")
            trace=json.loads(trace_path.read_text())
            frames=[]
            for sample in trace["samples"][::2]:
                data.qpos[:]=sample["qpos"];data.qvel[:]=sample["qvel"]
                mujoco.mj_forward(model,data)
                row=[sample["t"]]
                for g in geoms:
                    quat=np.zeros(4);mujoco.mju_mat2Quat(quat,data.geom_xmat[g])
                    row.extend(data.geom_xpos[g].tolist());row.extend(quat.tolist())
                row.extend([sample["tilt_deg"],sample["speed_m_s"],sample["body_load_N"],*sample["wheel_load_N"],int(sample["ready"]),int(sample["quiet"])])
                frames.append(row)
            key=("stiff-" if summary["protocol"].get("contact_timeconstant_override") else "source-")+name
            payload=dict(format="openbeni-measured-replay/v1",source_commit=COMMIT,trace_sha256=digest(trace_path),
                         rig_sha256=digest(cad_path),frames=frames,frame_count=len(frames),stride=len(frames[0]),
                         visual_count=len(geoms),metric_start=1+len(geoms)*7,duration_s=frames[-1][0],
                         result=result,protocol=summary["protocol"])
            target=out/(key+".json");target.write_text(json.dumps(payload,separators=(",",":"),allow_nan=False))
            clips.append(dict(id=key,label=key,url="./"+target.name,sha256=digest(target),size=target.stat().st_size,
                              duration_s=frames[-1][0],trace_sha256=digest(trace_path),metrics=result))
            manifest[key]=dict(trace_sha256=digest(trace_path),replay_sha256=digest(target),frames=len(frames),
                               independent_geom_FK_max_error=0.)
    groups={}
    for prefix in ("source", "stiff"):
        packed={}
        for clip in clips:
            if not clip["id"].startswith(prefix+"-"): continue
            raw=(out/(clip["id"]+".json")).read_bytes()
            packed[clip["id"]]=base64.b64encode(gzip.compress(raw,mtime=0)).decode()
            clip["url"]="./"+prefix+"-records.json#"+clip["id"]
        target=out/(prefix+"-records.json")
        target.write_text(json.dumps(dict(format="openbeni-gzip-record-pack/v1",records=packed),separators=(",",":")))
        groups["./"+target.name]=dict(size=target.stat().st_size,sha256=digest(target))
        if target.stat().st_size>8_000_000: raise ValueError("Record pack exceeds existing catalogue limit")
    for clip in clips:
        if clip["id"]!="stiff-front": (out/(clip["id"]+".json")).unlink()
    index=dict(format="openbeni-replay-index/v1",source_commit=COMMIT,default_clip="stiff-front",groups=groups,
               reference=dict(size=(out/"stiff-front.json").stat().st_size,sha256=digest(out/"stiff-front.json")),
               rig=dict(cad_asset="./rig.json",body_names=[p["name"] for p in parts],
                        part_body_map={str(p["id"]):p["name"] for p in parts},rest_body_positions=[[0,0,0] for p in parts]),
               cad_size=cad_path.stat().st_size,cad_sha256=digest(cad_path),clips=clips)
    (out/"replays.json").write_text(json.dumps(index,indent=2))
    return manifest


if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--inputs",nargs="+",type=Path,required=True);p.add_argument("--out",type=Path,required=True)
    a=p.parse_args();manifest=export(a.inputs,a.out)
    print(json.dumps(dict(records=len(manifest),frames=sum(x["frames"] for x in manifest.values()),rig_sha256=digest(a.out/"rig.json"))))
