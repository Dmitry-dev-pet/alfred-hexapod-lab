#!/usr/bin/env python3
"""Verified recorded-qpos exporter for NavBot EN01. No invented animation or physics."""
from __future__ import annotations
import argparse, hashlib, json, math
from pathlib import Path

FMT = "navbot-en01-mujoco-physical-replay-v1"
SHA = "3139f2fb32b38d10611feb905e16f8aa012f2f2e"
MODES = {"stand": ("research", "Стойка: без падения, с дрейфом"),
         "crouch": ("research", "Присед: без падения, с дрейфом"),
         "jump": ("failed", "Медленный прыжок: неудачная попытка"),
         "jump_fast": ("failed", "Быстрый прыжок: падение до отрыва")}
COLUMNS = ["time_s","x_m","y_m","z_m","qw","qx","qy","qz","hip_left_rad",
           "wheel_left_rad","hip_right_rad","wheel_right_rad",
           "ground_0_1","torso_floor_0_1","normal_force_N","pitch_rad"]

def digest(raw): return hashlib.sha256(raw).hexdigest()
def number(x): return type(x) in (int,float) and math.isfinite(x)

def export(source:Path, destination:Path, mode:str):
    if mode not in MODES: raise ValueError("unknown experiment")
    raw=source.read_bytes()
    obj=json.loads(raw)
    if obj.get("source_sha")!=SHA or obj.get("mode")!=mode:
        raise ValueError("source SHA or mode mismatch")
    if obj.get("mujoco_version")!="3.14.0" or obj.get("physics_steps")!=3000:
        raise ValueError("unverified physics runtime")
    if obj.get("result_classification")!="EXPLORATORY_ONLY_NOT_SIM_TO_REAL_VALIDATED":
        raise ValueError("unexpected research status")
    if obj.get("model")!="hand-built approximate EN01, NOT upstream CAD":
        raise ValueError("unexpected geometry/model provenance")
    samples=obj.get("frames")
    if not isinstance(samples,list) or len(samples)!=300:
        raise ValueError("missing recorded frames")
    rows=[];last=-1
    for sample in samples:
        t=sample.get("t");q=sample.get("qpos");v=sample.get("qvel");u=sample.get("ctrl")
        if not number(t) or t<=last or abs(t-(len(rows)*0.02+0.002))>0.00001:
            raise ValueError("invalid sample time")
        last=t
        for x,n in ((q,11),(v,10),(u,4)):
            if not isinstance(x,list) or len(x)!=n or not all(map(number,x)):
                raise ValueError("nonfinite physical state")
        if abs(sum(z*z for z in q[3:7])-1)>0.05:raise ValueError("invalid orientation")
        a=sample.get("ground_contact");b=sample.get("torso_ground_contact")
        if type(a)!=bool or type(b)!=bool or (b and not a):raise ValueError("invalid contact")
        if not number(sample.get("ground_normal_force_n")) or not number(sample.get("pitch_rad")):
            raise ValueError("missing contact/IMU measurement")
        rows.append([t,*q,int(a),int(b),sample["ground_normal_force_n"],sample["pitch_rad"]])
    if mode.startswith("jump"):
        if obj.get("has_flight_ge_20ms") is not False or obj.get("stable_landing_2s_observed") is not False:
            raise ValueError("reassess jump before publishing")
        if obj.get("torso_ground_steps",0)<1:raise ValueError("missing verified fall")
    elif obj.get("stable_stand_or_crouch") is not True or obj.get("torso_ground_steps")!=0:
        raise ValueError("upright status contradicted")
    if not all(number(obj.get(k)) for k in ("max_abs_pitch_rad","max_abs_x_m")):
        raise ValueError("missing outcome metrics")
    status,desc=MODES[mode]
    xml=Path(__file__).with_name("model_v0.xml")
    result={"format":FMT,"schema":FMT,
      "title":"NavBot EN01 · "+desc,"detail":desc,
      "robot":"EN01 approximate four-actuator model, NOT original CAD",
      "physics_engine":"MuJoCo","physics_version":"3.14.0","duration_s":6.0,
      "status":status,
      "source":{"upstream_commit":SHA,"trace_sha256":digest(raw),
        "model_xml_sha256":digest(xml.read_bytes()),"physics_steps":3000,
        "controller":"experimental PD, NOT upstream firmware"},
      "result":{key:obj.get(key) for key in
        ("mode","max_abs_pitch_rad","max_abs_x_m","torso_ground_steps",
         "has_flight_ge_20ms","stable_landing_2s_observed")},
      "limitations":["Collision geometry is approximate; not aligned to original STEP.",
        "Wheel torque is assumed; motor SKU unresolved.",
        "No verified physical jump or sim-to-real proof."],
      "frame_columns":COLUMNS,"frame_count":len(rows),
      "sample_stride":len(COLUMNS),"frames":rows}
    data=(json.dumps(result,ensure_ascii=False,allow_nan=False,separators=(",",":"))+"\n").encode()
    if destination.exists() and destination.read_bytes()!=data:
        raise ValueError("immutable replay already differs")
    destination.parent.mkdir(parents=True,exist_ok=True)
    destination.write_bytes(data)
    return {"status":status,"frames":len(rows),"trace_sha256":digest(raw),
            "replay_sha256":digest(data)}

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--trace",required=True,type=Path)
    ap.add_argument("--out",required=True,type=Path)
    ap.add_argument("--mode",required=True,choices=list(MODES))
    a=ap.parse_args()
    print(json.dumps(export(a.trace,a.out,a.mode),indent=2))

if __name__=="__main__":main()
