"""MJCF with source-derived four-bar closure and explicit surrogate inertias."""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from .linkage import PINNED_BLOB, solve

HERE = Path(__file__).resolve().parent


def load_evidence(path=HERE / "linkage_evidence.json"):
    e = json.loads(Path(path).read_text())
    if e.get("schema") != "en01-fourbar-source-evidence/v1" or e.get("source", {}).get("git_blob_sha1") != PINNED_BLOB:
        raise ValueError("Unrecognised source evidence")
    return e


def create(evidence=None, *, fixed_base=False, gravity=True, timestep=.001):
    e = evidence or load_evidence()
    if e.get("source", {}).get("git_blob_sha1") != PINNED_BLOB:
        raise ValueError("Wrong geometry provenance")
    P = e["sides"]["cad_z_positive"]["points_xy_mm"]
    A = P["A"]
    def v(key):
        p = P[key]
        return [(p[0]-A[0])/1000, 0, (p[1]-A[1])/1000-.015]
    pts = {k:v(k) for k in "ABCDW"}
    def sub(a,b): return [x-y for x,y in zip(a,b)]
    def fmt(a): return " ".join(f"{x:.12g}" for x in a)
    r = ET.Element("mujoco", model="EN01_v2_CAD_fourbar_SURROGATE_DYNAMICS")
    r.append(ET.Comment("CAD axes/planar closure only. Masses, motor limits, servo zero, springs and collisions UNCALIBRATED."))
    ET.SubElement(r,"compiler",angle="radian",autolimits="true")
    ET.SubElement(r,"option",timestep=str(timestep),gravity="0 0 -9.81" if gravity else "0 0 0",integrator="implicitfast",solver="Newton",iterations="100",tolerance="1e-10")
    default=ET.SubElement(r,"default")
    ET.SubElement(default,"joint",damping="0.0005",armature="0.0000001")
    ET.SubElement(default,"geom",contype="2",conaffinity="1",condim="3",friction="0.9 0.005 0.0001",solref="0.006 1")
    world=ET.SubElement(r,"worldbody")
    ET.SubElement(world,"geom",name="floor",type="plane",size="3 3 .1",contype="1",conaffinity="2")
    root_z=.0185-pts["W"][2]
    torso=ET.SubElement(world,"body",name="torso",pos=f"0 0 {root_z:.12g}")
    if not fixed_base: ET.SubElement(torso,"freejoint",name="root")
    ET.SubElement(torso,"geom",name="torso_geom",type="box",size=".047 .034 .024",mass=".180")
    ET.SubElement(torso,"site",name="imu_site",size=".002")
    equality=ET.SubElement(r,"equality")
    actuator=ET.SubElement(r,"actuator")
    for side, sign in [("left",1),("right",-1)]:
        # The planar projection chooses one load plane per side. Lateral load
        # distribution remains an assumption; tire midplanes are source measured.
        zcad=e["sides"]["cad_z_negative" if sign==1 else "cad_z_positive"]["tire_midplane_cad_z_mm"]
        y=-(zcad-32)/1000
        positions={k:[p[0],y,p[2]] for k,p in pts.items()}
        upper=ET.SubElement(torso,"body",name=side+"_upper",pos=fmt(positions["A"]))
        ET.SubElement(upper,"joint",name="hip_"+side,axis="0 1 0",range="-.22 .65",armature=".00002")
        B=sub(positions["B"],positions["A"])
        ET.SubElement(upper,"geom",name=side+"_upper_geom",type="capsule",fromto=fmt([0,0,0]+B),size=".004",mass=".009")
        calf=ET.SubElement(upper,"body",name=side+"_calf",pos=fmt(B))
        ET.SubElement(calf,"joint",name="knee_"+side,axis="0 1 0",limited="false")
        W=sub(positions["W"],positions["B"]);C=sub(positions["C"],positions["B"])
        ET.SubElement(calf,"geom",name=side+"_calf_geom",type="capsule",fromto=fmt(C+W),size=".004",mass=".013")
        ET.SubElement(calf,"site",name=side+"_closure_calf",pos=fmt(C),size=".001")
        wheel=ET.SubElement(calf,"body",name=side+"_wheel",pos=fmt(W))
        ET.SubElement(wheel,"joint",name="wheel_"+side,axis="0 1 0",limited="false",armature=".00005",damping=".0002")
        ET.SubElement(wheel,"geom",name=side+"_wheel_geom",type="cylinder",size=".0185 .0065",quat=".707106781 .707106781 0 0",mass=".047")
        rod=ET.SubElement(torso,"body",name=side+"_rod",pos=fmt(positions["D"]))
        ET.SubElement(rod,"joint",name="rod_"+side,axis="0 1 0",limited="false")
        DC=sub(positions["C"],positions["D"])
        ET.SubElement(rod,"geom",name=side+"_rod_geom",type="capsule",fromto=fmt([0,0,0]+DC),size=".002",mass=".002")
        ET.SubElement(rod,"site",name=side+"_closure_rod",pos=fmt(DC),size=".001")
        ET.SubElement(equality,"connect",name=side+"_fourbar",site1=side+"_closure_calf",site2=side+"_closure_rod",solref=".003 1",solimp=".99 .9999 .001")
        ET.SubElement(actuator,"motor",name="a_hip_"+side,joint="hip_"+side,gear="1",ctrlrange="-.14709975 .14709975")
        ET.SubElement(actuator,"motor",name="a_wheel_"+side,joint="wheel_"+side,gear="1",ctrlrange="-.03 .03")
    sensor=ET.SubElement(r,"sensor")
    ET.SubElement(sensor,"gyro",name="imu_gyro",site="imu_site")
    ET.indent(r,space="  ")
    return ET.tostring(r,encoding="unicode")+"\n"


def set_configuration(model,data,points,q):
    """Set analytical closed-loop pose for exact independent FK checks."""
    import mujoco
    s=solve(points,-q)
    for side in ("left","right"):
        for name,value in [("hip_",q),("knee_",-s["calf_delta_rad"]-q),("rod_",-s["rod_delta_rad"])]:
            j=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,name+side)
            data.qpos[model.jnt_qposadr[j]]=value
    mujoco.mj_forward(model,data)
    return s


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--out",type=Path,default=HERE/"model_v2_fourbar.xml")
    a=p.parse_args();a.out.write_text(create());print(hashlib.sha256(a.out.read_bytes()).hexdigest())


if __name__=="__main__":main()
