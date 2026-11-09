"""Bounded CPU dynamics of EN01's source-derived four-bar, with assumed masses.

No robot firmware, learned policy, calibrated servo zero or spring model is used.
Every state and visual body pose comes from real mj_step then mj_forward.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
from .linkage_model import create, load_evidence

HERE=Path(__file__).resolve().parent
MODES=("stand","crouch","jump","push","drive","fall")


def target(t,mode):
    # Model convention only: negative q raises wheel relative to hip.
    if mode in ("stand","push","drive","fall"): return 0.0
    if t<1.5: return 0.0
    if t<3.35: return -.18*min(1,(t-1.5)/1.85)
    if mode=="crouch" or t<3.6: return -.18
    return .20


def run(mode,out,duration=6.,timestep=.001):
    import mujoco
    import numpy as np
    if mode not in MODES: raise ValueError(mode)
    if duration<=0 or not math.isfinite(duration): raise ValueError("Invalid duration")
    e=load_evidence(); xml=create(e,timestep=timestep)
    model=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(model)
    bid=lambda n:mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,n)
    jid=lambda n:mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,n)
    aid=lambda n:mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_ACTUATOR,n)
    sid=lambda n:mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,n)
    torso=bid("torso");floor=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,"floor")
    wheels={mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,s+"_wheel_geom") for s in ("left","right")}
    bodies=[mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,b) for b in range(1,model.nbody)]
    joints={mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,j):int(model.jnt_qposadr[j]) for j in range(model.njnt)}
    if mode=="fall":
        # Independent explicit 60-degree lateral drop; no self-right claim.
        d.qpos[2]+=.025;d.qpos[3:7]=[math.cos(math.pi/6),math.sin(math.pi/6),0,0]
    mujoco.mj_forward(model,d)
    frames=[];max_closure=0.;max_pitch=0.;max_roll=0.;max_x=0.;nonwheel_steps=0;max_pen=0.;max_qvel=0.
    flight_start=None;candidate=False;touchdown=None;stable=0.;best_stable=0.;flights=[];pre_ok=0;pre_n=0
    steps=round(duration/timestep);sample_every=round(.02/timestep)
    force=np.zeros(6)
    for step in range(steps):
        t=float(d.time); goal=target(t,mode)
        for side in ("left","right"):
            j=jid("hip_"+side);q=float(d.qpos[model.jnt_qposadr[j]]);v=float(d.qvel[model.jnt_dofadr[j]])
            raw=3*(goal-q)-.045*v
            # Continuous nominal torque ceiling; linear no-load derating is an
            # assumed diagnostic curve. Braking is bounded at the same ceiling.
            limit=.14709975*(max(0,1-abs(v)/11.6355) if raw*v>0 else 1)
            d.ctrl[aid("a_hip_"+side)]=np.clip(raw,-limit,limit)
        R=d.xmat[torso].reshape(3,3)
        pitch=math.atan2(-R[2,0],R[2,2]);gyro=float(d.sensor("imu_gyro").data[1])
        mean_rate=sum(float(d.qvel[model.jnt_dofadr[jid("wheel_"+s)]]) for s in ("left","right"))/2
        desired=.10/.0185 if mode=="drive" and 1<=t<4 else 0.
        torque=float(np.clip(.15*pitch+.05*gyro+.008*(mean_rate-desired),-.03,.03))
        for s in ("left","right"): d.ctrl[aid("a_wheel_"+s)]=torque
        d.xfrc_applied[:]=0
        if mode=="push" and 2<=t<2.1:d.xfrc_applied[torso,0]=.3
        mujoco.mj_step(model,d)
        mujoco.mj_forward(model,d)
        mujoco.mj_subtreeVel(model,d)
        if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all():
            raise RuntimeError("Nonfinite physics; refusing replay")
        R=d.xmat[torso].reshape(3,3)
        pitch=float(math.atan2(-R[2,0],R[2,2]));roll=float(math.atan2(R[2,1],R[2,2]))
        max_pitch=max(max_pitch,abs(pitch));max_roll=max(max_roll,abs(roll));max_x=max(max_x,abs(float(d.qpos[0])))
        max_qvel=max(max_qvel,float(max(abs(d.qvel))))
        closure=max(float(np.linalg.norm(d.site_xpos[sid(s+"_closure_calf")]-d.site_xpos[sid(s+"_closure_rod")])) for s in ("left","right"))
        max_closure=max(max_closure,closure)
        contact_names=[];normal=0.;nonwheel=False;pen=0.
        for i in range(d.ncon):
            c=d.contact[i]
            if floor not in (int(c.geom1),int(c.geom2)):continue
            other=int(c.geom2) if int(c.geom1)==floor else int(c.geom1)
            mujoco.mj_contactForce(model,d,i,force)
            normal+=max(0,float(force[0]));nonwheel|=other not in wheels
            pen=max(pen,max(0,-float(c.dist)))
            contact_names.append(mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,other))
        max_pen=max(max_pen,pen);nonwheel_steps+=int(nonwheel)
        upright=abs(pitch)<.35 and abs(roll)<.35 and not nonwheel
        ground=bool(contact_names)
        now=float(d.time)
        if mode=="jump":
            if 3.35<=now<3.6:pre_n+=1;pre_ok+=int(ground and upright)
            if now>=3.6:
                if not ground and flight_start is None:
                    flight_start=now;candidate=upright and float(d.subtree_linvel[torso,2])>.05 and pre_n>0 and pre_ok/pre_n>=.95
                if ground and flight_start is not None:
                    dt=now-flight_start
                    flights.append({"start_s":flight_start,"duration_s":dt,"qualified":bool(candidate and dt>=.02)})
                    touchdown=now if candidate and dt>=.02 else None;stable=0.;flight_start=None
                if touchdown is not None and ground and upright:
                    stable+=timestep;best_stable=max(best_stable,stable)
                elif touchdown is not None:
                    stable=0.;touchdown=None
        if step%sample_every==0:
            frames.append({"t":now,"qpos":d.qpos.tolist(),"qvel":d.qvel.tolist(),"ctrl":d.ctrl.tolist(),
                           "body_poses":[[*d.xpos[bid(n)].tolist(),*d.xquat[bid(n)].tolist()] for n in bodies],
                           "ground_contact":ground,"nonwheel_ground_contact":nonwheel,"contact_geoms":contact_names,
                           "ground_normal_force_n":normal,"pitch_rad":pitch,"roll_rad":roll,"closure_error_m":closure})
    qualified=any(f["qualified"] for f in flights)
    # A nominal upright state is diagnostic; no success claim on hardware.
    summary={"schema":"en01-fourbar-diagnostic/v1","mode":mode,"mujoco_version":mujoco.__version__,
             "duration_s":duration,"timestep_s":timestep,"physics_steps":steps,"frame_count":len(frames),
             "total_mass_kg":float(sum(model.body_mass)),"mass_status":"UNMEASURED_v0_aggregate_redistributed",
             "hip_continuous_ceiling_nm":.14709975,"wheel_torque_nm":"ASSUMED_0.030",
             "spring_model":"OMITTED_unknown_stiffness_and_preload", "model_sha256":hashlib.sha256(xml.encode()).hexdigest(),
             "geometry_source_blob":e["source"]["git_blob_sha1"],"max_closure_error_m":max_closure,
             "max_abs_pitch_deg":math.degrees(max_pitch),"max_abs_roll_deg":math.degrees(max_roll),
             "max_abs_x_m":max_x,"final_x_m":float(d.qpos[0]),"nonwheel_ground_steps":nonwheel_steps,
             "max_floor_penetration_m":max_pen,"max_abs_qvel":max_qvel,"flights":flights,
             "stable_landing_s":best_stable,"jump_pass":qualified and best_stable>=2,
             "upright_6s":nonwheel_steps==0 and max_pitch<.35 and max_roll<.35,
             "status":"REVIEW_ONLY_NO_HARDWARE_OR_LEARNED_POLICY_CLAIM"}
    if flight_start is not None:summary["unfinished_airborne_interval_s"]=float(d.time)-flight_start
    out=Path(out);out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps({"summary":summary,"body_names":bodies,"joint_qpos_addresses":joints,"frames":frames},allow_nan=False,separators=(",",":"))+"\n")
    summary["trace_sha256"]=hashlib.sha256(out.read_bytes()).hexdigest()
    out.with_name(out.stem+"_summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    return summary


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--mode",choices=MODES,required=True)
    p.add_argument("--out",type=Path,required=True);p.add_argument("--timestep",type=float,default=.001)
    a=p.parse_args();print(json.dumps(run(a.mode,a.out,timestep=a.timestep),indent=2))


if __name__=="__main__":main()
