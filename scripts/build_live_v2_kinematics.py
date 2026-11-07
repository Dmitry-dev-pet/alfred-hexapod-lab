#!/usr/bin/env python3
"""Compile an offline joint-gait lookup for the *existing* Hexapod Lab viewer.

Input: owner's pinned CAD30 model and pure LiveGait PR63 generator.
Output: 6 single-axis, unit-amplitude constant-command cycles of 20 joint angles,
        plus a reference MuJoCo kinematics tree. Uses MJ_FORWARD and IK only.
No physics is simulated by this builder and no contact/force claims are made.
The viewer blends the per-axis joint commands; combined commands are only
kinematic previews and NOT a proven physical gait.
"""
from __future__ import annotations
import argparse, hashlib, json, math
from pathlib import Path
from scipy.spatial.transform import Rotation
import numpy as np
import mujoco as mj
from hexlab.live_gait import LEGS, LiveGait, Twist
from hexlab.stand_v04 import static_pose, hold_under_gravity
from hexlab.stand_geometry_v04 import load_legs,tip_world
from hexlab.step_one_leg_v04 import solve_waypoint,forward_from_geometry

MODES={
    'fwd':Twist(vx=.006), 'back':Twist(vx=-.006),
    'left':Twist(vy=.005), 'right':Twist(vy=-.005),
    'ccw':Twist(wz=.075), 'cw':Twist(wz=-.075),
}
PERIOD=2.4
SAMPLES=48
TICK=PERIOD/SAMPLES

def rf(seq,d=7):
    return [round(float(x),d) for x in seq]

def xyzw_to_wxyz(rot):
    x,y,z,w=rot.as_quat()
    if w<0: x,y,z,w=-x,-y,-z,-w
    return [w,x,y,z]

def create_kit(source_dir:Path)->dict:
    model_path=source_dir/'mjcf'/'scene_local30_nominal.xml'
    model=mj.MjModel.from_xml_path(str(model_path))
    data=mj.MjData(model)
    assert model.nq==27 and model.nbody==48 and model.nu==20
    legs=load_legs(model,data,mj)
    static_pose(model,data,mj,legs,.145,.15)
    settle=hold_under_gravity(model,data,mj,data.qpos.copy(),1.0)
    if not settle['stable_by_proxy_thresholds']:raise RuntimeError('Standing state invalid')
    mj.mj_forward(model,data)
    rootid=model.body('base_link_2').id
    rootp=data.xpos[rootid].copy(); R0=data.xmat[rootid].reshape(3,3).copy()
    r0=Rotation.from_matrix(R0)
    forward_world=np.array([*forward_from_geometry(legs),0.])
    forward_body=R0.T@forward_world
    forward_body/=np.linalg.norm(forward_body)
    left_body=np.array([-forward_body[1],forward_body[0],0.]);left_body/=np.linalg.norm(left_body)
    by_name={l['suffix']:l for l in legs}
    anchors_body={l['suffix']:R0.T@(tip_world(model,data,l)-rootp) for l in legs}
    anchors_xy={n:(float(a@forward_body),float(a@left_body)) for n,a in anchors_body.items()}
    qref=data.qpos.copy()
    skeleton=[]
    for bid in range(2,model.nbody):
        parent=int(model.body_parentid[bid]);
        parentp=np.zeros(3) if parent==0 else np.asarray(data.xpos[parent]).copy()
        Rp=np.eye(3) if parent==0 else np.asarray(data.xmat[parent]).reshape(3,3).copy()
        p=(Rp.T@(np.asarray(data.xpos[bid])-parentp))
        Rrel=Rp.T@np.asarray(data.xmat[bid]).reshape(3,3)
        item={'id':bid,'parent':parent,'p':rf(p,9),'q':rf(xyzw_to_wxyz(Rotation.from_matrix(Rrel)),9)}
        if model.body_jntnum[bid]:
            if model.body_jntnum[bid]!=1:raise RuntimeError('Multiple body joints')
            j=int(model.body_jntadr[bid]);adr=int(model.jnt_qposadr[j])
            if model.jnt_type[j]!=mj.mjtJoint.mjJNT_HINGE:raise RuntimeError('Only hinges expected below base')
            item.update(adr=adr-7,axis=rf(Rp.T@data.xaxis[j],9),anchor=rf(Rp.T@(data.xanchor[j]-parentp),9))
        skeleton.append(item)
    qstand=rf(qref[7:],9)
    qdata={};maxerr={}
    for key,cmd in MODES.items():
        gen=LiveGait(anchors_xy,gait='wave',period_s=PERIOD,foot_lift_m=.007,max_stride_m=.022)
        # Bring the scheduler into its settled constant-command cycle.
        for _ in range(SAMPLES*3):
            gen.set_command(cmd);gen.tick(TICK)
        rows=[];errors=[]
        for _ in range(SAMPLES):
            gen.set_command(cmd);g=gen.tick(TICK)
            # Solve each leg in the original settled root frame. Root never moves.
            q=qref.copy()
            for name in LEGS:
                reference=mj.MjData(model)
                reference.qpos[:]=qref
                reference.qvel[:]=0
                mj.mj_forward(model,reference)
                dx,dy,dz=g.foot_offsets[name]
                offset=dx*forward_body+dy*left_body+np.array([0,0,dz])
                desired=rootp + R0@(anchors_body[name]+offset)
                leg=by_name[name]
                v,err=solve_waypoint(model,reference,mj,leg,desired,qref[leg['qadrs']])
                if not np.isfinite(err) or err>.003:raise RuntimeError(f'Bad IK {key} {name}: {err}')
                q[leg['qadrs']]=v
                errors.append(float(err))
            if np.max(np.abs(q[7:25]))>1.7:raise RuntimeError('Joint angle limit exceeded')
            rows.append(rf(q[7:],8))
        qdata[key]=rows
        maxerr[key]=round(max(errors)*1000,5)
    kit={
        'format':'fnk0052-cad30-kinematic-control-v1',
        'study':'issue62-live-v2-offline',
        'notice':'Kinematic preview only. No live physics, contact force, or physical stability guarantee.',
        'upstream_cad_commit':'5c9a84c16335b30036e451c691d2c544abf4bc72',
        'generator':'hexlab/live_gait.py PR63; Python inverse kinematics and MuJoCo mj_forward OFFLINE only',
        'period_s':PERIOD,'sample_interval_s':TICK,'samples':SAMPLES,
        'max_amplitudes':{'vx':.006,'vy':.005,'wz':.075},
        'root_position':rf(rootp,9),
        'root_quat_wxyz':rf(xyzw_to_wxyz(r0),9),
        'forward_world':rf(forward_world,9),
        'left_world':rf(R0@left_body,9),
        'q_stand':qstand,'leg_order':list(LEGS),
        'toe_local':{str(l['body']):rf(l['tip_local'],8) for l in legs},
        'skeleton':skeleton,'joint_tables':qdata,
        'max_offline_IK_error_mm':maxerr,
        'limitations':['No MuJoCo physics at runtime','Runtime combinations not independently physics verified','joint interpolation does not prove contact/stability','Servo PWM/calibration unknown','Body motion is commanded kinematic integration, not observed displacement','Original CAD assets not redistributed']}
    return kit

def main():
    a=argparse.ArgumentParser();a.add_argument('--source-dir',type=Path,required=True);a.add_argument('--output',type=Path,required=True)
    ns=a.parse_args()
    kit=create_kit(ns.source_dir)
    ns.output.parent.mkdir(parents=True,exist_ok=True)
    ns.output.write_text(json.dumps(kit,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({'out':str(ns.output),'bytes':ns.output.stat().st_size,'sha256':hashlib.sha256(ns.output.read_bytes()).hexdigest(),'max_ik_error_mm':kit['max_offline_IK_error_mm'],'modes':list(MODES),'samples':SAMPLES}))
if __name__=='__main__':main()
