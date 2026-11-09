#!/usr/bin/env python3
"""Mini Wheelbot source-pulse MuJoCo diagnostic with non-colliding CAD visuals omitted.

Pinned numeric source parameters and collision/inertia assumptions from PR #86.
This is a real mj_step rollout of the PR #86 experimental contact-proxy model,
not a rendering of original CAD or a hardware validated digital twin.
"""
import argparse, hashlib, json, math
from pathlib import Path
import mujoco as mj
import numpy as np

ORIGINAL_COMMIT='73815e49371c5fb5f49eec59d364e4d9d1b57b10'
PARAMS={'m_WR':0.154800009999768, 'm_B':0.22470645904855896,
        'I_Wxz_Ryz':6.360999999966876e-05, 'I_Wy_Rx':0.00010279151990686121,
        'I_Bx':0.0001099900000194522,'I_By':0.0004500099999982192,'I_Bz':0.0002850099999992028,
        'r_W':0.032,'l_WB':0.065,'fric_magn':0.002802071865854826,'fric_slope':119.99742098981706}
SOURCE_CONTROLLER={'source':'wheelbot/Mini-Wheelbot/wheelbot-lib/config/default.json',
                 'inputs_standup_rp':[0.55, 3600, 1500],
                 'inputs_standup_rn':[-0.55,3600,1500],
                 'K_roll':[-1.3,-0.16,-0.00008,-0.0004]}
DT=.001
OMEGA_CAP=22.2*380*2*math.pi/60

def xml_model(profile='stiff'):
    p=PARAMS
    ix,iy,iz=[p[k] for k in ('I_Bx','I_By','I_Bz')]
    violation=max(0., iy-ix-iz)
    if violation:
        ix+=violation/3; iy-=violation/3; iz+=violation/3
    bix,biy,biz=ix,iz,iy
    axle=p['I_Wy_Rx']; side=p['I_Wxz_Ryz']; mw=p['m_WR']; mb=p['m_B']
    xml=f'''<mujoco model="wheelbot-step-axis-collision-only"><compiler angle="radian"/>
      <option timestep="{DT}" gravity="0 0 -9.81" integrator="implicitfast" iterations="100"/>
      <worldbody>
        <geom name="ground" type="plane" size="3 3 .1" friction="0.9 .01 .001"/>
        <body name="body" pos="0 0 .065">
          <freejoint name="base"/>
          <inertial pos="0 0 .0325" mass="{mb}" diaginertia="{bix} {biy} {biz}"/>
          <geom name="body_collision_PROXY" type="box" size=".043 .0515 .043" friction=".8 .01 .001" rgba=".45 .6 .75 .16"/>
          <body name="lower_drive" pos="0 0 -.0325">
            <joint name="drive_axis_STEPX" type="hinge" axis="1 0 0" damping=".0002"/>
            <inertial pos="0 0 0" mass="{mw}" diaginertia="{axle} {side} {side}"/>
            <geom name="lower_drive_coll_PROXY" type="cylinder" size=".032 .014" quat=".707106781 0 .707106781 0" friction="1 .01 .001"/>
          </body>
          <body name="upper_reaction" pos="0 0 .0325">
            <joint name="reaction_axis_STEPZ" type="hinge" axis="0 -1 0" damping=".0002"/>
            <inertial pos="0 0 0" mass="{mw}" diaginertia="{side} {axle} {side}"/>
            <geom name="upper_reaction_coll_PROXY" type="cylinder" size=".032 .014" quat=".707106781 .707106781 0 0" friction="1 .01 .001"/>
          </body>
        </body>
      </worldbody>
      <actuator>
        <motor name="drive" joint="drive_axis_STEPX" gear="1" ctrllimited="true" ctrlrange="-.55 .55"/>
        <motor name="reaction" joint="reaction_axis_STEPZ" gear="1" ctrllimited="true" ctrlrange="-.55 .55"/>
      </actuator>
    </mujoco>'''
    if profile=='stiff':
        for friction in ('0.9', '0.8', '1'):
            needle=f'friction="{friction} .01 .001"'
            xml=xml.replace(needle, needle+' solref=".003 1" solimp=".97 .99 .0001"')
    else: raise ValueError('Only pinned stiff profile supported')
    return xml

def upstream_roll_input(t,sign=1):
    ms=(t-1.)*1000
    if ms<0: return 0.
    if ms<=3600:return -.55*sign*(ms/3600)*.6
    if ms<=5100:return .55*sign
    return 0.

def run_one(out:Path, mode:str, seconds=8.0):
    xml=xml_model()
    m=mj.MjModel.from_xml_string(xml)
    d=mj.MjData(m)
    dof=m.jnt_dofadr[m.joint('reaction_axis_STEPZ').id]
    bid=m.body('body').id
    d.qpos[:3]=(0,0,.07)
    d.qpos[3:7]=[math.cos(math.pi/4),0,math.sin(math.pi/4),0]
    mj.mj_forward(m,d)
    n=int(round(seconds/DT)); stride=50
    frames=[]; stats=[]; peak_w=0; min_tilt=180.; max_pen=0.; longest=.0; enter=None; warnings=[]; stepnum=0
    for k in range(n):
        zaxis=d.xmat[bid].reshape(3,3)[:,2]
        tilt=math.degrees(math.acos(float(np.clip(zaxis[2],-1.,1.))))
        min_tilt=min(min_tilt,tilt)
        if tilt<=10:
            if enter is None: enter=float(d.time)
            longest=max(longest,float(d.time)-enter)
        else: enter=None
        w=float(d.qvel[dof]);peak_w=max(peak_w,abs(w))
        u=0. if mode=='passive' else upstream_roll_input(float(d.time))
        if abs(w)>=OMEGA_CAP and u*w>0: u=0.
        d.ctrl[:]=[0.,u]
        if d.ncon: max_pen=max(max_pen, max(0,-min(float(c.dist) for c in d.contact))*1000)
        if k%stride==0 or k==n-1:
            # Captures actual mj_step state (t, qpos, tilt, reaction speed, torque, contact count)
            row=[round(float(d.time),3),*map(lambda v:round(float(v),6),d.qpos),round(tilt,3),round(w,3),round(u,4),int(d.ncon)]
            frames.append(row)
            stats.append({'t':round(float(d.time),3), 'qpos':[float(x) for x in d.qpos], 'qvel':[float(x) for x in d.qvel],
                          'contacts':int(d.ncon),'reaction_w':w,'control_torque':u,'tilt_deg':tilt})
        mj.mj_step(m,d)
        stepnum=k+1
        if any(item.number>0 for item in d.warning):
            warnings=[{'index':i,'count':int(s.number),'time':float(d.time)} for i,s in enumerate(d.warning) if s.number>0];break
        if not (np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()):
            warnings=[{'nonfinite':True,'time':float(d.time)}];break
    zz=d.xmat[bid].reshape(3,3)[:,2]
    final=math.degrees(math.acos(float(np.clip(zz[2],-1,1))))
    # Matching research PR #86 result schema, but distinct provenance tag.
    info={'status':'DIAGNOSTIC_COLLISION_PROXY_NO_VISUAL_CAD',
          'mode':mode,'source_illustration':'github research PR #86 source model without noncolliding STL geom',
          'simulated_duration_s':stepnum*DT,'start_roll_deg':90,'speed_cap_assumed':True,
          'numerical_warning': warnings or None,'meets_provisional_up_criterion':bool(final<=10 and longest>=1 and not warnings),
          'motor_limits_hardware_validated':False,'source_joint_label_parity_validated':False,
          'min_tilt_deg':round(min_tilt,3),'final_tilt_deg':round(final,3),'longest_upright_s':round(longest,3),
          'peak_reaction_speed_rad_s':round(peak_w,3),'max_contact_penetration_mm':round(max_pen,3),
          'physics_steps':stepnum,'model_xml_sha256':hashlib.sha256(xml.encode()).hexdigest(),
          'physics_engine':'MuJoCo','physics_version':mj.__version__, 'source_upstream':ORIGINAL_COMMIT}
    raw={'schema':'wheelbot-step-collision-only-raw/v1','info':info,'frames':frames,'samples':stats}
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(raw,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n')
    print(json.dumps({'result':str(out),'mode':mode,**{k:info[k] for k in ('physics_steps','simulated_duration_s','min_tilt_deg','final_tilt_deg','longest_upright_s','peak_reaction_speed_rad_s','max_contact_penetration_mm','numerical_warning','meets_provisional_up_criterion')}},ensure_ascii=False,indent=2))
    return raw

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=8)
    parser.add_argument('--mode',choices=('upstream','passive'),default='upstream')
    args=parser.parse_args();run_one(args.out,args.mode,args.seconds)
