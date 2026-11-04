#!/usr/bin/env python3
"""Export measured Drake/IDTO Mini Cheetah poses and contact forces for Hexapod viewer.

Uses real 50ms recorded states/forces and original upstream URDF joint origins;
only 3D appearance is an explicitly labeled schematic, never fake movement.
Does not redistribute TRI/MIT OBJ meshes or run a new physics simulation.
"""
import argparse, csv, json, math, hashlib, xml.etree.ElementTree as ET
from pathlib import Path
from scipy.spatial.transform import Rotation as R
import numpy as np
parser=argparse.ArgumentParser(description='Export genuine Drake Mini Cheetah IDTO recording as 3D schematic replay')
parser.add_argument('--urdf',type=Path,required=True)
parser.add_argument('--states',type=Path,required=True)
parser.add_argument('--contacts',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
model=args.urdf
text=model.read_text().replace('<robot name="mini_cheetah"','<robot xmlns:drake="https://drake.mit.edu" name="mini_cheetah"')
root=ET.fromstring(text)
joints={j.find('child').attrib['link']:j for j in root.findall('joint')}
legs=['fl','fr','hl','hr']

def dof_node(parent_pos,parent_rot,child_name,ang):
    j=joints[child_name]
    o=j.find('origin').attrib
    xyz=np.array([float(t) for t in o.get('xyz','0 0 0').split()]);rpy=np.array([float(t) for t in o.get('rpy','0 0 0').split()]);
    pos=parent_pos+parent_rot.apply(xyz)
    newrot=parent_rot*R.from_euler('xyz',rpy)
    if j.attrib.get('type')!='fixed':
        axis=np.array([float(t) for t in j.find('axis').attrib['xyz'].split()]);newrot=newrot*R.from_rotvec(axis*ang)
    return pos,newrot

def parse_one(path):
    with open(path,newline='') as f:return list(csv.DictReader(f))
states=parse_one(args.states)
contacts=parse_one(args.contacts)
assert len(states)==201 and len(contacts)==201
frames=[]
for state, force in zip(states, contacts):
    t=float(state['t'])
    assert abs(t-float(force['t']))<1e-6
    s=np.array([float(state['state'+str(i)]) for i in range(37)])
    quat=s[:4] # wxyz
    p=s[4:7]
    rot=R.from_quat([quat[1],quat[2],quat[3],quat[0]])
    joints_pos=[]
    for i,leg in enumerate(legs):
        hip,hrot=dof_node(p,rot,'abduct_'+leg,s[7+3*i]);thigh,trot=dof_node(hip,hrot,'thigh_'+leg,s[8+3*i]);knee,krot=dof_node(thigh,trot,'shank_'+leg,s[9+3*i]);foot,_=dof_node(knee,krot,{'fl':'LF_FOOT','fr':'RF_FOOT','hl':'LH_FOOT','hr':'RH_FOOT'}[leg],0)
        joints_pos.extend(list(hip)+list(thigh)+list(knee)+list(foot))
    forces=[float(force[k+'_fz_N']) for k in ('lf','rf','lh','rh')]
    row=[*p,*quat,*joints_pos,*forces,float(force['max_pair_slip_mps'])]
    assert len(row)==60 and all(math.isfinite(x) for x in row)
    frames.append([round(float(x),6) for x in row])
check=frames[0]
print('initial base',check[:7],'initial feet z',[round(check[7+i*12+11],3) for i in range(4)])
assert all(-.05 < check[7+i*12+11] < .1 for i in range(4))
actual_counts={2:0,3:0,4:0,1:0,0:0}
for row in frames:
    n=sum(row[55+i]>0.2 for i in range(4))
    actual_counts[n]=actual_counts.get(n,0)+1
print('actual loads',actual_counts,'end x',frames[-1][0],'length',len(frames))
assert actual_counts[2]>100
name='cheetah-idto-10s'
data={
 'format':'mini-cheetah-idto-physical-replay-v1',
 'version':1,
 'title':'Mini Cheetah · IDTO MPC',
 'source':{
   'robot':'Mini Cheetah quadruped',
   'physics':'Drake 1.30.0',
   'controller':'TRI IDTO, real closed-loop MPC',
   'idto_commit':'de0629c7811aa9b330e56c4385629005b09495f0',
   'recording':'2026-11-04 original Mini Cheetah run, instrumented for state and foot Fz',
   'states_sha256':hashlib.sha256(args.states.read_bytes()).hexdigest(),
   'contacts_sha256':hashlib.sha256(args.contacts.read_bytes()).hexdigest(),
   'urdf_sha256':hashlib.sha256(model.read_bytes()).hexdigest(),
   'metrics_source':'https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/pull/33'
 },
 'visualization':'Kinematic skeleton reconstructed from original URDF and measured Drake qpos. Not the original OBJ meshes; no simulated/fake motion.',
 'leg_order':['FL','FR','HL','HR'],
 'coord':'Drake world X forward, Z up. Base xyz + quaternion wxyz.',
 'duration_s':10.,'sample_interval_s':.05,'sample_count':201,
 'stride':60,
 'frame_layout':'base xyz (3), base quaternion wxyz (4), for each leg [hip, upper joint, knee, toe] xyz (12), 4 actual foot contact Fz Newton, max contact slip m/s (1)',
 'loaded_threshold_N':0.2,
 'summary':{'forward_m':round(frames[-1][0]-frames[0][0],6),'contact_counts':{str(k):v for k,v in sorted(actual_counts.items())},'gait_confirmed':'dynamic MPC with predominantly two loaded legs','not_freenove':True},
 'limitations':['Recorded physical states/forces every 50ms: no guarantee of continuous contacts between samples',
 'Measured base and foot loads are actual; mesh shape displayed is schematic reconstructed from URDF kinematics',
 'Mass distribution, contacts and actuator strength do not equal the Freenove robot',
 'This baseline alone is not proof of guaranteed non-slip gait'],
 'frames':frames
}
pout=args.output;pout.parent.mkdir(parents=True,exist_ok=True);pout.write_text(json.dumps(data,separators=(',',':'),ensure_ascii=False)+'\n')
print('data bytes',pout.stat().st_size, 'sha256',hashlib.sha256(pout.read_bytes()).hexdigest())
