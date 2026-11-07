"""Mac-friendly MuJoCo CPU training model of the pinned Stanford Pupper v3.

This is deliberately NOT Stanford's full MJLab/MuJoCo-Warp training environment.
It retains the pinned physical inertias, contacts and joint names, adding the
same simple 12 position servos as the independently reproduced CPU baseline.
No root-position/velocity scripting. No vendor pretrained weights are trained.
"""
from __future__ import annotations

from collections import deque
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
ORIGINAL_XML = HERE.parent / 'upstream' / 'pupper_v3.xml'
XML_BLOB = '7771e176c42b3aed1d48cc62434771b0fceb7c8c'
DEFAULT = np.array([.26, 0, -.52, -.26, 0, .52,
                    .26, 0, -.52, -.26, 0, .52], dtype=np.float64)
JOINTS = [f'leg_{a}_{s}_{i}' for a,s in
          [('front','r'),('front','l'),('back','r'),('back','l')]
          for i in (1,2,3)]
FOOT_GEOMS = [f'leg_{a}_{s}_3_collision' for a,s in
              [('front','r'),('front','l'),('back','r'),('back','l')]]
FOOT_SITES = [f'leg_{a}_{s}_3_foot_site' for a,s in
              [('front','r'),('front','l'),('back','r'),('back','l')]]
DT = .004
DECIMATION = 5
TORQUE = 3.0


def git_blob_sha(data:bytes) -> str:
    return hashlib.sha1(b'blob '+str(len(data)).encode()+b'\x00'+data).hexdigest()


def load_cpu_model():
    original = ORIGINAL_XML.read_bytes()
    if git_blob_sha(original) != XML_BLOB:
        raise ValueError('Stanford original MJCF SHA-1 mismatch; refuse training')
    root = ET.fromstring(original)
    if root.attrib.get('model')!='pupper_v3':
        raise ValueError('Not the source Pupper v3 model')
    nvisual=0
    for body in root.findall('.//body'):
        for geom in list(body.findall('geom')):
            if geom.get('type')=='mesh':
                if geom.get('density')!='0' or geom.get('class') is not None:
                    raise ValueError('Cannot strip collision/inertial mesh')
                body.remove(geom)
                nvisual+=1
    if nvisual!=13 or len(root.find('asset').findall('mesh'))!=13:
        raise ValueError('Unexpected source visual assets')
    root.remove(root.find('asset'))
    root.find('compiler').attrib.pop('meshdir',None)
    option=ET.Element('option',{
        'timestep':str(DT),'integrator':'Euler','cone':'pyramidal',
        'impratio':'10','iterations':'2','ls_iterations':'5'})
    ET.SubElement(option,'flag',{'eulerdamp':'disable'})
    root.insert(1,option)
    world=root.find('worldbody')
    ET.SubElement(world,'geom',{
        'name':'ground_cpu_preflight','type':'plane','size':'5 5 .1',
        'friction':'0.8 0.02 0.01','contype':'1','conaffinity':'1',
        'condim':'3','group':'0'})
    act=ET.SubElement(root,'actuator')
    for name in JOINTS:
        ET.SubElement(act,'position',{'name':'servo_'+name,'joint':name,
            'kp':'5.5','kv':'0.25','forcelimited':'true','forcerange':'-3 3'})
    model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
    if (model.nq,model.nv,model.nu)!=(19,18,12) or not np.isclose(sum(model.body_mass),3.17,atol=1e-6):
        raise ValueError('Stanford Pupper physical model dimensions/mass mismatch')
    for geom in FOOT_GEOMS:
        if mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,geom)<0:raise ValueError(geom)
    for site in FOOT_SITES:
        if mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,site)<0:raise ValueError(site)
    return model


class PupperCPU:
    """True MuJoCo physics at 250Hz; 50Hz control with 4-tick servo delay."""

    def __init__(self, delay_steps:int=4):
        if not 0<=delay_steps<=20:raise ValueError('delay outside bounded scope')
        self.model=load_cpu_model()
        self.data=mujoco.MjData(self.model)
        self.base_id=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_BODY,'base_link')
        self.plane_id=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_GEOM,'ground_cpu_preflight')
        self.foot_ids=[mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_GEOM,n) for n in FOOT_GEOMS]
        self.site_ids=[mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_SITE,n) for n in FOOT_SITES]
        self.delay_steps=delay_steps
        self._command=np.zeros(3,dtype=np.float32)
        self.history=np.zeros((20,48),dtype=np.float32)
        self.action=np.zeros(12,dtype=np.float32)
        self.queue=deque([],maxlen=delay_steps+1)
        self.control_steps=0
        self.joint_limits=np.asarray([self.model.jnt_range[
            mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_JOINT,n)]
            for n in JOINTS],dtype=np.float64)
        self.reset()

    def set_command(self,command):
        arr=np.asarray(command,dtype=np.float32)
        if arr.shape!=(3,) or not np.isfinite(arr).all():raise ValueError('invalid 3-axis command')
        if abs(arr[0])>.30 or abs(arr[1])>.20 or abs(arr[2])>.60:
            raise ValueError('outside bounded training command limits')
        self._command=arr.copy()
        self.history[0]=self.frame()

    def reset(self,command=(0,0,0)):
        mujoco.mj_resetData(self.model,self.data)
        self.data.qpos[:3]=[0.,0.,.18]
        self.data.qpos[3:7]=[1.,0.,0.,0.]
        self.data.qpos[7:19]=DEFAULT
        self.data.ctrl[:]=DEFAULT
        self.action=np.zeros(12,dtype=np.float32)
        self._command=np.asarray(command,dtype=np.float32)
        self.control_steps=0
        self.queue=deque([DEFAULT.copy() for _ in range(self.delay_steps+1)],maxlen=self.delay_steps+1)
        self.history=np.zeros((20,48),dtype=np.float32)
        self.history[:,5]=-1.
        mujoco.mj_forward(self.model,self.data)
        self.history[0]=self.frame()
        return self.observation()

    def local_velocity(self):
        vel=np.zeros(6,dtype=np.float64)
        mujoco.mj_objectVelocity(self.model,self.data,mujoco.mjtObj.mjOBJ_BODY,self.base_id,vel,1)
        return vel  # angular xyz, linear xyz, all body-local

    def frame(self):
        rot=self.data.xmat[self.base_id].reshape(3,3)
        gravity=rot.T@np.array([0.,0.,-1.])
        ang=self.local_velocity()[:3]
        return np.concatenate((ang,gravity,self._command,[0.,0.,1.],
                 self.data.qpos[7:19]-DEFAULT,self.action,np.zeros(12))).astype(np.float32)

    def observation(self):
        return np.clip(self.history.reshape(960),-10,10).astype(np.float32)

    def contact_forces(self):
        f=np.zeros(4,dtype=np.float64)
        out=np.zeros(6,dtype=np.float64)
        for i in range(self.data.ncon):
            c=self.data.contact[i]
            if self.plane_id not in (c.geom1,c.geom2):continue
            other=c.geom2 if c.geom1==self.plane_id else c.geom1
            if other in self.foot_ids:
                mujoco.mj_contactForce(self.model,self.data,i,out)
                f[self.foot_ids.index(other)]+=max(0.,out[0])
        return f

    def metrics(self):
        vel=self.local_velocity()
        tilt=math.acos(float(np.clip(self.data.xmat[self.base_id][8],-1.,1.)))
        z=float(self.data.qpos[2])
        return {'vel_xy':vel[3:5].copy(),'yaw_rate':float(vel[2]),
                'vertical_rate':float(vel[5]),'tilt_rad':tilt,'height_m':z,
                'torque_norm':float(np.mean(np.square(self.data.actuator_force/TORQUE))),
                'feet_loaded':int(np.count_nonzero(self.contact_forces()>.2)),
                'position_world':self.data.qpos[:3].copy(),
                'actuator_max_nm':float(np.max(abs(self.data.actuator_force)))}

    def step(self,action):
        command=np.asarray(action,dtype=np.float32)
        if command.shape!=(12,) or not np.isfinite(command).all():
            raise ValueError('invalid 12-dimensional action')
        command=np.clip(command,-1,1)
        prev=self.action.copy()
        target=DEFAULT+0.75*command
        # Resolve actual source joint limits in explicit source order.
        target=np.clip(target,self.joint_limits[:,0],self.joint_limits[:,1])
        for _ in range(DECIMATION):
            self.queue.append(target.copy())
            self.data.ctrl[:]=self.queue[0]
            mujoco.mj_step(self.model,self.data)
        self.action=command.copy()
        self.control_steps+=1
        self.history[1:]=self.history[:-1].copy()
        self.history[0]=self.frame()
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise FloatingPointError('MuJoCo generated non-finite physical state')
        m=self.metrics()
        m['delta_action_l2']=float(np.mean((command-prev)**2))
        m['fell']=(m['tilt_rad']>math.radians(35) or m['height_m']<.06)
        return self.observation(),m
