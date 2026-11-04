"""Independent CAD30 MuJoCo observability/actuation boundary for future ports.

This IS NOT neuroWalknet. Its Hector-facing BioFlexBus signals and neuronal
activation dynamics have not been ported. We only expose raw MuJoCo quantities
with explicit units, and a bounded joint-torque command function.

Avoid importing GPL-3.0 upstream neuroWalknet controller code into this file.
"""
from __future__ import annotations

import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT=next((p for p in Path(__file__).resolve().parents if (p/'hexlab').is_dir()),None)
if ROOT is None:
    raise RuntimeError('Expected existing hexlab/ Python package in CAD30 tree')
for p in (str(ROOT),str(Path(__file__).resolve().parent.parent)):
    if p not in sys.path:sys.path.insert(0,p)
from hexlab.stand_geometry_v04 import load_legs, tip_world
from hexlab.step_one_leg_v04 import floor_contacts
from failure_one_leg_cad30 import forces_by_leg

CAD_LEG_CODES=('fl','ml','bl','br','mr','fr')
# Original CAD is mirrored against Freenove servo channel conventions.
# Real wiring must still be verified before operating hardware.


def read_observation(model: mujoco.MjModel, data: mujoco.MjData, *, legs=None) -> dict:
    """Return read-only physically simulated inputs with units for six local legs."""
    if legs is None:
        legs=load_legs(model,data,mujoco)
    names={leg['suffix'] for leg in legs}
    if names!=set(CAD_LEG_CODES):
        raise ValueError('Expected exactly six CAD leg names')
    contact=floor_contacts(model,data,mujoco)
    forces=forces_by_leg(model,data)
    legs_out={}
    for leg in legs:
        code=leg['suffix']
        legs_out[code]={
            'joint_angles_rad':np.asarray(data.qpos[leg['qadrs']],float).tolist(),
            'joint_rates_rad_per_s':np.asarray(data.qvel[leg['dadrs']],float).tolist(),
            'toe_marker_world_m':tip_world(model,data,leg).tolist(),
            'tibia_floor_contact':code in contact,
            'tibia_normal_force_N':float(forces[code]),
        }
    root=int(model.body('base_link_2').id)
    orientation=np.asarray(data.xmat[root]).reshape(3,3)
    return {
        'units':'rad,rad/s,m,N,s',
        'data_source':'MuJoCo CAD30 estimated physics; NOT measured hardware',
        'time_s':float(data.time),
        'base_world_xyz_m':np.asarray(data.xpos[root],float).tolist(),
        'body_orientation_world_3x3':orientation.tolist(),
        'legs':legs_out,
    }


def command_bounded_torques(model: mujoco.MjModel, data: mujoco.MjData,
                            commands_nm: dict[str, list[float]], *, legs=None) -> dict:
    """Apply ordered per-leg 3-axis joint torques, clipping to MuJoCo limits.

    This is merely an actuator boundary: it does not run PID, neuroWalknet,
    amplitude/velocity conversion or electrical motor models.
    """
    if legs is None:
        legs=load_legs(model,data,mujoco)
    if set(commands_nm)!=set(CAD_LEG_CODES):
        raise ValueError('All six named legs required, including any disabled legs')
    motors=np.asarray(model.actuator_trnid[:,0],int)
    motor_index={int(j):i for i,j in enumerate(motors)}
    output=np.zeros(model.nu,dtype=float)
    for leg in legs:
        code=leg['suffix']
        desired=np.asarray(commands_nm[code],dtype=float)
        if desired.shape!=(3,) or not np.all(np.isfinite(desired)):
            raise ValueError(f'Expected three finite torques for {code}')
        for jid,torque in zip(leg['jids'],desired):
            output[motor_index[int(jid)]]=torque
    clamped=np.clip(output,model.actuator_ctrlrange[:,0],model.actuator_ctrlrange[:,1])
    data.ctrl[:]=clamped
    return {'saturated_motor_count':int(np.sum(np.abs(clamped-output)>1e-12)),
            'control_nm':clamped.tolist()}