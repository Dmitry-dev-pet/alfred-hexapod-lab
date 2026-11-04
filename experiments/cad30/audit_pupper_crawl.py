"""Audit an UNMODIFIED Stanford Pupper gait scheduler on FNK0052 CAD30 contacts.

No gait or motor commands are executed: this only checks the actual saved
four-leg geometry against a four-beat contact schedule. Scheduler copied with
Stanford Student Robotics MIT license and attribution to a fixed upstream ref.
The original CAD assets must be restored locally; they are not distributed.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Sequence

import numpy as np
from scipy.spatial import ConvexHull

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE/'vendor'/'stanford_pupper'))
from Gaits import GaitController

LEGS=('ml','mr','bl','br')
PUPPER_LEG_MAP={'front_left':'ml','front_right':'mr','hind_left':'bl','hind_right':'br'}
# Deterministic illustrative crawl order, NOT claimed optimal for this robot.
DEFAULT_SWING_ORDER=('bl','mr','br','ml')


class SlowCrawlConfig:
    """Only config: original upstream GaitController is not altered.

    This 8-phase contact schedule has four all-stance windows and four
    one-leg swings; no claim is made that the original Pupper ships this gait.
    """
    def __init__(self, swing_order:Sequence[str]=DEFAULT_SWING_ORDER,
                 overlap_ticks:int=60,swing_ticks:int=40):
        if len(swing_order)!=4 or set(swing_order)!=set(LEGS):
            raise ValueError('Swing order must contain each four stance legs once')
        if overlap_ticks<1 or swing_ticks<1:
            raise ValueError('Each phase must have a positive duration')
        self.swing_order=tuple(swing_order)
        self.num_phases=8
        self.swing_ticks=int(swing_ticks)
        self.overlap_ticks=int(overlap_ticks)
        self.phase_ticks=np.array([self.overlap_ticks,self.swing_ticks]*4,dtype=int)
        self.phase_length=int(self.phase_ticks.sum())
        self.contact_phases=np.ones((4,8),dtype=int)
        for i,leg in enumerate(self.swing_order):
            self.contact_phases[LEGS.index(leg),2*i+1]=0


def signed_margin_m(points:np.ndarray,com_xy:np.ndarray)->float:
    if len(points)<3:return float('-inf')
    eq=ConvexHull(np.asarray(points,float)[:,:2]).equations
    return float(np.min(-(eq[:,:2]@np.asarray(com_xy,float)+eq[:,2])))


def load_contact_pose(model_xml:Path,pose_json:Path):
    """Determine real MuJoCo geometry at archived pose (not invented toes)."""
    import mujoco
    model=mujoco.MjModel.from_xml_path(str(model_xml))
    data=mujoco.MjData(model)
    pose=json.loads(pose_json.read_text())
    q=np.asarray(pose['qpos'],float)
    if q.shape!=(model.nq,):raise ValueError('pose qpos does not match MJCF')
    data.qpos[:]=q;data.qvel[:]=0.
    mujoco.mj_forward(model,data)
    com=(data.xipos*model.body_mass[:,None]).sum(axis=0)/model.body_mass.sum()
    floor=int(model.geom('floor').id)
    foot_geom={int(model.geom(f'tibia{leg}_collision_0').id):leg for leg in (*LEGS,'fl','fr')}
    contacts={leg:[] for leg in (*LEGS,'fl','fr')}
    for j in range(data.ncon):
        c=data.contact[j];g1,g2=int(c.geom1),int(c.geom2)
        other=g2 if g1==floor else g1 if g2==floor else -1
        if other in foot_geom:
            contacts[foot_geom[other]].append(np.asarray(c.pos).copy())
    if any(not contacts[leg] for leg in LEGS):
        raise ValueError('Archived pose has missing four-ground leg contact')
    if contacts['fl'] or contacts['fr']:
        raise ValueError('Front manipulator legs unexpectedly contacting ground')
    return (com,np.array([np.mean(contacts[leg],axis=0) for leg in LEGS]),
            {leg:len(contacts[leg]) for leg in contacts},
            {'mass_kg':float(model.body_mass.sum()),'nmesh':int(model.nmesh),
             'mujoco_version':mujoco.__version__,'nq':int(model.nq)})


def audit(model_xml:Path,pose_json:Path,margin_threshold_m:float=.005):
    cfg=SlowCrawlConfig();schedule=GaitController(cfg)
    com,feet,counts,physics=load_contact_pose(model_xml,pose_json)
    margins={leg:round(1e3*signed_margin_m(feet[[i for i,x in enumerate(LEGS) if x!=leg]],com[:2]),5) for leg in LEGS}
    schedule_phases=[]
    for phase in range(cfg.num_phases):
        tick=int(cfg.phase_ticks[:phase].sum())
        cs=np.asarray(schedule.contacts(tick),int)
        absent=[leg for leg,c in zip(LEGS,cs) if not c]
        schedule_phases.append({'phase':phase,'tick':tick,'swing':absent,'support_count':int(cs.sum()),
                                'static_5mm_allowed_at_initial_pose':bool(all(margins[x]>=margin_threshold_m*1000 for x in absent))})
    assert all(p['support_count']>=3 for p in schedule_phases)
    return {
        'schema':'pupper_scheduler_cad30_geometry_audit/1',
        'status':'SCHEDULER_COMPATIBLE; STATIC_SUPPORT_NOT_READY',
        'source_upstream':'https://github.com/stanfordroboticsclub/StanfordQuadruped/blob/dc0c5e2089cf2c0aac3c59982a4f140c31803175/src/Gaits.py',
        'source_license':'MIT, Stanford Student Robotics 2020',
        'source_unmodified':True,
        'model':physics,
        'four_walk_legs':list(LEGS),
        'two_front_manipulator_legs':['fl','fr'],
        'pupper_semantic_mapping':PUPPER_LEG_MAP,
        'derived_from_recorded_pose':str(pose_json.name),
        'COM_world_xyz_m':np.round(com,9).tolist(),
        'toe_contact_centroid_world_xyz_m':{leg:np.round(feet[i],9).tolist() for i,leg in enumerate(LEGS)},
        'contact_points_per_leg':counts,
        'phase_count':cfg.num_phases,
        'swing_leg_order':list(cfg.swing_order),
        'phase_ticks':cfg.phase_ticks.tolist(),
        'minimum_supporting_legs_by_schedule':min(p['support_count'] for p in schedule_phases),
        'scheduler_phases':schedule_phases,
        'static_leave_one_leg_out_margin_mm':margins,
        'static_liftoff_feasible_at_fixed_COM_with_5mm_margin':{leg:(v>=5.0) for leg,v in margins.items()},
        'hardware_motion_approved':False,
        'dynamic_walk_achieved':False,
        'notes':['Pupper upstream default is a two-at-once diagonal trot and is NOT used here.',
                 'This audit changes only the contact phase configuration; it does not automatically add COM weight shifting.',
                 'Static point-in-support-triangle is necessary for slow statically stable steps, but NOT a motor/force feasibility proof.',
                 'Actual MuJoCo contact positions are averaged geometrically (not weighted by Fz).',
                 'No swing motions, contact force trajectories, servo output, or closed-loop walking were run.',
                 'The archived state descends from an earlier six-to-four transition that did not satisfy all provisional joint limits.']
    }


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--model',type=Path,required=True)
    p.add_argument('--pose',type=Path,required=True)
    p.add_argument('--out',type=Path)
    args=p.parse_args()
    result=audit(args.model,args.pose)
    data=json.dumps(result,indent=2,ensure_ascii=False)+'\n'
    if args.out:args.out.parent.mkdir(parents=True,exist_ok=True);args.out.write_text(data)
    print(data)

if __name__=='__main__':main()