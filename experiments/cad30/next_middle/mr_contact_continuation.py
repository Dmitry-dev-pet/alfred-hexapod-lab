#!/usr/bin/env python3
"""Reproducible saved-physical-state continuation of the four-foot CAD30 crawl.

No commands to physical robots, no direct MuJoCo root motion after initializing a previously
measured state. Trial uses position servo PD, correct per-tibia MuJoCo contacts.
A pass is a LOCAL step, not a complete gait or net forward progress.
"""
from __future__ import annotations
import argparse, json, sys, gzip, math
from pathlib import Path
import numpy as np
import mujoco as mj
from scipy.spatial import ConvexHull

LEGS=('ml','mr','bl','br','fl','fr'); WALK=('ml','mr','bl','br');DT=.002

def smoothstep(x):return x*x*(3-2*x)

def helpers(cad_root,repo_root):
    sys.path.insert(0,str(cad_root))
    sys.path.insert(0,str(repo_root/'experiments/cad30/pupper_baseline'))
    sys.path.insert(0,str(repo_root/'experiments/cad30/math_first_dynamic'))
    from math_first_four_crawl_v1 import ModelStudy
    from dynamic_replay_math_first import read_contact,model_com
    from hexlab.stand_geometry_v04 import load_legs, tip_world, geom_mesh_world_vertices
    return ModelStudy,read_contact,model_com,load_legs,tip_world,geom_mesh_world_vertices

class Trial:
    def __init__(self,cad_root,repo_root,checkpoint,mu=1.1,leg='mr'):
        self.cad_root=Path(cad_root);self.repo_root=Path(repo_root);self.checkpoint=Path(checkpoint)
        self.leg=leg;self.mu=mu
        self.ModelStudy,self.read_contact,self.model_com,self.load_legs,self.tip_world,self.mesh_verts=helpers(self.cad_root,self.repo_root)
        assert leg in WALK
        self.rest=tuple(k for k in WALK if k!=leg)
        self.xml=self.cad_root/'source/mjcf/scene_local30_nominal.xml'

    def make(self):
        m=mj.MjModel.from_xml_path(str(self.xml));m.opt.timestep=DT
        d=mj.MjData(m)
        if self.checkpoint.suffix=='.npz':
            with np.load(self.checkpoint) as p:
                d.qpos[:]=p['qpos'];d.qvel[:]=p['qvel'];goals=p['goals'].copy()
        else:
            state=json.loads(self.checkpoint.read_text());d.qpos[:]=state['qpos'];d.qvel[:]=state['qvel'];goals=np.asarray(state['motor_goals'])
        floor=int(m.geom('floor').id)
        geoms={int(m.geom(f'tibia{k}_collision_0').id):k for k in LEGS}
        for g in (*geoms,floor):m.geom_friction[g,0]=self.mu
        mj.mj_forward(m,d)
        jid=np.asarray(m.actuator_trnid[:,0],int);ja=np.asarray(m.jnt_qposadr[jid],int);va=np.asarray(m.jnt_dofadr[jid],int)
        assert len(ja)==20
        legs={l['suffix']:l for l in self.load_legs(m,d,mj)}
        return m,d,floor,geoms,ja,va,goals,legs

    def margin(self,xy,com):
        if not all(k in xy for k in self.rest):return float('-inf')
        e=ConvexHull([xy[k] for k in self.rest]).equations
        return float(np.min(-(e[:,:2]@com[:2]+e[:,2]))*1000)

    def run(self,step_mm=28.,clearance_mm=17.,swing_node_s=.22,pre_shift_xy_mm=(0.,0.),stance_press_mm=0.,stance_bias_z_mm=(0.,0.,0.),ml_servo_bias_rad=(0.,0.,0.),hold_s=.4,initial_hold_s=.2,kp=7.,kd=.32,save_raw=False,live_model_data=None):
        if live_model_data is None:
            m,d,floor,geoms,ja,va,initial_goals,legs=self.make()
        else:
            # Exactly the ongoing measured MuJoCo MjData. No reset, no state edits.
            m,d,initial_goals=live_model_data
            floor=int(m.geom('floor').id)
            geoms={int(m.geom(f'tibia{k}_collision_0').id):k for k in LEGS}
            jids=np.asarray(m.actuator_trnid[:,0],int)
            ja=np.asarray(m.jnt_qposadr[jids],int)
            va=np.asarray(m.jnt_dofadr[jids],int)
            legs={l['suffix']:l for l in self.load_legs(m,d,mj)}
            initial_goals=np.asarray(initial_goals,float).copy()
        original_world={k:self.tip_world(m,d,legs[k]).copy() for k in WALK}
        original_root=d.qpos[:3].copy();original_com=self.model_com(m,d).copy()
        original_geom=int(legs[self.leg]['geom'])
        initial_Fz,contactxy,_=self.read_contact(m,d,floor,geoms)
        initial_margin=self.margin(contactxy,original_com)
        ref_xy={k:self.tip_world(m,d,legs[k])[:2].copy() for k in self.rest}
        current_goals=initial_goals.copy()
        joints=[int(m.body(a+self.leg).jntadr[0]) for a in ('coxa','femur','tibia')]
        motor_idx=[i for i in range(m.nu) if int(m.actuator_trnid[i,0]) in joints]
        assert len(motor_idx)==3
        last_seen_fz=None;max_tau=0.;max_raw_tau=0.;max_q=0.;max_drift=0.;min_fz=float('inf');min_margin=float('inf');tripod_bad_frames=0;tripod_bad_longest=0;tripod_bad_run=0
        air_t=0.;mesh_air_t=0.;max_mesh=-float('inf');air_world=[];last_loaded_four_frac=[];num_steps=0
        failures=[];trace=[];first_foot=original_world[self.leg]
        def drive(target,seconds,phase):
            nonlocal num_steps,max_tau,max_raw_tau,max_q,max_drift,min_fz,min_margin,air_t,mesh_air_t,max_mesh,last_seen_fz,tripod_bad_frames,tripod_bad_longest,tripod_bad_run
            current=current_goals.copy();n=max(1,round(seconds/DT))
            for tick in range(n):
                fraction=smoothstep((tick+1)/n);goals=current+(target-current)*fraction
                raw=kp*(goals-d.qpos[ja])-kd*d.qvel[va]
                cmd=np.clip(raw,m.actuator_ctrlrange[:,0],m.actuator_ctrlrange[:,1])
                d.ctrl[:]=cmd;mj.mj_step(m,d)
                fz,xy,unknown=self.read_contact(m,d,floor,geoms)
                com=self.model_com(m,d)
                margin=self.margin(xy,com)
                max_tau=max(max_tau,float(np.max(np.abs(cmd[:18]))))
                max_raw_tau=max(max_raw_tau,float(np.max(np.abs(raw[:18]))))
                max_q=max(max_q,float(np.max(np.abs(d.qpos[ja[:18]]))))
                if phase.startswith('swing'):
                    min_fz=min(min_fz,*[float(fz[k]) for k in self.rest])
                    min_margin=min(min_margin,margin)
                    for k in self.rest:
                        max_drift=max(max_drift,float(np.linalg.norm(self.tip_world(m,d,legs[k])[:2]-ref_xy[k])*1000))
                    # Verify geometry+contact separately, not just desired q command.
                    zero_pairs=not any((int(c.geom1)==floor and int(c.geom2)==original_geom) or (int(c.geom2)==floor and int(c.geom1)==original_geom) for c in d.contact[:d.ncon])
                    toe=self.tip_world(m,d,legs[self.leg])
                    if zero_pairs:air_t+=DT
                    if zero_pairs and tick%10==0:
                        z=float(self.mesh_verts(m,d,original_geom)[:,2].min());max_mesh=max(max_mesh,z)
                        if z>.002:mesh_air_t+=DT*10;air_world.append(float(toe[1]))
                    if any(fz[k]<.2 for k in self.rest):
                        tripod_bad_frames+=1;tripod_bad_run+=1;tripod_bad_longest=max(tripod_bad_longest,tripod_bad_run)
                    else:tripod_bad_run=0
                    if any(fz[k]<.2 for k in self.rest) or margin<10:
                        if not failures:failures.append({'phase':phase,'frame':num_steps,'margin_mm':margin,'Fz_N':fz,'reason':'intended_tripod_loss'})
                if phase=='final_hold':
                    last_loaded_four_frac.append(all(fz[k]>=.2 for k in WALK))
                if max_raw_tau>.7+1e-6 and not any(x.get('reason')=='model_tau_limit' for x in failures):failures.append({'phase':phase,'frame':num_steps,'reason':'model_tau_limit'})
                if max_q>1.7+1e-6 and not any(x.get('reason')=='model_joint_limit' for x in failures):failures.append({'phase':phase,'frame':num_steps,'reason':'model_joint_limit'})
                if not np.all(np.isfinite(d.qpos)) or d.qpos[2]<.08:
                    if not failures:failures.append({'phase':phase,'frame':num_steps,'reason':'bad_root_state'})
                if save_raw:
                    trace.append({'time_s':float(d.time),'phase':phase,'qpos':d.qpos.copy().tolist(),'qvel':d.qvel.copy().tolist(),
                                  'motor_goal':goals.tolist(),'Fz_N':fz,'tripod_margin_mm':margin})
                num_steps+=1;last_seen_fz=fz
            current_goals[:]=target
        # Stability hold: keep previous actual (not desired q angles) motor targets.
        drive(current_goals.copy(),initial_hold_s,'initial_hold')
        shift=np.asarray(pre_shift_xy_mm,dtype=float)/1000
        ik_error=0
        if np.linalg.norm(shift)>1e-10:
            study=self.ModelStudy(self.xml,d.qpos.copy())
            base_toes={k:study.original_feet[k] for k in WALK}
            q,_errors=study.ik(d.qpos[:2]+shift,base_toes,d.qpos.copy(),anchored=WALK)
            ik_error=max(_errors.values())
            pre_goal=q[ja].copy();pre_goal[18:]=current_goals[18:]
            drive(pre_goal,1.0,'weight_transfer')
            drive(pre_goal,.25,'weight_transfer_hold')
        if stance_press_mm>0:
            # Tiny downward *target* adjustment for the physically measured stance toe.
            # Does not move root; model contacts and force distribution remain physical.
            study_press=self.ModelStudy(self.xml,d.qpos.copy())
            toe=study_press.original_feet['ml'].copy()
            toe[2]-=stance_press_mm/1000
            qpress,press_err=study_press.ik(d.qpos[:2],{'ml':toe},d.qpos.copy(),anchored=('ml',))
            leg_joints=[int(m.body(a+'ml').jntadr[0]) for a in ('coxa','femur','tibia')]
            press_indices=[i for i in range(m.nu) if int(m.actuator_trnid[i,0]) in leg_joints]
            press_goal=current_goals.copy();press_goal[press_indices]=qpress[ja[press_indices]]
            drive(press_goal,.65,'stance_press_ml')
            drive(press_goal,.20,'stance_press_hold')
        if np.max(np.abs(stance_bias_z_mm))>1e-10:
            study_bias=self.ModelStudy(self.xml,d.qpos.copy())
            goals_bias={}
            for k,dz in zip(self.rest,stance_bias_z_mm):
                goals_bias[k]=study_bias.original_feet[k].copy()
                goals_bias[k][2]+=dz/1000
            q_bias,bias_errors=study_bias.ik(d.qpos[:2],goals_bias,d.qpos.copy(),anchored=self.rest)
            bg=current_goals.copy()
            for leg in self.rest:
                js=[int(m.body(a+leg).jntadr[0]) for a in ('coxa','femur','tibia')]
                indices=[i for i in range(m.nu) if int(m.actuator_trnid[i,0]) in js]
                bg[indices]=q_bias[ja[indices]]
            drive(bg,.70,'tripod_force_rebalance')
            drive(bg,.20,'tripod_force_hold')
        if any(abs(x)>1e-10 for x in ml_servo_bias_rad):
            ml_js=[int(m.body(a+'ml').jntadr[0]) for a in ('coxa','femur','tibia')]
            ml_idx=[i for i in range(m.nu) if int(m.actuator_trnid[i,0]) in ml_js]
            # Each offset is a pure servo command, not a change of measured qpos.
            biased=current_goals.copy()
            for act,delta in zip(ml_idx,ml_servo_bias_rad):biased[act]+=delta
            drive(biased,.6,'ml_servo_force_bias')
            drive(biased,.2,'ml_servo_force_hold')
        start_swing_com=self.model_com(m,d).copy()
        start_fz,_,_=self.read_contact(m,d,floor,geoms)
        # Generate 9 IK waypoints for the requested leg while the three stance targets stay frozen.
        # Use actual joint state and measured toe as initial condition, avoid a kinematic teleport.
        study=self.ModelStudy(self.xml,d.qpos.copy())
        old=study.original_feet[self.leg].copy()
        seed=d.qpos.copy();goals=current_goals.copy();qerrors=[]
        for index in range(1,10):
            u=index/9.;toe_goal=old.copy()
            toe_goal[1]+=step_mm/1000*u
            toe_goal[2]+=clearance_mm/1000*math.sin(math.pi*u)
            q,err=study.ik(d.qpos[:2],{self.leg:toe_goal},seed,anchored=(self.leg,))
            seed=q.copy();qerrors.extend(err.values());goals[motor_idx]=q[ja[motor_idx]]
            drive(goals,swing_node_s,'swing_'+self.leg)
        drive(goals,hold_s,'final_hold')
        last_com=self.model_com(m,d)
        foot_tip=self.tip_world(m,d,legs[self.leg]);final_fz,_,_=self.read_contact(m,d,floor,geoms)
        result={'model':'FNK0052 CAD30 MuJoCo original geometry nominal m≈1.805kg','swing_leg':self.leg,'other_legs':self.rest,
                'mu':self.mu,'controller_PD_Kp_Kd':(kp,kd),'step_command_mm':step_mm,'clearance_command_mm':clearance_mm,'swing_node_s':swing_node_s,
                'pre_shift_xy_mm':list(pre_shift_xy_mm),'initial_hold_s':initial_hold_s,'stance_press_mm':stance_press_mm,'stance_bias_z_mm':list(stance_bias_z_mm),'ml_servo_bias_rad':list(ml_servo_bias_rad),'initial_Fz_N':initial_Fz,'initial_tripod_margin_mm':initial_margin,
                'start_swing_Fz_N':start_fz,'COM_y_move_during_next_step_mm':float((last_com[1]-original_com[1])*1000),
                'root_y_move_during_next_step_mm':float((d.qpos[1]-original_root[1])*1000),
                'physical_toe_forward_mm':float((foot_tip[1]-old[1])*1000),
                'in_air_toe_forward_mm':float((max(air_world)-min(air_world))*1000) if air_world else 0.,
                'zero_contact_tibia_total_s':air_t,'mesh_above_2mm_no_contact_sampled_s':mesh_air_t,
                'max_mesh_height_mm_sampled':max_mesh*1000,'min_fz_trio_N':min_fz,'min_margin_trio_mm':min_margin,'trio_bad_force_frames':tripod_bad_frames,'trio_max_contiguous_bad_frames':tripod_bad_longest,
                'max_stance_marker_drift_mm':max_drift,'max_ik_error_mm':max(qerrors or [ik_error]),
                'max_leg_tau_Nm':max_tau,'max_raw_tau_Nm':max_raw_tau,'max_leg_joint_rad':max_q,
                'last_hold_fraction_four_loaded':sum(last_loaded_four_frac)/max(1,len(last_loaded_four_frac)),
                'final_Fz_N':final_fz,'failures':failures,'physics_steps':num_steps,
                'final_qpos':d.qpos.copy().tolist(),'final_qvel':d.qvel.copy().tolist(),'final_motor_goals':current_goals.tolist(),
                'dt_s':DT,'no_root_teleport':True,'warning':'local single-step only; not full gait, hardware limits not validated'}
        result['accepted']=(not failures and min_fz>=.2 and min_margin>=10 and mesh_air_t>=.25 and
            result['physical_toe_forward_mm']>=20 and result['in_air_toe_forward_mm']>=10 and
            result['max_stance_marker_drift_mm']<5 and result['max_ik_error_mm']<2 and
            result['max_raw_tau_Nm']<=.7 and result['max_leg_joint_rad']<=1.7 and
            result['last_hold_fraction_four_loaded']>=.99)
        result['status']='LOCAL_NEXT_STEP_PASS_NOT_GAIT' if result['accepted'] else 'FAIL'
        return result,trace

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--cad-root',type=Path,required=True)
    p.add_argument('--repo-root',type=Path,default=Path(__file__).resolve().parents[3])
    p.add_argument('--checkpoint',type=Path,required=True)
    p.add_argument('--leg',default='mr');p.add_argument('--step-mm',type=float,default=28.)
    p.add_argument('--clearance-mm',type=float,default=17.);p.add_argument('--node-s',type=float,default=.22)
    p.add_argument('--shift-x-mm',type=float,default=0.);p.add_argument('--shift-y-mm',type=float,default=0.)
    p.add_argument('--kp',type=float,default=7.);p.add_argument('--kd',type=float,default=.32);p.add_argument('--initial-hold-s',type=float,default=.2);p.add_argument('--servo-coxa',type=float,default=0.);p.add_argument('--servo-femur',type=float,default=0.);p.add_argument('--servo-tibia',type=float,default=0.);p.add_argument('--bias-ml',type=float,default=0.);p.add_argument('--bias-bl',type=float,default=0.);p.add_argument('--bias-br',type=float,default=0.);p.add_argument('--press-ml-mm',type=float,default=0.);p.add_argument('--mu',type=float,default=1.1);p.add_argument('--out',type=Path,required=True);p.add_argument('--raw',action='store_true')
    args=p.parse_args()
    trial=Trial(args.cad_root,args.repo_root,args.checkpoint,mu=args.mu,leg=args.leg)
    result,trace=trial.run(step_mm=args.step_mm,clearance_mm=args.clearance_mm,swing_node_s=args.node_s,
                           pre_shift_xy_mm=(args.shift_x_mm,args.shift_y_mm),stance_press_mm=args.press_ml_mm,stance_bias_z_mm=(args.bias_ml,args.bias_bl,args.bias_br),ml_servo_bias_rad=(args.servo_coxa,args.servo_femur,args.servo_tibia),initial_hold_s=args.initial_hold_s,kp=args.kp,kd=args.kd,save_raw=args.raw)
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    if args.raw:
        with gzip.open(args.out.with_name(args.out.stem+'_2ms.json.gz'),'wt') as f:
            json.dump({'schema':'cad30-contiguous-next-foot-2ms/1','frames':trace},f)
    print(json.dumps({k:v for k,v in result.items() if k not in ('final_qpos','final_qvel','final_motor_goals')},indent=2,ensure_ascii=False))
if __name__=='__main__':main()
