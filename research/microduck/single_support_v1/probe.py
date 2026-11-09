#!/usr/bin/env python3
"""Physical, motor-only weight transfer and single-support prerequisite study.

The kinematic planner may set positions ONLY in its separate scratch MjData.
The physical MjData is initialized once and then evolves exclusively by mj_step.
This is an experimental task controller, not a newly learned ONNX capability.
"""
from __future__ import annotations
import argparse, contextlib, hashlib, io, json, math, sys
from pathlib import Path
import mujoco as mj
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'step_goal_v1'))
import tune_recovery as base

DT, CDT = .0005, .02

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def smooth(x):
    u = float(np.clip(x, 0, 1)); return u*u*(3-2*u)

class Planner:
    def __init__(self, m, physical, sq, bq, feet, margin=.07, support='left', free_swing=False, head=False):
        self.m, self.d = m, mj.MjData(m)
        self.q0 = physical.qpos.copy()
        self.q0[sq] += self.q0[bq]
        self.q0[bq] = 0
        self.sq, self.bq, self.feet = sq, bq, feet
        self.support,self.free_swing=support,free_swing
        self.legs = np.array([i for i, j in enumerate(m.actuator_trnid[:, 0])
                              if head or 'left_' in mj.mj_id2name(m, mj.mjtObj.mjOBJ_JOINT, int(j))
                              or 'right_' in mj.mj_id2name(m, mj.mjtObj.mjOBJ_JOINT, int(j))])
        self.joints = m.actuator_trnid[self.legs, 0]
        self.lo = m.jnt_range[self.joints, 0] + margin
        self.hi = m.jnt_range[self.joints, 1] - margin
        self.r0 = Rotation.from_quat(self.q0[3:7][[1,2,3,0]]).as_rotvec()
        self.x0 = np.r_[self.q0[:3], self.r0, self.q0[sq[self.legs]]]
        self.x = self.x0.copy()
        self.lower = np.r_[self.q0[:2]-.13, .065, [-.65,-.65,-.5], self.lo]
        self.upper = np.r_[self.q0[:2]+.13, .18, [.65,.65,.5], self.hi]
        self.assign(self.x)
        self.pos = {s:self.d.geom_xpos[g].copy() for s,g in feet.items()}
        self.rot = {s:self.d.geom_xmat[g].reshape(3,3).copy() for s,g in feet.items()}
        self.com0 = self.d.subtree_com[1].copy()
    def assign(self, x):
        self.d.qpos[:] = self.q0
        self.d.qpos[:3] = x[:3]
        self.d.qpos[3:7] = Rotation.from_rotvec(x[3:6]).as_quat()[[3,0,1,2]]
        self.d.qpos[self.sq[self.legs]] = x[6:]
        mj.mj_forward(self.m, self.d)
    def solve(self, comxy, footpos):
        def residual(x):
            self.assign(x); r=[]
            for s,g in self.feet.items():
                r.extend(500*(self.d.geom_xpos[g]-footpos[s]))
                relative = self.rot[s].T @ self.d.geom_xmat[g].reshape(3,3)
                if self.free_swing:
                    rv=Rotation.from_matrix(self.d.geom_xmat[g].reshape(3,3)@self.rot[s].T).as_rotvec()
                    r.extend(np.array([50,50,50] if s==self.support else [.3,50,50])*rv)
                else:r.extend(5*Rotation.from_matrix(relative).as_rotvec())
            r.extend(500*(self.d.subtree_com[1,:2]-comxy))
            r.append(15*(x[2]-self.x0[2]))
            r.extend(.12*(x[3:6]-self.x0[3:6]))
            r.extend((.2 if self.free_swing else .025)*(x[6:]-self.x0[6:]))
            return np.array(r)
        sol=least_squares(residual, np.clip(self.x,self.lower+1e-8,self.upper-1e-8),
                          bounds=(self.lower,self.upper), max_nfev=80,
                          ftol=1e-7,xtol=1e-7,gtol=1e-7)
        self.x=sol.x; self.assign(self.x)
        q=self.d.qpos[self.sq].copy()
        errors={'foot_position_error_m':{s:float(np.linalg.norm(self.d.geom_xpos[g]-footpos[s])) for s,g in self.feet.items()},
                'com_xy_error_m':float(np.linalg.norm(self.d.subtree_com[1,:2]-comxy)),
                'foot_orientation_error_deg':{s:float(np.linalg.norm(Rotation.from_matrix(self.rot[s].T@self.d.geom_xmat[g].reshape(3,3)).as_rotvec())*180/np.pi) for s,g in self.feet.items()},
                'least_squares_success':bool(sol.success),'planned_root_pose':self.d.qpos[:7].tolist()}
        return q,errors

    def gravity_offset(self, ctrl, single=True):
        """Static gravity feedforward converted to position error of unchanged BAM.

        This predicts a feasible support wrench, it does not apply any force to
        the physical data. All actuation still goes through BAM motor targets.
        """
        m,d=self.m,self.d
        mg=float(m.body_mass.sum()*abs(m.opt.gravity[2]));external=np.zeros(m.nv)
        for s,g in self.feet.items():
            if single:weight=float(s==self.support)
            else:
                ly=self.pos['left'][1];ry=self.pos['right'][1]
                left=float(np.clip((d.subtree_com[1,1]-ry)/(ly-ry),0,1))
                weight=left if s=='left' else 1-left
            jp,jr=np.zeros((3,m.nv)),np.zeros((3,m.nv))
            pos=d.geom_xpos[g];mj.mj_jac(m,d,jp,jr,pos,int(m.geom_bodyid[g]))
            force=np.array([0.,0.,mg*weight])
            moment=np.cross(d.subtree_com[1]-pos,force)
            external+=jp.T@force+jr.T@moment
        tau=(d.qfrc_bias-external)[m.jnt_dofadr[m.actuator_trnid[:,0]]]
        act=ctrl.model.actuator
        gain=act.error_gain*act.kp*act.vin*ctrl.model.kt.value/ctrl.model.R.value
        return np.clip(tau/gain,-.5,.5)

def state(m,d,feet,floor,step):
    loads={s:0. for s in feet};top={s:0. for s in feet};other_load=0.;contacts=[]
    for i,ct in enumerate(d.contact):
        pair=(int(ct.geom1),int(ct.geom2));w=np.zeros(6);mj.mj_contactForce(m,d,i,w)
        env=floor in pair or step>=0 and step in pair
        if env:
            other=pair[1] if pair[0] in [floor,step] else pair[0]
            if other not in feet.values():other_load+=max(0.,float(w[0]))
            if abs(ct.frame[2])>.85:
                for s,g in feet.items():
                    if g in pair:
                        loads[s]+=max(0.,float(w[0]))
                        if step>=0 and step in pair:top[s]+=max(0.,float(w[0]))
        contacts.append({'geoms':list(pair),'force_n':float(w[0]),'distance_m':float(ct.dist),
                         'pos':ct.pos.tolist(),'normal':ct.frame[:3].tolist()})
    bounds={s:base.E.foot_bounds(m,d,g) for s,g in feet.items()}
    return {'foot_loads_n':loads,'step_loads_n':top,'nonfoot_environment_load_n':other_load,
            'foot_bounds':bounds,'com_m':d.subtree_com[1].tolist(),'contacts':contacts}

def run(params):
    support=params.get('support','left');swing='right' if support=='left' else 'left'
    height=params.get('step_height_m',0.);edge=.045
    base.E.EDGE=edge
    with contextlib.redirect_stdout(io.StringIO()):
        m,d,c,_=base.BUILD(base.IP,height)
        mj.mj_resetDataKeyframe(m,d,mj.mj_name2id(m,mj.mjtObj.mjOBJ_KEY,'STAND'))
        joints=m.actuator_trnid[:,0];sq=m.jnt_qposadr[joints]
        if params.get('seed'):
            d.qpos[sq]+=np.random.default_rng(params['seed']).uniform(-.005,.005,len(sq))
        c.reset(d.qpos);c.last_ts=float(d.time);mj.mj_forward(m,d)
        p=base.IP.PolicyInference(m,d,walking_onnx_path=str(base.E.POLICY),bam_ctrl=c,
                                   use_projected_gravity=True,new_cmd_obs=True)
        base.E.bench.install_encoder_view(p,c,m,d);bq,_=base.E.bench.backlash_indices(m)
        c.q_target[:]=d.qpos[sq]+d.qpos[bq];p.set_vel_cmd(0,0,0)
    floor=mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,'floor')
    step=mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,'step_top')
    feet={s:mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,s+'_foot_collision') for s in ['left','right']}
    lo,hi=m.jnt_range[joints,0]+.07,m.jnt_range[joints,1]-.07
    frames=[];maxenv=maxself=maxtorque=maxover=0.;resets=False
    longest=continuous=0.;longest_quiet=quiet_continuous=0.;first_one=None
    mg=float(m.body_mass.sum()*abs(m.opt.gravity[2]));planner=None;plans={};start=None
    control=params.get('control','planned');feedback=params.get('feedback',False)
    lift=params.get('lift_m',.015);duration=max(8. if height else 7.2,params.get('shift_s',1.5)+5.7)
    nsteps=0;initial={'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist()}
    for k in range(round(duration/CDT)):
        t=k*CDT;mj.mj_forward(m,d)
        phase='settle'
        if t<1 or control in ['stand','walk']:
            if control=='walk' and t>=1:
                with contextlib.redirect_stdout(io.StringIO()):p.set_vel_cmd(params.get('speed_mps',.15),0,0)
                phase='walk'
            target=np.clip(p.default_pose+p.infer()*p.action_scale,lo,hi)
        else:
            if planner is None:
                planner=Planner(m,d,sq,bq,feet,support=support,free_swing=params.get('free_swing',False),head=params.get('head',False))
                start=c.q_target.copy()
                offset=np.clip(start-(d.qpos[sq]+d.qpos[bq]),-.10,.10)
                b=base.E.foot_bounds(m,d,feet[support])
                center=(np.array(b['min'])+np.array(b['max']))[:2]/2
                center[0]=planner.com0[0]
                center[1]=planner.com0[1]+params.get('shift_fraction',1.)*(center[1]-planner.com0[1])
                fp={s:v.copy() for s,v in planner.pos.items()}
                qshift,e1=planner.solve(center,fp)
                shift_offset=planner.gravity_offset(c,False) if params.get('gravity_comp',False) else offset
                fp[swing][2]+=lift
                qlift,e2=planner.solve(center,fp)
                lift_offset=planner.gravity_offset(c) if params.get('gravity_comp',False) else offset
                plans={'shift':e1,'lift':e2,'target_com_xy_m':center.tolist(),
                       'shift_output_q_rad':qshift.tolist(),'lift_output_q_rad':qlift.tolist(),
                       'baseline_hold_offset_rad':offset.tolist(),
                       'shift_gravity_offset_rad':shift_offset.tolist(),'lift_gravity_offset_rad':lift_offset.tolist()}
                qshift=np.clip(qshift+shift_offset,lo,hi);qlift=np.clip(qlift+lift_offset,lo,hi)
                if height:
                    # Place complete sole beyond the edge, not just its centre.
                    shift_x=edge+.003-base.E.foot_bounds(m,planner.d,feet[swing])['min'][0]
                    fp[swing][0]+=shift_x
                    qforward,e3=planner.solve(center,fp)
                    fp[swing][2]-=lift-height
                    qland,e4=planner.solve(center,fp)
                    plans.update({'forward':e3,'land':e4,'swing_forward_m':float(shift_x)})
                    qforward=np.clip(qforward+offset,lo,hi);qland=np.clip(qland+offset,lo,hi)
            shift_s=params.get('shift_s',1.5)
            a=1.;b=a+shift_s;cc=b+.8;dd=cc+1.
            if t<b:phase='shift';target=start+(qshift-start)*smooth((t-a)/shift_s)
            elif t<cc:phase='lift';target=qshift+(qlift-qshift)*smooth((t-b)/.8)
            elif t<dd:phase='hold';target=qlift.copy()
            elif height:
                if t<dd+.8:phase='swing_forward';target=qlift+(qforward-qlift)*smooth((t-dd)/.8)
                elif t<dd+1.6:phase='land';target=qforward+(qland-qforward)*smooth((t-dd-.8)/.8)
                else:phase='step_hold';target=qland.copy()
            elif t<dd+.8:phase='lower';target=qlift+(qshift-qlift)*smooth((t-dd)/.8)
            elif t<dd+1.8:phase='recenter';target=qshift+(start-qshift)*smooth(t-dd-.8)
            else:phase='recover';target=np.clip(p.default_pose+p.infer()*p.action_scale,lo,hi)
            if feedback and phase in ['lift','hold','swing_forward','land','step_hold']:
                # A bounded measured-state correction, applied only to motor targets.
                # Hip-roll axes have opposite sign across the mirrored legs.
                err=center[1]-d.subtree_com[1,1]
                corr=float(np.clip(5*err-.10*d.qvel[1],-.08,.08))
                for i,j in enumerate(joints):
                    name=mj.mj_id2name(m,mj.mjtObj.mjOBJ_JOINT,int(j))
                    if name=='left_hip_roll':target[i]-=corr
                    elif name=='right_hip_roll':target[i]+=corr
        target=np.clip(target,lo,hi);c.q_target[:]=target
        p.last_action=((target-p.default_pose)/p.action_scale).astype(np.float32)
        for _ in range(round(CDT/DT)):
            before=float(d.time);c.update();mj.mj_step(m,d);nsteps+=1
            resets|=abs(float(d.time)-before-DT)>1e-8
            if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all():raise FloatingPointError('Nonfinite state')
            maxtorque=max(maxtorque,float(abs(d.actuator_force).max()))
            # Both output angle and motor angle are inspected; neither limit is widened.
            for qq in [d.qpos[sq],d.qpos[sq]+d.qpos[bq]]:
                maxover=max(maxover,float(np.maximum(m.jnt_range[joints,0]-qq,qq-m.jnt_range[joints,1]).max()))
            for ct in d.contact:
                if floor in (ct.geom1,ct.geom2) or step>=0 and step in (ct.geom1,ct.geom2):maxenv=max(maxenv,-float(ct.dist))
                else:maxself=max(maxself,-float(ct.dist))
        mj.mj_forward(m,d);s=state(m,d,feet,floor,step);sample=base.E.bench.sample(m,d,0,0)
        one=(s['foot_loads_n'][support]>.8*mg and s['foot_loads_n'][swing]<.02
             and s['foot_bounds'][swing]['min'][2]>=.005 and s['nonfoot_environment_load_n']<.02)
        stable=one and sample['tilt_deg']<20 and sample['linear_speed_mps']<.06 and sample['angular_speed_rps']<.7
        continuous=continuous+CDT if one else 0.;longest=max(longest,continuous)
        quiet_continuous=quiet_continuous+CDT if stable else 0.;longest_quiet=max(longest_quiet,quiet_continuous)
        if one and first_one is None:first_one=float(d.time)
        frames.append({'t':round(float(d.time),6),'phase':phase,**sample,**s,
                       'motor_target_rad':target.tolist(),'single_support':bool(one),'quiet_single_support':bool(stable)})
    warnings={str(i):int(w.number) for i,w in enumerate(d.warning) if w.number}
    quality=not warnings and not resets and maxenv<.001 and maxself<.001 and maxover<math.radians(1)
    last=frames[-50:]
    top=bool(height and all(f['step_loads_n'][swing]>.2*mg and f['foot_bounds'][swing]['min'][0]>edge+.001
                            and f['tilt_deg']<20 and f['nonfoot_environment_load_n']<.02 for f in last))
    summary={'params':params,'longest_single_support_s':round(longest,6),
             'longest_quiet_single_support_s':round(longest_quiet,6),'first_single_support_s':first_one,
             'single_support_1s':longest_quiet>=.999,'step_foot_placed_final_1s':top,'quality_gate':bool(quality),
             'status':'PASS' if quality and (top if height else longest_quiet>=.999) else ('CONTROL' if control in ['stand','walk'] else 'FAIL'),
             'max_environment_penetration_m':maxenv,'max_self_penetration_m':maxself,'max_joint_limit_overshoot_deg':math.degrees(max(0.,maxover)),
             'peak_motor_torque_nm':maxtorque,'max_tilt_deg':max(f['tilt_deg'] for f in frames),
             'final_tilt_deg':frames[-1]['tilt_deg'],'warnings':warnings,'physics_reset_detected':resets,
             'physical_steps':nsteps,'mass_kg':float(m.body_mass.sum()),'plan':plans}
    return {'schema':'microduck-single-support/v1','dt':DT,'control_dt':CDT,'initial':initial,
            'terrain':{'height_m':height,'edge_x_m':edge},'summary':summary,'frames':frames}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--workspace',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True);ap.add_argument('--cases',type=Path,required=True)
    a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    with contextlib.redirect_stdout(io.StringIO()):gate=base.initialize(a.workspace.resolve())
    import verify_source
    helpers=verify_source.verify()
    results=[]
    for params in json.loads(a.cases.read_text()):
        tr=run(params);path=a.out/(params['name']+'.json')
        path.write_text(json.dumps(tr,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n')
        entry={**tr['summary'],'trace_file':path.name,'trace_sha256':sha(path)};results.append(entry)
        print(json.dumps({k:v for k,v in entry.items() if k!='plan'}),flush=True)
    (a.out/'summary.json').write_text(json.dumps({'source_gate':gate,'helper_verification':helpers,
         'code_sha256':sha(__file__),'episodes':results},indent=2)+'\n')

if __name__=='__main__':main()
