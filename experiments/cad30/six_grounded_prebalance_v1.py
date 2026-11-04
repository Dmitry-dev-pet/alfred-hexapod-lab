"""Six-grounded start -> body reposition -> front-leg force-unload micro-ramp.
Diagnostic, NOT a validated four-leg gait. All state evolves with MuJoCo.
"""
from __future__ import annotations
from pathlib import Path
import os,sys,math,json
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'experiments/cad30')]
import numpy as np,mujoco
from optimized_claw_gait import init_model,actuator_mapping,physical_contact_forces,cartesian_shift_paths,sample_ik_path,force_oracle,FRONT,ALL
from hexlab.stand_geometry_v04 import tip_world,geom_mesh_world_vertices
from hexlab.step_one_leg_v04 import floor_contacts,solve_waypoint,smoothstep
from support_wrench_qp import support_margin_xy

def simulate_case(shift=0.036008,z=0.010,unload_dur=1.5,stand_dur=0.7,prep_hold=0.6,high_claws=0.10,high_dur=1.5,high_hold=0.5,record=False,mu=1.1):
    m,d,legs=init_model(mu)
    qa,da,idx=actuator_mapping(m,d,legs)
    qbase=d.qpos[qa].copy()
    paths,errs=cartesian_shift_paths(m,d,legs,FRONT,shift)
    limit=m.actuator_ctrlrange.copy();dt=float(m.opt.timestep)
    bid=m.body('base_link_2').id
    rows=[];snapshots=[];saturation=0;peak_u=0;mincontacts=6;min_stance_contacts=4;last_t=0
    if record:
        snapshots.append({'t':0.0,'qpos':np.round(d.qpos,8).tolist(),'phase':'six_leg_start',
                          'contacts':sorted(floor_contacts(m,d,mujoco))})
    def step(command,t,phase):
        nonlocal saturation,peak_u,mincontacts,min_stance_contacts,last_t
        u=7.*(command-d.qpos[qa])-.32*d.qvel[da]
        saturation+=int(np.any((u>limit[:,1]+1e-7)|(u<limit[:,0]-1e-7)))
        peak_u=max(peak_u,float(np.max(abs(u))))
        d.ctrl[:]=np.clip(u,limit[:,0],limit[:,1]);mujoco.mj_step(m,d)
        contacts=floor_contacts(m,d,mujoco)
        mincontacts=min(mincontacts,len(contacts))
        min_stance_contacts=min(min_stance_contacts,sum(c in contacts for c in ALL if c not in FRONT))
        last_t=t
        if record and int(round(t/dt))%50==0:snapshots.append({"t":round(t,3),"qpos":np.round(d.qpos,8).tolist(),"phase":phase,"contacts":sorted(contacts)})
        if int(round(t/dt))%10==0:
            f,p=physical_contact_forces(m,d)
            com=(d.xipos*m.body_mass[:,None]).sum(axis=0)/m.body_mass.sum()
            four=[p[c][:2] for c in ALL if c not in FRONT and p[c] is not None]
            mar=support_margin_xy(np.array(four),com[:2]) if len(four)>=3 else -0.5
            rows.append({'t':round(t,3),'phase':phase,'front_N':round(float(sum(f[c][2] for c in FRONT)),4),
                        'front_contact':sum(c in contacts for c in FRONT),'four_contacts':sum(c in contacts for c in ALL if c not in FRONT),
                        'com_xy':np.round(com[:2],5).tolist(),'four_margin_mm':round(mar*1000,2),
                        'max_abs_tau_nm':round(float(max(abs(d.ctrl))),4),'contacts':sorted(contacts)})
    n=round(2/dt)
    for k in range(n):
        t=(k+1)*dt;a=smoothstep(t/2)
        cmd=qbase.copy()
        for c,path in paths.items():cmd[idx[c]]=sample_ik_path(path,a)
        step(cmd,t,'body_reposition_6_contacts')
    # Let contact loads settle while all six supports are still active.
    for k in range(round(prep_hold/dt)):
        t=2+(k+1)*dt
        cmd=qbase.copy()
        for c,path in paths.items():cmd[idx[c]]=path[-1]
        step(cmd,t,'stabilize_before_unload')
    prep=d.qpos.copy()
    prepfz=rows[-1]['front_N'];prepcom=rows[-1]['com_xy'];prepmargin=rows[-1]['four_margin_mm']
    finalcmd=qbase.copy()
    for c,path in paths.items():finalcmd[idx[c]]=path[-1]
    frontcmd={};front_err={}
    for leg in legs:
        c=leg['suffix']
        if c not in FRONT:continue
        ref=mujoco.MjData(m);ref.qpos[:]=prep;ref.qvel[:]=0;mujoco.mj_forward(m,ref)
        tip=tip_world(m,ref,leg)
        q,e=solve_waypoint(m,ref,mujoco,leg,tip+[0,0,z],prep[leg['qadrs']])
        frontcmd[c]=q;front_err[c]=round(float(e)*1000,3)
    n=round(unload_dur/dt)
    for k in range(n):
        t=2+prep_hold+(k+1)*dt;a=smoothstep((k+1)/n)
        cmd=finalcmd.copy()
        for c,q in frontcmd.items():cmd[idx[c]]=qbase[idx[c]]+a*(q-qbase[idx[c]])
        step(cmd,t,'gradual_front_unload')
    # Hold target and observe if front still loads (4leg stable if no front contacts).
    n=round(stand_dur/dt)
    for k in range(n):
        t=2+prep_hold+unload_dur+(k+1)*dt
        cmd=finalcmd.copy()
        for c,q in frontcmd.items():cmd[idx[c]]=q
        step(cmd,t,'settle_after_unload')
    # Extra visual claws-raising phase happens ONLY after the gradual unloading
    # and is verified as a real, free-body MuJoCo evolution.
    clearances={}
    if high_claws:
        prep_high=d.qpos.copy()
        high_command={};high_ik={}
        for leg in legs:
            c=leg['suffix']
            if c not in FRONT:continue
            ref=mujoco.MjData(m);ref.qpos[:]=prep_high;ref.qvel[:]=0.;mujoco.mj_forward(m,ref)
            tip=tip_world(m,ref,leg)
            q,e=solve_waypoint(m,ref,mujoco,leg,tip+[0,0,high_claws],prep_high[leg['qadrs']])
            high_command[c]=q;high_ik[c]=round(float(e)*1000,3)
        n=round(high_dur/dt)
        # Front joint angles are currently at low unload targets.
        current=finalcmd.copy()
        for c,q in frontcmd.items():current[idx[c]]=q
        command_final=current.copy()
        for c,q in high_command.items():command_final[idx[c]]=q
        for k in range(n):
            t=2+prep_hold+unload_dur+stand_dur+(k+1)*dt
            a=smoothstep((k+1)/n)
            cmd=current*(1-a)+command_final*a
            step(cmd,t,'raise_front_high')
        for k in range(round(high_hold/dt)):
            t=2+prep_hold+unload_dur+stand_dur+high_dur+(k+1)*dt
            step(command_final,t,'hold_front_high')
        clearances={c:round(float(geom_mesh_world_vertices(m,d,next(ll for ll in legs if ll['suffix']==c)['geom'])[:,2].min())*1000,2) for c in FRONT}
    frontloads=[row for row in rows if row['phase']=='gradual_front_unload']
    timebelow=next((row['t'] for row in frontloads if row['front_N'] <.2),None)
    last=rows[-1];record={'model_friction_mu':mu,'shift_cmd_mm':round(shift*1000,2),'front_z_command_mm':round(z*1000,2),
      'after_prep_front_load_N':prepfz,'after_prep_com_xy_m':prepcom,'after_prep_four_margin_mm':prepmargin,
      'after_unload_front_load_N':last['front_N'],'after_unload_front_contacts':last['front_contact'],
      'after_unload_four_support_contacts':last['four_contacts'],'after_unload_four_margin_mm':last['four_margin_mm'],
      'first_time_front_Fz_below_0p2N':timebelow,'min_contacts_all_steps':mincontacts,
      'peak_unclipped_control_Nm':round(peak_u,3),'motor_saturation_fraction':round(saturation/(round((2+prep_hold+unload_dur+stand_dur+(high_dur+high_hold if high_claws else 0))/dt)),5),
      'max_front_IK_error_mm':max(front_err.values()),'last_contacts':last['contacts'],
      'high_claw_clearances_mm':clearances,'min_stance_contacts_all_steps':min_stance_contacts,
      'timeseries_50hz':rows,'qpos_10hz':snapshots}
    return record

if __name__ == '__main__':
    import argparse
    ap=argparse.ArgumentParser(description='Compute six-grounded -> four-support prebalance, no autonomous stepping')
    ap.add_argument('--shift-mm',type=float,default=38.)
    ap.add_argument('--front-unload-mm',type=float,default=10.)
    ap.add_argument('--mu',type=float,default=1.1)
    ap.add_argument('--record',action='store_true')
    ap.add_argument('--out',type=Path,default=ROOT/'results/six_grounded_claws_mu1p1_2026-11-04.json')
    args=ap.parse_args()
    result=simulate_case(shift=args.shift_mm/1000,z=args.front_unload_mm/1000,mu=args.mu,record=args.record)
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(result,ensure_ascii=False,separators=(',',':'))+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('timeseries_50hz','qpos_10hz')},ensure_ascii=False,indent=2))
