#!/usr/bin/env python3
"""Independent clearance LP, baseline state identity, goals and repeats."""
from pathlib import Path
import contextlib,io,json,math
import numpy as np,mujoco as mj
from scipy.optimize import linprog
import wall_cpu as e

def linear_program_extent(m,d,h):
    best=-np.inf
    for g in range(m.ngeom):
        if m.geom_type[g]!=mj.mjtGeom.mjGEOM_MESH:continue
        mid=m.geom_dataid[g];a,n=m.mesh_vertadr[mid],m.mesh_vertnum[mid];v=m.mesh_vert[a:a+n]@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
        if v[:,2].min()>h or v[:,2].max()<0:continue
        # Any point in the convex hull is a convex combination of original vertices.
        r=linprog(-v[:,0],A_ub=np.stack([v[:,2],-v[:,2]]),b_ub=[h,0],A_eq=np.ones((1,n)),b_eq=[1],bounds=(0,None),method='highs')
        assert r.success;best=max(best,-r.fun)
    return best

def edge_stance_receipt(tr):
    last=tr['frames'][-50:];edge=tr['summary']['edge_x_m'];overhang=max(0.,max(edge-b['min'][0] for f in last for b in f['foot_bounds'].values()));loads={s:min(f['step_loads_n'][s] for f in last) for s in ['left','right']}
    assert all(v>.05 for v in loads.values()) and .018<overhang<.019 and not tr['summary']['step_goal_reached']
    return {'case':'close-h10.json','both_feet_loading_top':True,'minimum_top_loads_n':loads,'left_sole_overhang_m':overhang,'strict_full_sole_goal':False}

def main():
    p=e.HERE/'traces';j=json.loads((p/'summary.json').read_text());layout=json.loads((e.HERE/'clearance-layout.json').read_text());ip=e.e.bench.upstream_inference(e.e.RL);ip.ort.InferenceSession=e.e.bench.cpu_session;initial=None;lp=[];standing=[];worstself=worstlimit=0.;contacts=[]
    for s in j['episodes']:
        f=p/s['trace_file'];assert e.sha(f)==s['trace_sha256'];tr=json.loads(f.read_text());fs=tr['frames'];assert len(fs)==500 and tr['stand_policy_sha256']==e.e.bench.POLICY_SHA256
        assert not s['warnings'] and not s['physics_reset_detected'] and s['initial_obstacle_loaded_substeps']==0 and s['initial_obstacle_penetration_m']==0
        first=np.array([x['qpos'] for x in fs[:50]])
        if initial is None:initial=first
        else:assert np.array_equal(first,initial)
        assert all(not any(82 in c['geoms'] and c['force_n']>.05 for c in x['contacts']) for x in fs[:50])
        with contextlib.redirect_stdout(io.StringIO()):m,d,*_=e.setup(ip,s['height_m'],s['edge_x_m'],'back')
        js=m.actuator_trnid[:,0];lo=m.jnt_range[js,0]+.035;hi=m.jnt_range[js,1]-.035
        for frame in fs:
            q=np.array(frame['motor_target_rad']);assert np.all(q>=lo-1e-12) and np.all(q<=hi+1e-12)
        if s['condition']=='close':
            d.qpos[:]=fs[49]['qpos'];mj.mj_forward(m,d);v=linear_program_extent(m,d,s['height_m']);gap=s['edge_x_m']-v
            assert abs(gap-s['surface_gap_before_trigger_m'])<1e-7 and .00199<gap<.00202;lp.append({'case':s['trace_file'],'gap_m':gap})
        else:
            old=json.loads((e.WORK/'microduck-fall/traces'/f"step{round(s['height_m']*1000)}-back.json").read_text())
            assert all(a['qpos']==b['qpos'] and a['qvel']==b['qvel'] and a['ctrl']==b['ctrl'] and a['motor_target_rad']==b['motor_target_rad'] for a,b in zip(fs,old['frames']))
        last=fs[-50:];top=all(all(x>.05 for x in a['step_loads_n'].values()) and all(b['min'][0]>s['edge_x_m']+.001 for b in a['foot_bounds'].values()) for a in last)
        assert bool(top)==s['both_feet_on_step_final_1s'];quiet=all(a['tilt_deg']<20 and .09<a['trunk_z_m']<.18+s['height_m'] and a['linear_speed_mps']<.06 and a['angular_speed_rps']<.7 and all(x>.05 for x in a['foot_loads_n'].values()) for a in last)
        assert quiet==s['standing_final_1s'] and s['step_goal_reached']==bool(top and quiet)
        if quiet:standing.append(s['trace_file'])
        if s['back_into_obstacle']:contacts.append(s['trace_file'])
        worstself=max(worstself,s['max_self_contact_penetration_m']);worstlimit=max(worstlimit,s['max_motor_limit_overshoot_rad'])
    repeats=[]
    for name in ['close-h20.json','close-h100.json']:
        old=(p/name).read_bytes();s=json.loads(old)['summary']
        with contextlib.redirect_stdout(io.StringIO()):tr=e.run(ip,'back','hip',.65,1.2,s['height_m'],s['edge_x_m'])
        tr['condition']=s['condition'];tr['summary']['condition']=s['condition'];raw=(json.dumps(tr,separators=(',',':'),allow_nan=False,sort_keys=True)+'\n').encode();assert raw==old;repeats.append(name);print('Repeat exact: '+name,flush=True)
    r={'episodes_checked':len(j['episodes']),'source_and_policy_hashes_checked':True,'unchanged_reference_physical_streams':2,'same_first_second_qpos_all_cases':True,'initial_obstacle_contacts_all_cases':0,'independent_linear_program_clearances':lp,'byte_identical_repeats':repeats,'rear_contact_protocol_pass_cases':contacts,'standing_final_1s_cases':standing,'upper_platform_successes':sum(s['step_goal_reached'] for s in j['episodes']),'max_sparse_self_penetration_m':worstself,'max_rotor_limit_overshoot_deg':math.degrees(worstlimit),'all_command_targets_2deg_inside_limits':True,'warnings_or_resets':0,'hardware_tested':False,'webgl_verified':False}
    r['edge_stance']=edge_stance_receipt(json.loads((p/'close-h10.json').read_text()))
    (e.HERE/'verification.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r))
if __name__=='__main__':main()
