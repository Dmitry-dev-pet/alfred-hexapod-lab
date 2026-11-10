#!/usr/bin/env python3
from pathlib import Path
import contextlib,io,json
import numpy as np,mujoco as mj
import contact_cpu as e
HERE=Path(__file__).resolve().parent

def audit_surface(m,d,tr):
    visible=[g for g in range(m.ngeom) if m.geom_group[g]==2];step=mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,'step_top');h=tr['terrain']['height_m'];edge=tr['terrain']['edge_x_m'];rootconv=allconv=rootpoint=0.;worst=None
    verts={g:m.mesh_vert[m.mesh_vertadr[m.geom_dataid[g]]:m.mesh_vertadr[m.geom_dataid[g]]+m.mesh_vertnum[m.geom_dataid[g]]] for g in visible}
    if not h:return {'max_visible_body_convex_overlap_m':0.,'max_all_visible_convex_overlap_m':0.,'max_visible_body_vertex_inside_step_m':0.,'worst_body_mesh':None}
    for f in tr['frames']:
        d.qpos[:]=f['qpos'];mj.mj_forward(m,d)
        for g in visible:
            v=verts[g]@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
            if v[:,0].max()<edge or v[:,0].min()>edge+1.2 or v[:,1].max()<-.6 or v[:,1].min()>.6 or v[:,2].max()<0 or v[:,2].min()>h:continue
            dep=max(0.,-float(mj.mj_geomDistance(m,d,g,step,.2,None)));allconv=max(allconv,dep)
            if m.body_weldid[m.geom_bodyid[g]]==1:
                if dep>rootconv:rootconv=dep;worst={'t':f['t'],'mesh':mj.mj_id2name(m,mj.mjtObj.mjOBJ_MESH,int(m.geom_dataid[g]))}
                point=np.stack([v[:,0]-edge,edge+1.2-v[:,0],v[:,1]+.6,.6-v[:,1],v[:,2],h-v[:,2]],axis=1).min(axis=1);rootpoint=max(rootpoint,float(point.max()))
    return {'max_visible_body_convex_overlap_m':rootconv,'max_all_visible_convex_overlap_m':allconv,'max_visible_body_vertex_inside_step_m':rootpoint,'worst_body_mesh':worst}

def main():
    j=json.loads((HERE/'traces/summary.json').read_text());ip=e.e.bench.upstream_inference(e.e.RL);ip.ort.InferenceSession=e.e.bench.cpu_session;episodes=[]
    for s in j['episodes']:
        p=HERE/'traces'/s['trace_file'];assert e.sha(p)==s['trace_sha256'];tr=json.loads(p.read_text());assert not s['warnings'] and not s['physics_reset_detected'] and not s['initial_obstacle_loaded_substeps'] and s['initial_obstacle_penetration_m']==0
        with contextlib.redirect_stdout(io.StringIO()):m,d,*_=e.setup(ip,s['height_m'],s['edge_x_m'],'back')
        assert m.opt.timestep==.0005 and m.nu==14 and m.nq==35;detail=audit_surface(m,d,tr);assert detail['max_visible_body_convex_overlap_m']<.001
        js=m.actuator_trnid[:,0]
        for f in tr['frames']:
            target=np.array(f['motor_target_rad']);assert np.all(target>=m.jnt_range[js,0]+.035-1e-12) and np.all(target<=m.jnt_range[js,1]-.035+1e-12)
        last=tr['frames'][-50:];edge_stance=bool(s['standing_final_1s'] and s['height_m'] and all(all(v>.05 for v in f['step_loads_n'].values()) for f in last))
        strict=bool(edge_stance and all(all(b['min'][0]>s['edge_x_m']+.001 for b in f['foot_bounds'].values()) for f in last));assert strict==s['step_goal_reached']
        detail.update(trace_file=s['trace_file'],trace_sha256=s['trace_sha256'],supported_top_stance=edge_stance);episodes.append(detail);print(json.dumps(detail),flush=True)
    old=json.loads((e.WORK/'microduck-wall/traces/close-h50.json').read_text())
    with contextlib.redirect_stdout(io.StringIO()):m,d,*_=e.old.setup(ip,.05,old['terrain']['edge_x_m'],'back')
    legacy=audit_surface(m,d,old);assert legacy['max_visible_body_vertex_inside_step_m']>.01
    repeats=[]
    for name in ['close-h50.json','flat-h0.json']:
        original=(HERE/'traces'/name).read_bytes();tr=json.loads(original);s=tr['summary']
        with contextlib.redirect_stdout(io.StringIO()):again=e.run(ip,'back','hip',.65,1.2,s['height_m'],s['edge_x_m'])
        again['condition']=s['condition'];again['summary']['condition']=s['condition'];raw=(json.dumps(again,separators=(',',':'),allow_nan=False,sort_keys=True)+'\n').encode();assert raw==original;repeats.append(name)
    r={'schema':'microduck-contact-verification/v2','episodes':episodes,'legacy_5cm_body_step_overlap':legacy,'old_5cm_trace_sha256':e.sha(e.WORK/'microduck-wall/traces/close-h50.json'),'corrected_5cm_substep_body_penetration_m':next(s['max_body_step_penetration_m'] for s in j['episodes'] if s['trace_file']=='close-h50.json'),'max_substep_body_step_penetration_m':max(s['max_body_step_penetration_m'] for s in j['episodes']),'max_substep_all_step_penetration_m':max(s['max_step_penetration_m'] for s in j['episodes']),'max_visible_body_step_convex_overlap_m':max(x['max_visible_body_convex_overlap_m'] for x in episodes),'byte_identical_repeats':repeats,'flat_recovery':next(s['fall_and_recover'] for s in j['episodes'] if s['condition']=='flat'),'upper_platform_successes':sum(s['step_goal_reached'] for s in j['episodes']),'initial_obstacle_contacts':0,'warnings_or_resets':0,'hardware_tested':False,'actual_webgl_rendering_verified':False}
    (HERE/'verification.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k not in ['episodes','legacy_5cm_body_step_overlap']}))
if __name__=='__main__':main()
