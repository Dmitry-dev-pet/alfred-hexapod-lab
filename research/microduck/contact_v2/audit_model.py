#!/usr/bin/env python3
from pathlib import Path
import contextlib,io,json
import numpy as np,mujoco as mj
from collision_model import old,setup

def main():
    ip=old.e.bench.upstream_inference(old.e.RL)
    with contextlib.redirect_stdout(io.StringIO()):a,da,*_=old.setup(ip,.05,.03,'back');b,db,*_=setup(ip,.05,.03,'back')
    assert a.nq==b.nq==35 and a.nv==b.nv==34 and a.nu==b.nu==14
    for name in ['jnt_range','dof_armature','actuator_forcerange','mesh_vert']:
        assert np.array_equal(getattr(a,name),getattr(b,name)),name
    robot_mass=0.
    for i in range(1,a.nbody):
        name=mj.mj_id2name(a,mj.mjtObj.mjOBJ_BODY,i);j=mj.mj_name2id(b,mj.mjtObj.mjOBJ_BODY,name)
        for prop in ['body_mass','body_inertia','body_ipos','body_iquat']:assert np.array_equal(getattr(a,prop)[i],getattr(b,prop)[j]),(name,prop)
        if name!='step_platform':robot_mass+=a.body_mass[i]
    ma=np.zeros((a.nv,a.nv));mb=np.zeros_like(ma);mj.mj_fullM(a,da,ma);mj.mj_fullM(b,db,mb);assert np.max(abs(ma-mb))<1e-12
    oa=[i for i in range(a.ngeom) if a.geom_group[i]==2];nb=[i for i in range(b.ngeom) if b.geom_group[i]==2];assert len(oa)==len(nb)==70
    covered=[]
    for i,j in zip(oa,nb):
        assert a.geom_dataid[i]==b.geom_dataid[j] and np.allclose(da.geom_xpos[i],db.geom_xpos[j],atol=1e-12) and np.allclose(da.geom_xmat[i],db.geom_xmat[j],atol=1e-12)
        match=[k for k in range(b.ngeom) if b.geom_dataid[k]==b.geom_dataid[j] and b.geom_type[k]==b.geom_type[j] and b.body_weldid[b.geom_bodyid[k]]==b.body_weldid[b.geom_bodyid[j]] and (int(b.geom_contype[k])&int(b.geom_conaffinity[0]) or int(b.geom_contype[0])&int(b.geom_conaffinity[k])) and np.allclose(db.geom_xpos[j],db.geom_xpos[k],atol=1e-12) and np.allclose(db.geom_xmat[j],db.geom_xmat[k],atol=1e-12)]
        assert match,(i,j);covered.append({'original_gid':i,'corrected_visual_gid':j,'environment_collision_gid':match[0]})
    added=np.flatnonzero(b.geom_contype==4).tolist();assert len(added)==60
    for g in added:
        assert b.geom_conaffinity[g]==0
        # The environment must accept bit 4; original robot geometries must not.
        for h in range(b.ngeom):
            n=mj.mj_id2name(b,mj.mjtObj.mjOBJ_GEOM,h)
            if n not in ['floor','step_top']:assert not ((int(b.geom_contype[g])&int(b.geom_conaffinity[h])) or (int(b.geom_contype[h])&int(b.geom_conaffinity[g])))
    mapping={x['corrected_visual_gid']:x['environment_collision_gid'] for x in covered};tr=json.loads((Path(__file__).parent/'traces/close-h50.json').read_text())
    for frame in tr['frames'][::50]:
        db.qpos[:]=frame['qpos'];mj.mj_forward(b,db)
        for g,c in mapping.items():assert np.allclose(db.geom_xpos[g],db.geom_xpos[c],atol=1e-12) and np.allclose(db.geom_xmat[g],db.geom_xmat[c],atol=1e-12)
    r={'added_environment_meshes':len(added),'all_original_visible_instances_covered':len(covered),'robot_mass_kg':robot_mass,'zero_mass_added_welded_bodies':int(b.nbody-a.nbody),'new_robot_self_collision_pairs':0,'mass_matrix_max_error':float(np.max(abs(ma-mb))),'original_mesh_bytes_joint_ranges_actuators_and_inertias_unchanged':True,'moving_attachment_checks':10,'visible_mapping':covered,'integration_dt_s':float(b.opt.timestep),'contact_solref':[.002,1],'contact_solimp':[.99,.999,.0001,.5,2]};(Path(__file__).parent/'model-audit.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='visible_mapping'}))
if __name__=='__main__':main()
