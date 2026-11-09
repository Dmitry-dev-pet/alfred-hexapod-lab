"""Complete environment-only coverage of original visible STL convex hulls.
No new robot self-collision pairs, geometry mass or kinematic degrees of freedom.
"""
from pathlib import Path
import sys,contextlib,io,os
import mujoco as mj,numpy as np
WORK=Path(os.environ['MICRODUCK_WORKSPACE']).resolve()
import wall_cpu as old
terrain=old.e;DT=.0005

def signature(g):
    q=np.asarray(g.quat,dtype=float);q=q/np.linalg.norm(q);first=next((x for x in q if abs(x)>1e-12),1)
    if first<0:q=-q
    return (g.meshname,tuple(np.round(g.pos,12)),tuple(np.round(q,12)))

def cover_environment(s):
    added=[]
    for b in list(s.bodies):
        gs=list(b.geoms);covered={signature(g) for g in gs if g.type==mj.mjtGeom.mjGEOM_MESH and (g.contype&1 or g.conaffinity&1)}
        missing=[g for g in gs if g.type==mj.mjtGeom.mjGEOM_MESH and g.group==2 and signature(g) not in covered]
        if not missing:continue
        child=b.add_body(name='env_cover_'+b.name)
        for i,g in enumerate(missing):
            name=f'env_{b.name}_{i}'
            child.add_geom(name=name,type=mj.mjtGeom.mjGEOM_MESH,meshname=g.meshname,pos=g.pos.tolist(),quat=g.quat.tolist(),contype=4,conaffinity=0,group=3,density=0,rgba=[.3,.6,.9,.15])
            added.append({'body':b.name,'mesh':g.meshname,'geom_name':name})
    for g in s.geoms:
        if g.name in ['floor','step_top']:g.conaffinity|=4
        if g.contype or g.conaffinity:
            g.solref=[.002,1];g.solimp=[.99,.999,.0001,.5,2]
    return added

class EnvironmentMuJoCo:
    def __init__(self,height,edge):self.height=height;self.edge=edge
    def __getattr__(self,name):
        if name!='MjSpec':return getattr(mj,name)
        height=self.height;edge=self.edge
        class Spec:
            @staticmethod
            def from_file(path):
                s=mj.MjSpec.from_file(path)
                if height:
                    b=s.worldbody.add_body(name='step_platform',pos=[edge+.6,0,height/2]);b.add_geom(name='step_top',type=mj.mjtGeom.mjGEOM_BOX,size=[.6,.6,height/2],friction=[1,.005,.0001])
                cover_environment(s);return s
        return Spec

def build(ip,height,dt=DT,firm=True):
    proxy=ip.mujoco;ip.mujoco=EnvironmentMuJoCo(height,terrain.EDGE)
    try:
        bam=ip.load_bam_model(200,7.4,None)
        return ip.load_mujoco_with_bam(str(terrain.RL/'src/mjlab_microduck/robot/microduck/scene_backlash.xml'),bam,DT,.1,6)
    finally:ip.mujoco=proxy

def setup(ip,height,edge,direction):
    previous=terrain.build;terrain.build=build
    try:return old.setup(ip,height,edge,direction)
    finally:terrain.build=previous

if __name__=='__main__':
    ip=terrain.bench.upstream_inference(terrain.RL)
    with contextlib.redirect_stdout(io.StringIO()):a,_,*_=old.setup(ip,.05,.03,'back');b,d,*_=setup(ip,.05,.03,'back')
    orig=[i for i in range(a.ngeom) if a.geom_group[i]==2];new=[i for i in range(b.ngeom) if b.geom_group[i]==2];assert len(orig)==len(new)==70
    for i,j in zip(orig,new):
        assert a.geom_dataid[i]==b.geom_dataid[j] and np.allclose(a.geom_pos[i],b.geom_pos[j]) and np.allclose(a.geom_quat[i],b.geom_quat[j])
    assert np.array_equal(a.jnt_range,b.jnt_range) and a.nq==b.nq==35 and a.nu==b.nu==14
    for i in range(1,a.nbody):
        name=mj.mj_id2name(a,mj.mjtObj.mjOBJ_BODY,i);j=mj.mj_name2id(b,mj.mjtObj.mjOBJ_BODY,name);assert np.array_equal(a.body_mass[i],b.body_mass[j]) and np.array_equal(a.body_inertia[i],b.body_inertia[j])
    print({'old_geoms':a.ngeom,'new_geoms':b.ngeom,'added_env_meshes':int(sum(b.geom_contype==4)),'old_bodies':a.nbody,'new_bodies':b.nbody,'mass_before':float(a.body_mass.sum()),'mass_after':float(b.body_mass.sum()),'visible_order_preserved':True,'dt':b.opt.timestep})
