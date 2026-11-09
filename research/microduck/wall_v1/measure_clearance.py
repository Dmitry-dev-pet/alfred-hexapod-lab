"""Conservative swept outer-geometry clearance for the first second of STAND."""
from pathlib import Path
import sys,json,contextlib,io
import numpy as np,mujoco as mj
from scipy.spatial import ConvexHull
WORK=next(p for p in Path(__file__).resolve().parents if (p/'microduck-work/source-manifest.json').is_file())
sys.path.insert(0,str(WORK/'microduck-fall'));import fall_cpu as f
HERE=Path(__file__).resolve().parent

def shape_data(m):
    shapes=[]
    for g in range(m.ngeom):
        mid=int(m.geom_dataid[g])
        if m.geom_type[g]!=mj.mjtGeom.mjGEOM_MESH or mid<0:continue
        a,n=int(m.mesh_vertadr[mid]),int(m.mesh_vertnum[mid]);pts=m.mesh_vert[a:a+n].astype(float)
        hull=ConvexHull(pts);edges=set()
        for tri in hull.simplices:
            for i,j in [(tri[0],tri[1]),(tri[1],tri[2]),(tri[2],tri[0])]:edges.add(tuple(sorted((int(i),int(j)))))
        shapes.append((g,pts,np.array(sorted(edges),dtype=int)))
    return shapes

def rear_extent(m,d,shapes,h,visual=True):
    best=-np.inf;gid=None
    for g,pts,edges in shapes:
        if not visual and int(m.geom_contype[g])!=1:continue
        v=pts@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
        # Convex hull clipped by the platform's 0..h vertical slab.
        inside=v[(v[:,2]>=0)&(v[:,2]<=h)];xs=[inside[:,0]]
        a,b=v[edges[:,0]],v[edges[:,1]];dz=b[:,2]-a[:,2]
        for z in [0,h]:
            good=(np.minimum(a[:,2],b[:,2])<=z)&(np.maximum(a[:,2],b[:,2])>=z)&(abs(dz)>1e-12)
            aa,bb,dd=a[good],b[good],dz[good]
            xs.append(aa[:,0]+(bb[:,0]-aa[:,0])*(z-aa[:,2])/dd)
        x=max((float(x.max()) for x in xs if len(x)),default=-np.inf)
        if x>best:best=x;gid=g
    return best,gid

def main():
    ip=f.e.bench.upstream_inference(f.e.RL)
    with contextlib.redirect_stdout(io.StringIO()):m,d,*_=f.setup(ip,.01,.08,'back')
    initial=d.qpos.copy();shapes=shape_data(m);ref=json.loads((WORK/'microduck-fall/traces/step10-back.json').read_text());states=[initial]+[np.array(x['qpos']) for x in ref['frames'][:50]];entries=[]
    for h in [.01,.02,.03,.05,.1,.25]:
        ext=[];coll=[];ids=[]
        for q in states:
            d.qpos[:]=q;mj.mj_forward(m,d);v,g=rear_extent(m,d,shapes,h,True);c,_=rear_extent(m,d,shapes,h,False);ext.append(v);coll.append(c);ids.append(g)
        edge=max(ext)+.002;g=ids[ext.index(max(ext))]
        entries.append({'height_m':h,'edge_x_m':edge,'swept_outer_rear_x_m':max(ext),'settled_outer_rear_x_m':ext[-1],'settled_surface_gap_m':edge-ext[-1],'swept_collision_rear_x_m':max(coll),'limiting_gid':g,'limiting_body':mj.mj_id2name(m,mj.mjtObj.mjOBJ_BODY,int(m.geom_bodyid[g]))})
    r={'method':'Maximum support X of all original visual/collision convex hulls clipped to platform vertical slab, over initial keyframe plus 50 settled reference states. Add 2 mm. This conservative geometry setup is followed by continuous initial contact checks in the actual run.','reference_trace_sha256':f.sha(WORK/'microduck-fall/traces/step10-back.json'),'samples':len(states),'entries':entries};(HERE/'clearance-layout.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
if __name__=='__main__':main()
