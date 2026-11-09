import sys,json,contextlib,io
from pathlib import Path
import numpy as np,mujoco as mj
sys.path.insert(0,str(Path('microduck-step').resolve()));import step_cpu as e
ip=e.bench.upstream_inference(e.RL)
with contextlib.redirect_stdout(io.StringIO()):m,d,c,_=e.build(ip,0,.001,True)
mj.mj_resetDataKeyframe(m,d,mj.mj_name2id(m,mj.mjtObj.mjOBJ_KEY,'STAND'));mj.mj_forward(m,d)
joints=[]
for a,j in enumerate(m.actuator_trnid[:,0]):joints.append({'a':a,'name':mj.mj_id2name(m,mj.mjtObj.mjOBJ_JOINT,int(j)),'range_deg':np.rad2deg(m.jnt_range[j]).tolist(),'limited':int(m.jnt_limited[j]),'pose':float(d.qpos[m.jnt_qposadr[j]])})
geoms=[{'id':i,'name':mj.mj_id2name(m,mj.mjtObj.mjOBJ_GEOM,i),'body':mj.mj_id2name(m,mj.mjtObj.mjOBJ_BODY,int(m.geom_bodyid[i])),'contype':int(m.geom_contype[i]),'conaffinity':int(m.geom_conaffinity[i])} for i in range(m.ngeom) if m.geom_contype[i] or m.geom_conaffinity[i]]
contacts=[{'pair':[int(x.geom1),int(x.geom2)],'distance_m':float(x.dist)} for x in d.contact]
r={'joints':joints,'collision_geoms':geoms,'initial_contacts':contacts,'nexclude':m.nexclude,'mass_kg':float(m.body_mass.sum()),'self_collision_note':'Upstream sparse convex collision meshes; visual STL is not a complete per-part self-collision model. No masks or exclusions altered.'}
Path('microduck-fall/model-audit.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
