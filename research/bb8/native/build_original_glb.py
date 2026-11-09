from pathlib import Path
import os
import json, numpy as np, trimesh
from trimesh.transformations import quaternion_matrix
from math import pi,cos,sin
WORKSPACE=Path(os.environ.get('BB8_WORKSPACE','/mnt/data'))
root=WORKSPACE/'bb8-hweigo-reconstructed/meshes'
source=json.loads((root/'scene.json').read_text())
output=WORKSPACE/'bb8-hweigo-reconstructed'
C=np.array([-41.24816673006777,-27.2571967211034,99.26489749059236])*0.001
R=np.array([[1,0,0],[0,0,1],[0,-1,0]], dtype=float) # source -Y points up in Z world
visual_class={}
def color(f):
 if '球壳' in f:return [237,239,238,235]
 if '轮子' in f:return [215,119,41,255]
 if '轮胎' in f:return [33,38,47,255]
 if '电机' in f:return [75,82,91,255]
 if '主控' in f:return [46,150,120,255]
 if '电池' in f:return [58,100,155,255]
 if '头部底面' in f:return [245,240,221,255]
 return [160,167,176,255]
for key,m in source['meshes'].items():
 mesh=trimesh.load_mesh(root/m['file'],force='mesh',process=False)
 mesh.apply_scale(.001)
 mesh.visual.face_colors=color(m['source_file'])
 visual_class[key]=mesh
 print('SOURCE MESH',key,'verts',len(mesh.vertices),'faces',len(mesh.faces),'file',m['file'])

def build(variant):
 scene=trimesh.Scene(base_frame='world')
 n=0;skipped=[]
 for instance in source['instances']:
  label=instance['part_key']
  original_name=source['meshes'][label]['source_file']
  if variant=='exterior':
   if instance['hidden_in_original'] and original_name!='球壳1.SLDPRT':continue
   if original_name=='球壳1.SLDPRT' and n and any(x.get('shell') for x in skipped):continue
  if variant=='internal':
   if original_name in ('球壳1.SLDPRT','球壳小.SLDPRT','头部底面.SLDPRT'):continue
  if variant=='exterior' and original_name=='球壳1.SLDPRT':
   # Show one CAD outer sphere surface; two instances duplicate much of it.
   if any(x.get('shell') for x in skipped):continue
   skipped.append({'shell':True})
  xyzw=instance['rotation_xyzw'];q=[xyzw[3],*xyzw[:3]]
  Qi=quaternion_matrix(q)[:3,:3]
  translation=np.asarray(instance['position_mm'],dtype=float)*.001
  M=np.eye(4)
  M[:3,:3]=R @ Qi
  M[:3,3]=R@(translation-C)+[0,0,0.1]
  # Keep geometry referenced by name, repeated mesh instances with 55 CAD placements.
  if label not in scene.geometry:scene.geometry[label]=visual_class[label].copy()
  scene.graph.update(frame_from='world',frame_to=instance['id'],matrix=M,geometry=label)
  n+=1
 path=output/f'bb8-hweigo-{variant}-original-cad.glb'
 path.write_bytes(trimesh.exchange.gltf.export_glb(scene, include_normals=True))
 b=scene.bounds
 print('GLB',variant,'instances',n,'unique',len(scene.geometry),'bounds',b,'size',path.stat().st_size)
 return {'name':path.name,'instances':n,'meshes':len(scene.geometry),'bounds_m':b.tolist(),'size':path.stat().st_size}
report={'schema':'alfred-bb8-original-cad-glb/v1','source':'HWeigo/BB8_Robot pinned 7318411c','status':'STATIC_CAD_VISUALIZATION_NOT_PHYSICS','unit':'meters','source_sphere_diameter_m':0.2,'render_policy':'Source CAD placements; y-negative moved to z-up','variants':[build(x) for x in ['exterior','internal']]}
(output/'glb-export-report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
