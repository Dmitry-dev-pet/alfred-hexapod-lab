"""Export source-derived FreeCAD meshes + original-instance placements to private cache.
Research only. No physical joints, no site publication.
"""
from pathlib import Path
import os
from collections import Counter
from math import isfinite
import FreeCAD as App
import MeshPart
import json,hashlib,sys, time
WORKSPACE=Path(os.environ.get('BB8_WORKSPACE','/mnt/data'))
sys.path.insert(0,str(WORKSPACE/'bb8-freecad-plugin/converter'))
from sw2fc import assembly

start=time.monotonic()
root=WORKSPACE/'bb8-hweigo-original/Model'
asm=root/'初装.SLDASM'
assembled=assembly.read_tree(str(asm)); leaf=list(assembled.walk())
doc=App.openDocument(str(WORKSPACE/'bb8-hweigo-reconstructed/bb8-hweigo-original.FCStd'))
items=[o for o in doc.Objects if o.TypeId=='Part::Feature']
assert len(leaf)==len(items)==55
out=WORKSPACE/'bb8-hweigo-reconstructed/meshes';out.mkdir(exist_ok=True)
scene={'schema':'alfred-bb8-source-cad-scene/v1','status':'SOURCE_CAD_VISUAL_GEOMETRY_ONLY_NOT_DYNAMICS','source_repo':'HWeigo/BB8_Robot','source_commit':'7318411cfbfbffdbd68f75e3734a50c3933969e5','assembly_git_blob_sha1':'bcfead6e92329575aa474fb4be05c24191ac5294','cad_native_length_units':'mm','world_transform_note':'Z axis in source assembly is not world up; visual root orientation rotates -90deg about X to make -Y source upward','meshes':{},'instances':[],'diagnostics':{'non_solid':[]}}
for k,((component,matrix),o) in enumerate(zip(leaf,items)):
 # FreeCAD auto-renames duplicate labels (e.g. 联轴器1 -> 联轴器001); import order follows the original assembly tree.
 part_name=Path(assembly.resolve_file(component.path,str(asm))).name
 key=part_name.casefold()
 if key not in scene['meshes']:
  shape=o.Shape.copy()
  mesh=MeshPart.meshFromShape(Shape=shape, LinearDeflection=0.35, AngularDeflection=0.35, Relative=False)
  name=f'part-{len(scene["meshes"]):02d}.stl'
  path=out/name
  mesh.write(str(path))
  blob=path.read_bytes()
  if len(blob)<100: raise ValueError(f'Missing STL {name}')
  source_path=root/part_name
  source_bytes=source_path.read_bytes()
  source_git=hashlib.sha1(b'blob '+str(len(source_bytes)).encode()+b'\0'+source_bytes).hexdigest()
  scene['meshes'][key]={'file':name,'source_file':part_name,'source_git_blob_sha':source_git,'sha256':hashlib.sha256(blob).hexdigest(),'bytes':len(blob),'points':mesh.CountPoints,'triangles':mesh.CountFacets,'solid_is_valid':o.Shape.isValid(),'source_shape_volume_mm3':round(o.Shape.Volume,4),'source_solid_count':len(o.Shape.Solids)}
  if len(o.Shape.Solids)==0:scene['diagnostics']['non_solid'].append(part_name)
 gp=o.getGlobalPlacement()
 q=gp.Rotation.Q
 pos=[float(gp.Base.x),float(gp.Base.y),float(gp.Base.z)]
 if not all(isfinite(x) for x in [*pos,*q]): raise ValueError('invalid placement')
 scene['instances'].append({'id':f'c{k:02d}','part_key':key,'label':o.Label,'hidden_in_original':bool(component.hidden),'position_mm':pos,'rotation_xyzw':[float(x) for x in q],'source_component_path':component.path})
 if k%10==0:print('MESH',k,'of 55','unique',len(scene['meshes']),flush=True)
scene['mesh_count']=len(scene['meshes']);scene['instance_count']=len(scene['instances'])
(scene_path:=out/'scene.json').write_text(json.dumps(scene,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print('DONE',scene['mesh_count'],'meshes',scene['instance_count'],'instances',sum(x['triangles'] for x in scene['meshes'].values()),'triangles','non_solid',scene['diagnostics']['non_solid'],'elapsed',round(time.monotonic()-start,2),flush=True)
App.closeDocument(doc.Name)
