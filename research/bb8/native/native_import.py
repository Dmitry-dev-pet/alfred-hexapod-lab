"""Research-only CAD import; pinned HWeigo source, no public export."""
from pathlib import Path
import os,sys,json,hashlib,traceback,time
WORKSPACE=Path(os.environ.get('BB8_WORKSPACE','/mnt/data'))
sys.path.insert(0,str(WORKSPACE/'bb8-freecad-plugin/converter'))
import FreeCAD as App
import Part
import importSolidWorks as SW
from sw2fc import assembly

start=time.monotonic()
root=WORKSPACE/'bb8-hweigo-original'
asm=root/'Model/初装.SLDASM'
out=WORKSPACE/'bb8-hweigo-reconstructed';out.mkdir(exist_ok=True)
payload=asm.read_bytes()
sha1=hashlib.sha1(b'blob '+str(len(payload)).encode()+b'\x00'+payload).hexdigest()
assert sha1=='bcfead6e92329575aa474fb4be05c24191ac5294'
tree=assembly.read_tree(str(asm)); leaves=list(tree.walk())
assert len(leaves)==55,len(leaves)
refs=[assembly.resolve_file(c.path,str(asm)) for c,m in leaves]
assert all(refs), [(c.name,c.path) for (c,m),r in zip(leaves,refs) if r is None]
print('ASSEMBLY TREE','instances',len(leaves),'unique',len(set(refs)),'hidden',sum(c.hidden for c,m in leaves),flush=True)
report={'schema':'alfred-bb8-source-assembly-import/v1','original_repo':'HWeigo/BB8_Robot','commit':'7318411cfbfbffdbd68f75e3734a50c3933969e5','original_assembly_git_sha':sha1,'source_cad_revisions_pinned':True,'freecad_version':'.'.join(App.Version()[:3]),'source_instances':len(leaves),'source_unique_files':len(set(refs)),'source_hidden_instances':sum(c.hidden for c,m in leaves),'gui':bool(App.GuiUp),'import_status':'started','shapes':[],'problems':[]}
try:
 doc=App.newDocument('HWeigo_BB8_Original')
 SW._import_assembly(str(asm),doc)
 doc.recompute()
 print('FREECAD IMPORT DONE in',round(time.monotonic()-start,1),'s','objects',len(doc.Objects),flush=True)
 features=[]
 for obj in doc.Objects:
  if obj.TypeId=='Part::Feature':
   shape=obj.Shape
   ext=[shape.BoundBox.XLength,shape.BoundBox.YLength,shape.BoundBox.ZLength]
   item={'name':obj.Name,'label':obj.Label,'type':obj.TypeId,'is_null':shape.isNull(), 'valid':bool(shape.isValid()) if not shape.isNull() else False, 'volume_mm3':round(shape.Volume,4) if not shape.isNull() else 0,'aabb_mm':[[round(x,4) for x in [shape.BoundBox.XMin,shape.BoundBox.YMin,shape.BoundBox.ZMin]],[round(x,4) for x in [shape.BoundBox.XMax,shape.BoundBox.YMax,shape.BoundBox.ZMax]]], 'extent_mm':[round(x,3) for x in ext]}
   features.append(obj);report['shapes'].append(item)
   if not item['valid'] or item['is_null']:report['problems'].append({'file':obj.Label,'error':'invalid/empty shape'})
 print('IMPORTED FEATURES',len(features),'valid',sum(x['valid'] for x in report['shapes']),'failed',len(report['problems']),flush=True)
 report['freecad_feature_count']=len(features)
 report['valid_feature_count']=sum(x['valid'] for x in report['shapes'])
 report['elapsed_s']=round(time.monotonic()-start,3)
 report['import_status']='completed' if len(features)>=len(leaves) and not report['problems'] else 'partial'
 fpath=out/'bb8-hweigo-original.FCStd';doc.saveAs(str(fpath))
 report['fcstd_file']=str(fpath);report['fcstd_bytes']=fpath.stat().st_size
 report['fcstd_sha256']=hashlib.sha256(fpath.read_bytes()).hexdigest()
 print('SAVED FCSTD',report['fcstd_bytes'],report['fcstd_sha256'],flush=True)
 for obj in features:
  shape=obj.Shape
  if obj.Label in ('球壳1','球壳小','轮子1','轮胎1','平衡车轮电机1','平衡车轮电机2','头部底面','SG90'):
   print('KEY PART',obj.Label,'volume',round(shape.Volume,2),'bb',[round(getattr(shape.BoundBox,k),2) for k in ['XMin','YMin','ZMin','XMax','YMax','ZMax']],flush=True)
except Exception as e:
 report['import_status']='error';report['exception']=repr(e);traceback.print_exc()
finally:
 (out/'cad_import_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
 print('REPORT',out/'cad_import_report.json','status',report['import_status'],flush=True)
 try: App.closeDocument(doc.Name)
 except: pass
