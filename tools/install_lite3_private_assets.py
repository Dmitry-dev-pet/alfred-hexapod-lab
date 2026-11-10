#!/usr/bin/env python3
"""Optional permanent private/offline Lite3 STL install into existing Termux viewer.

One-time verified download from original LuwuDynamics GitHub at pinned commit.
Never commits the vendor STL meshes or changes existing CAD30 files.
The viewer can also securely fetch/cache the same verified meshes in-browser.
"""
from __future__ import annotations
import argparse,concurrent.futures,hashlib,json,os,re,shutil,tempfile,urllib.request
from pathlib import Path

COMMIT='447945fd40fb5585271dce4ea8cc82567916032a'
SOURCE=('https://raw.githubusercontent.com/LuwuDynamics/luwu_mjlab/'+COMMIT+'/'
        'src/assets/robots/lite3/xmls/assets/')


def sha256(path):
 with path.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()


def install(viewer:Path,manifest:dict):
 viewer=viewer.expanduser().resolve()
 if not (viewer/'OFFLINE_MANIFEST.json').is_file() or not (viewer/'hexapod/library.json').is_file():
  raise RuntimeError('Existing Hexapod Lab viewer installation not found at '+str(viewer))
 if manifest.get('schema')!='xgo-lite3-original-local-stl-assets-v1' or manifest.get('commit')!=COMMIT:
  raise RuntimeError('Unexpected pinned Lite3 asset version')
 items=manifest.get('meshes',[])
 if len(items)!=17 or len({x.get('mesh') for x in items})!=17:
  raise RuntimeError('Expected 17 unique original STL files')
 dest=viewer/'assets/lite3-original'
 if dest.is_symlink() or (dest.exists() and not dest.is_dir()):
  raise RuntimeError('Unsafe destination: '+str(dest))
 dest.parent.mkdir(parents=True,exist_ok=True)
 with tempfile.TemporaryDirectory(prefix='.lite3-private-download-',dir=dest.parent) as tmp:
  stage=Path(tmp)
  def get(spec):
   name=spec['mesh']
   if not re.fullmatch('[A-Za-z][A-Za-z0-9_]*\\.STL',name):raise ValueError('Invalid STL filename')
   path=dest/name
   if path.exists():
    if path.is_symlink() or sha256(path)!=spec['sha256'] or path.stat().st_size!=spec['size']:
     raise RuntimeError('Existing private mesh has wrong SHA; manual review needed: '+name)
    return {'name':name,'source':'cached'}
   req=urllib.request.Request(SOURCE+name,headers={'User-Agent':'Alfred-Hexapod-Lite3-Private-Install/1.0'})
   with urllib.request.urlopen(req,timeout=90) as res:
    data=res.read(spec['size']+1)
   if len(data)!=spec['size'] or hashlib.sha256(data).hexdigest()!=spec['sha256']:
    raise RuntimeError('Source STL checksum mismatch: '+name)
   (stage/name).write_bytes(data)
   return {'name':name,'source':'downloaded'}
  with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
   results=list(pool.map(get,items))
  dest.mkdir(exist_ok=True)
  for entry in results:
   stagefile=stage/entry['name']
   if stagefile.is_file():
    if (dest/entry['name']).exists():raise RuntimeError('Concurrent asset modification')
    os.replace(stagefile,dest/entry['name'])
  (dest/'LITE3_PRIVATE_MANIFEST.json').write_text(json.dumps({
     'vendor_commit':COMMIT,'meshes':items,'privacy':'local private storage only',
     'note':'Source vendor license preserved; this installation is not a public redistribution.'
  },ensure_ascii=False,indent=2)+'\n')
 return {'installed_at':str(dest),'files':len(results),'downloaded':sum(x['source']=='downloaded' for x in results),
         'total_bytes':sum(x['size'] for x in items)}


def main():
 parser=argparse.ArgumentParser()
 parser.add_argument('--viewer',type=Path,default=Path.home()/'hexapod-viewer-local')
 parser.add_argument('--manifest',type=Path,default=None)
 args=parser.parse_args()
 if args.manifest:
  manifest=json.loads(args.manifest.read_text())
 else:
  import urllib.request
  url=('https://raw.githubusercontent.com/Dmitry-dev-pet/alfred-hexapod-lab/'
       'feature/android-local-offline-viewer-20261104/docs/hexapod/lite3-original-v1/asset-manifest.json')
  with urllib.request.urlopen(url,timeout=30) as res:manifest=json.load(res)
 print(json.dumps(install(args.viewer,manifest),ensure_ascii=False,indent=2))

if __name__=='__main__':main()
