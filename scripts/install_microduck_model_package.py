#!/usr/bin/env python3
"""Install the model adapter into an existing Sites static export.

Keep the working replay branch/tablet's existing upstream loader unchanged until
the adapter PR is reviewed. Only the hosted Site uses the bundled model overlay.
"""
import argparse, hashlib, json, shutil
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
ROUTES=['/hexapod/model-package.js','/hexapod/microduck-standup-v1/app.js',
        '/hexapod/microduck-standup-v1/','/hexapod/microduck-standup-v1/index.html']

def install(site, package):
    ref=json.loads((ROOT/'data/microduck_model_package_ref.json').read_text())
    data=package.read_bytes()
    if len(data)!=ref['size'] or hashlib.sha256(data).hexdigest()!=ref['sha256']:
        raise ValueError('Wrong model package; Site unchanged')
    worker=site/'github-live-sw.js';source=worker.read_text()
    marker="  // The hosted UI must load immediately even if GitHub is slow/offline."
    if marker not in source: raise ValueError('Unknown Site bridge; Site unchanged')
    # A fixed reviewed Site adapter uses the bundled model. The rest of the
    # catalog remains pinned/verified by the existing GitHub bridge.
    if '// Hosted model package adapter v1.' not in source:
        source=source.replace(marker,"  // Hosted model package adapter v1.\n  if ("+json.dumps(ROUTES)+".includes(pathname)) return;\n"+marker)
    target=site/ref['url'].lstrip('/');target.parent.mkdir(parents=True,exist_ok=True)
    shutil.copyfile(package,target)
    for path in ROUTES:
        if path.endswith('/'): continue
        target=site/path.lstrip('/');target.parent.mkdir(parents=True,exist_ok=True)
        content=(ROOT/'docs'/path.lstrip('/')).read_text()
        if path.endswith('.html'):
            content=content.replace('https://cdn.jsdelivr.net/npm/three@0.169.0/build/three.module.js','/vendor/three/build/three.module.js').replace('https://cdn.jsdelivr.net/npm/three@0.169.0/examples/jsm/','/vendor/three/examples/jsm/')
            content=content.replace('</head>','<script src="/github-live-boot.js"></script><script src="/webgl-fallback.js"></script></head>')
        target.write_text(content)
    worker.write_text(source)
    version=site/'hosted-version.json';meta=json.loads(version.read_text())
    meta['model_packages']=[ref]
    meta['mesh_policy']='original-pinned-upstream; MicroDuck bundled under CC-BY-NC-SA-4.0'
    for path in ['github-live-sw.js','hexapod/model-package.js','hexapod/microduck-standup-v1/app.js','hexapod/microduck-standup-v1/index.html']:
        meta.setdefault('ui_asset_sha256',{})[path]=hashlib.sha256((site/path).read_bytes()).hexdigest()
    version.write_text(json.dumps(meta,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'package_bytes':len(data),'model':ref['id'],'meshes':ref['mesh_count'],'instances':ref['instance_count']}))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--site',type=Path,required=True)
    p.add_argument('--package',type=Path,required=True)
    a=p.parse_args();install(a.site,a.package)
