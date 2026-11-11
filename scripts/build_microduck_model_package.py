#!/usr/bin/env python3
"""Reproducibly package pinned MicroDuck STL bytes for one browser download.

The generated model is a CC BY-NC-SA 4.0 data artifact (not our code licence).
Keep it outside this source repository; publish to the existing Site's assets.
"""
from __future__ import annotations
import argparse, base64, concurrent.futures, gzip, hashlib, json
from pathlib import Path
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
PIN = 'bae39f5900b318f7e87fc485c30a31213cfa14eb'
UPSTREAM = f'https://raw.githubusercontent.com/pollen-robotics/microduck_rl/{PIN}/'

def checked(data, spec):
    if len(data) != spec['size'] or hashlib.sha256(data).hexdigest() != spec['sha256']:
        raise ValueError('Original STL integrity failure: '+spec['file'])
    return data

def build(manifest, cache, notice):
    def one(item):
        name, spec = item
        file = cache / spec['file']
        if file.exists():
            data = checked(file.read_bytes(), spec)
        else:
            with urllib.request.urlopen(UPSTREAM+manifest['root']+spec['file'], timeout=45) as r:
                data = checked(r.read(), spec)
            file.write_bytes(data)
        return name, base64.b64encode(gzip.compress(data, compresslevel=9, mtime=0)).decode('ascii')
    cache.mkdir(parents=True, exist_ok=True)
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        meshes = dict(pool.map(one, sorted(manifest['meshes'].items())))
    return {'schema':'robot-model-package/v1', 'id':'microduck-original',
            'encoding':'gzip-stl-base64', 'manifest':manifest, 'meshes':meshes,
            'license':{'spdx':'CC-BY-NC-SA-4.0',
                       'url':'https://creativecommons.org/licenses/by-nc-sa/4.0/',
                       'source':UPSTREAM+'LICENSE-MODELS.md', 'notice':notice,
                       'changes':'Original STL bytes gzip-compressed without geometry changes; manifest and instance links bundled by Alfred Hexapod Lab.'}}

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--cache', type=Path, required=True)
    ap.add_argument('--descriptor', type=Path, required=True)
    a=ap.parse_args()
    manifest=json.loads((ROOT/'docs/hexapod/microduck-standup-v1/asset-manifest.json').read_text())
    if manifest['revision'] != PIN or manifest['repo'] != 'pollen-robotics/microduck_rl':
        raise ValueError('Unreviewed model source')
    with urllib.request.urlopen(UPSTREAM+'LICENSE-MODELS.md', timeout=30) as r:
        notice=r.read().decode('utf-8')
    data=json.dumps(build(manifest,a.cache,notice),sort_keys=True,separators=(',',':')).encode()
    sha=hashlib.sha256(data).hexdigest()
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_bytes(data)
    descriptor={'schema':'robot-model-package-ref/v1','id':'microduck-original',
                'url':f'/assets/robot-models/microduck-original-{sha}.json',
                'sha256':sha,'size':len(data),'source_revision':PIN,
                'mesh_count':len(manifest['meshes']),'instance_count':len(manifest['visible_geoms']),
                'license_spdx':'CC-BY-NC-SA-4.0'}
    a.descriptor.write_text(json.dumps(descriptor,indent=2)+'\n')
    print(json.dumps(descriptor))

if __name__ == '__main__': main()
