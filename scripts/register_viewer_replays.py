#!/usr/bin/env python3
"""Atomic, repeatable registration of physical MuJoCo replays in Hexapod player.

Run after a *completed, evidence-backed* physics calculation and export. This
never creates a fake gait, checks out CAD binaries, or triggers GitHub Actions.
It creates a catalog for review; publishing the files into the exact viewer
branch is a separate explicit step.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import re

ID=re.compile(r'^[a-z0-9-]{1,64}$')
HREF=re.compile(r'^/hexapod/(?:[a-z0-9-]+/)?$')
ALLOW={'.html','.js','.css','.json'}

def register(existing:dict,new_entries:list,docs:Path)->dict:
    assert existing.get('schema')=='hexapod-library/v1', 'Unexpected catalog schema'
    assert isinstance(existing.get('revision'),int), 'Missing catalog revision'
    old=list(existing['entries'])
    ids={x['id'] for x in old}; hrefs={x['href'] for x in old}
    paths={f['path'] for x in old for f in x.get('files',[])}
    changed=False
    for item in new_entries:
        sid=item['id']
        if not ID.fullmatch(sid) or not HREF.fullmatch(item['href']):raise ValueError('Invalid replay id/href')
        if sid in ids:
            prior=next(x for x in old if x['id']==sid)
            if prior!=item:raise ValueError('Immutable replay ID changed: '+sid)
            continue
        if item['href'] in hrefs:raise ValueError('Duplicate player route')
        if not (isinstance(item.get('duration_s'),(int,float)) and 0<item['duration_s']<=3600):raise ValueError('Invalid duration')
        if not (isinstance(item.get('files'),list) and item['files']):raise ValueError('Missing replay files')
        for row in item['files']:
            path=row['path'];p=Path(path)
            if p.is_absolute() or len(p.parts)>8 or not path.startswith('hexapod/') or any(x in ('','..','.') or x.startswith('.') for x in p.parts) or p.suffix not in ALLOW or path in paths:
                raise ValueError('Invalid/duplicate path: '+path)
            actual=docs/p
            if not actual.is_file() or actual.is_symlink():raise FileNotFoundError(actual)
            b=actual.read_bytes()
            if len(b)!=row['size'] or len(b)>8_000_000 or hashlib.sha256(b).hexdigest()!=row['sha256']:
                raise ValueError('Replay content differs from manifest: '+path)
            paths.add(path)
        page=docs/item['href'].lstrip('/')/'index.html'
        if not page.is_file() or page.is_symlink():raise ValueError('Missing player page for '+sid)
        ids.add(sid);hrefs.add(item['href']);old.append(item);changed=True
    if len(old)>120:raise ValueError('Too many catalog entries')
    return {**existing,'revision':existing['revision']+(1 if changed else 0),'entries':old}

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--catalog',required=True,type=Path)
    ap.add_argument('--entries',required=True,type=Path)
    ap.add_argument('--docs',required=True,type=Path)
    ap.add_argument('--output',required=True,type=Path)
    a=ap.parse_args()
    merged=register(json.loads(a.catalog.read_text()),json.loads(a.entries.read_text()),a.docs)
    data=json.dumps(merged,ensure_ascii=False,indent=2)+'\n'
    a.output.write_text(data)
    print(json.dumps({'revision':merged['revision'],'entries':len(merged['entries'])}))

if __name__=='__main__':main()
