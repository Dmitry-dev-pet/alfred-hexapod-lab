"""Register a completed standalone recording at its public URL; no player changes."""
import argparse
import gzip
import hashlib
import json
import pathlib
import re
from urllib.parse import urlsplit


def register(catalog_path, recording_path, public_url, recording_id):
    assert re.fullmatch(r'[a-z0-9-]{1,64}', recording_id), 'Invalid recording id'
    url=urlsplit(public_url)
    assert (public_url.startswith('/') and not public_url.startswith('//')) or url.scheme=='https'
    assert not url.username and not url.password and not url.fragment
    raw=pathlib.Path(recording_path).read_bytes()
    data=json.loads(gzip.decompress(raw) if raw[:2]==b'\x1f\x8b' else raw)
    assert data['schema']=='robot-replay/v1'
    entry={'id':recording_id,'title':data['metadata']['title'][:100],
           'detail':data['metadata'].get('description','')[:180],
           'duration_s':max(c['duration_s'] for c in data['clips']),
           'recording':{'schema':'robot-replay/v1','url':public_url,'size':len(raw),'sha256':hashlib.sha256(raw).hexdigest()}}
    path=pathlib.Path(catalog_path)
    catalog=json.loads(path.read_text()) if path.exists() else {'schema':'robot-replay-catalog/v1','revision':1,'entries':[]}
    assert catalog['schema']=='robot-replay-catalog/v1'
    old=next((e for e in catalog['entries'] if e['id']==recording_id),None)
    if old:
        assert old['recording']==entry['recording'], 'Recording id already points to different immutable bytes'
        return False
    catalog['entries'].append(entry)
    if len(catalog['entries'])>1:catalog['revision']+=1
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(catalog,ensure_ascii=False,indent=2)+'\n')
    return True


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--catalog',required=True);p.add_argument('--recording',required=True);p.add_argument('--url',required=True);p.add_argument('--id',required=True)
    a=p.parse_args();print('registered' if register(a.catalog,a.recording,a.url,a.id) else 'already registered')
