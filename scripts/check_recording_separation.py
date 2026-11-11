"""Reject bundled motion payloads before publishing the existing Site."""
from pathlib import Path
import argparse
import json
import re

def check(root):
    dist=root/'dist'
    payloads=list(dist.rglob('*.robotreplay'))
    for prefix in ('assets/robot-replays','assets/robot-motions','hexapod/data'):
        payloads.extend(p for p in (dist/prefix).rglob('*') if p.is_file())
    assert not payloads, 'Recording payloads must stay in GitHub: '+str(payloads)
    catalog=json.loads((dist/'replay/catalog.json').read_text())
    for entry in catalog['entries']:
        url=entry['recording']['url']
        assert re.match(r'^https://raw\.githubusercontent\.com/Dmitry-dev-pet/alfred-hexapod-lab/[a-f0-9]{40}/',url), entry['id']
    store=json.loads((dist/'robots/recording-assets.json').read_text())
    assert store['schema']=='robot-recording-store/v1'
    assert re.fullmatch(r'[a-f0-9]{40}',store['commit'])
    assert store['repository']=='Dmitry-dev-pet/alfred-hexapod-lab'
    assert store['prefix']=='recordings/site-v33'
    assert len({f['path'] for f in store['files']})==len(store['files'])
    print('PASS: motion payloads are external immutable GitHub files; Site contains reader and metadata')

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
    check(parser.parse_args().root)
