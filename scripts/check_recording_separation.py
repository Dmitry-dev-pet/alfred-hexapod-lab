"""Fail publication if standalone recording payloads are bundled into the Site."""
from pathlib import Path
import json

root=Path(__file__).resolve().parents[1]
payloads=list((root/'dist').rglob('*.robotreplay'))
assert not payloads, 'Move recording files to public external storage: '+str(payloads)
catalog=json.loads((root/'dist/replay/catalog.json').read_text())
for entry in catalog['entries']:
    url=entry['recording']['url']
    assert url.startswith('https://') and '.chatgpt.site/' not in url, entry['id']
print('PASS: standalone files are external; Site contains catalog metadata and the shared reader')
