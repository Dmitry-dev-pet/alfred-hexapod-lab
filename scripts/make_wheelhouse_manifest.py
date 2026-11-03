"""SHA-256 manifest for wheelhouse files; no network dependency."""
import argparse
import hashlib
import json
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument('--check',action='store_true')
p.add_argument('wheelhouse',type=Path)
p.add_argument('manifest',type=Path)
a=p.parse_args()
if not a.wheelhouse.is_dir():
    p.error('wheelhouse directory does not exist')
files={w.name:hashlib.sha256(w.read_bytes()).hexdigest() for w in sorted(a.wheelhouse.glob('*.whl'))}
if not files: p.error('wheelhouse has no wheels')
if a.check:
    expected=json.loads(a.manifest.read_text())['sha256']
    if expected!=files: raise SystemExit('wheelhouse checksum verification FAILED')
    print('OK: checked',len(files),'wheels')
else:
    a.manifest.write_text(json.dumps({'sha256':files},indent=2)+'\n')
    print('Manifest created:',a.manifest)