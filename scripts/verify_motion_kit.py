"""Verifies V5 standalone kit metadata without the original mesh files."""
import hashlib,json,sys
from pathlib import Path
p=Path(sys.argv[1]);d=json.loads(p.read_text())
assert d['format']=='fnk0052-cad30-offline-motions-v5'
assert d['no_realtime_physics'] is True and d['physical_guarantee'] is False
assert len(d['pose_axes'])==6 and len(d['leg_axes'])==6
for axis,poses in d['pose_axes'].items():
 assert set(poses)=={'pos','neg'}
 assert len(poses['pos'])==len(poses['neg'])==20
print('V5 motion metadata PASS',hashlib.sha256(p.read_bytes()).hexdigest())
