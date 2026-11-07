"""Offline E2E test: simulate a browser-click refresh fetching the three real files."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
SRC=ROOT/'tools/local_viewer_server.py'
spec=importlib.util.spec_from_file_location('hexapod_refresh',SRC)
server=importlib.util.module_from_spec(spec);spec.loader.exec_module(server)

class RefreshPipeline(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
  self.dir=Path(self.tmp.name);(self.dir/'hexapod').mkdir()
  (self.dir/'hexapod/index.html').write_text('<div>Old existing library</div>')
  full=json.loads((ROOT/'docs/hexapod/library.json').read_text())
  self.old=full['entries'][0]
  self.new=[x for x in full['entries'] if x['id'].startswith('sebaju-')]
  self.assertEqual(len(self.new),3)
  self.oldcatalog={'schema':'hexapod-library/v1','revision':34,'entries':[self.old]}
  self.newcatalog={'schema':'hexapod-library/v1','revision':35,'entries':[self.old,*self.new]}
  (self.dir/'hexapod/library.json').write_text(json.dumps(self.oldcatalog))
  self.commit='a'*40
  self.mapping={
   server.BRANCH_API:json.dumps({'commit':{'sha':self.commit}}).encode(),
   server.RAW_BASE+self.commit+'/docs/hexapod/library.json':json.dumps(self.newcatalog).encode()
  }
  for row in self.new:
   for f in row['files']:
    data=(ROOT/'docs'/f['path']).read_bytes()
    self.assertEqual(hashlib.sha256(data).hexdigest(),f['sha256'])
    self.mapping[server.RAW_BASE+self.commit+'/docs/'+f['path']]=data
  self.mocks=patch.multiple(server, ROOT=self.dir, LIBRARY=self.dir/'hexapod/library.json')
  self.mocks.start();self.addCleanup(self.mocks.stop)

 def fake_remote(self,url,limit):
  b=self.mapping[url]
  if len(b)>limit:raise ValueError('oversized')
  return b

 def test_three_replays_install_verified_and_old_library_survives(self):
  with patch.object(server,'remote_bytes',side_effect=self.fake_remote):res=server.refresh_library()
  self.assertEqual(res['added'],3)
  self.assertEqual(res['assets'],7)
  self.assertEqual(res['revision'],35)
  got=json.loads((self.dir/'hexapod/library.json').read_text())
  self.assertEqual([x['id'] for x in got['entries']],[self.old['id'],*(x['id'] for x in self.new)])
  for row in self.new:
   for f in row['files']:
    target=self.dir/f['path'];self.assertTrue(target.is_file())
    if target.suffix=='.html':
     html=target.read_text();self.assertIn('/vendor/three/build/three.module.js',html)
     self.assertNotIn('cdn.jsdelivr.net',html)
     self.assertNotEqual(hashlib.sha256(target.read_bytes()).hexdigest(),f['sha256'])
     self.assertEqual(hashlib.sha256(target.read_bytes()).hexdigest(),next(k for e in got['entries'] if e['id']==row['id'] for k in e['files'] if k['path']==f['path'])['local_sha256'])
    else:self.assertEqual(hashlib.sha256(target.read_bytes()).hexdigest(),f['sha256'])

 def test_corrupted_asset_fails_closed_before_catalog_replace(self):
  key=server.RAW_BASE+self.commit+'/docs/hexapod/data/sebaju-tip12-v1.json'
  self.mapping[key]+=b'X'
  with patch.object(server,'remote_bytes',side_effect=self.fake_remote):
   with self.assertRaises(ValueError):server.refresh_library()
  self.assertEqual(json.loads((self.dir/'hexapod/library.json').read_text()),self.oldcatalog)
  self.assertFalse((self.dir/'hexapod/sebaju-jump-v1/index.html').exists())

if __name__=='__main__':unittest.main()
