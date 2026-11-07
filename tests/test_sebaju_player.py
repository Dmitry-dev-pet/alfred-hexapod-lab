"""Regression gates for the SeBaJu physical-replay publication. Offline, no GPU/web."""
from __future__ import annotations
import base64
import gzip
import hashlib
import json
import os
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
import publish_sebaju_player as pub
import register_viewer_replays as registry
EVIDENCE=Path(os.environ.get('SEBAJU_EVIDENCE_DIR', str(ROOT/'research/sebaju/evidence')))
EVIDENCE_AVAILABLE=(EVIDENCE/'canonical_jump/physical_trace.json').is_file()
DOCS=ROOT/'docs'

class SeBaJuViewerTests(unittest.TestCase):
 @unittest.skipUnless(EVIDENCE_AVAILABLE, "Supply SEBAJU_EVIDENCE_DIR to verify raw original traces")
 def test_three_archive_payloads_match_exact_source_frames(self):
  for case,run,_,_,_ in pub.EXPERIMENTS:
   with self.subTest(case=case):
    raw_trace=(EVIDENCE/run/'physical_trace.json').read_bytes()
    pack=json.loads((DOCS/'hexapod/data'/f'{case}.json').read_text())
    raw=gzip.decompress(base64.b64decode(pack['base64']))
    self.assertEqual(len(raw),pack['raw_bytes']);self.assertEqual(hashlib.sha256(raw).hexdigest(),pack['raw_sha256'])
    data=json.loads(raw)
    trace=json.loads(raw_trace)
    self.assertEqual(pack['source_trace_sha256'],hashlib.sha256(raw_trace).hexdigest())
    self.assertEqual(len(data['rows']),len(trace['frames']))
    self.assertEqual(data['source_commit'],pub.SOURCE)
    for row,src in zip(data['rows'],trace['frames']):
     self.assertEqual(row[0],src['t'])
     self.assertEqual(row[1:14],src['qpos'])
     self.assertEqual(row[14],src['tilt_deg'])
     self.assertEqual(row[15:17],[int(x) for x in src['grounded']])
     self.assertEqual(row[17:19],src['wheel_vertical_normal_N'])
     self.assertEqual(row[19],sum(1<<pub.GEOM_INDEX[g] for g in set(src['ground_contact_geoms'])))
     self.assertEqual(row[20],src['wheel_gap_min_m'])
     self.assertEqual(len(row),21)

 def test_new_catalog_entries_are_immutable_and_hashed(self):
  c=json.loads((DOCS/'hexapod/library.json').read_text())
  self.assertEqual(c['schema'],'hexapod-library/v1')
  self.assertGreaterEqual(c['revision'],35)
  self.assertGreaterEqual(len(c['entries']),56)
  self.assertEqual(len({x['id'] for x in c['entries']}),len(c['entries']))
  self.assertEqual(len({x['href'] for x in c['entries']}),len(c['entries']))
  entries=[e for e in c['entries'] if e['id'].startswith('sebaju-')]
  self.assertEqual([e['id'] for e in entries],[x[0] for x in pub.EXPERIMENTS]);self.assertEqual([e['status'] for e in entries],['passed','failed','failed'])
  for e in entries:
   for file in e['files']:
    p=DOCS/file['path'];self.assertTrue(p.exists());self.assertEqual(p.stat().st_size,file['size'])
    self.assertEqual(hashlib.sha256(p.read_bytes()).hexdigest(),file['sha256'])
  self.assertEqual(registry.register(c,entries,DOCS),c)

 def test_installed_player_server_accepts_new_catalog(self):
  import importlib.util
  sp=importlib.util.spec_from_file_location('hexapod_local_viewer',ROOT/'tools/local_viewer_server.py')
  m=importlib.util.module_from_spec(sp);sp.loader.exec_module(m)
  m.validate_catalog(json.loads((DOCS/'hexapod/library.json').read_text()))

 def test_origins_can_be_verified_without_publishing_upstream_assets(self):
  self.assertFalse((DOCS/'hexapod'/'SeBaJu_BOT(II).xml').exists())
  self.assertFalse((DOCS/'hexapod'/'SeBaJu_Ctrl_BalJumpAchieve.py').exists())
  js=(DOCS/'hexapod/sebaju-player.js').read_text()
  self.assertNotIn('mj_step(',js)
  self.assertIn('rowAt',js)
  self.assertIn('DecompressionStream',js)
  self.assertIn('SHA-256',js)
  for case,_,_,_,_ in pub.EXPERIMENTS:
   html=(DOCS/'hexapod'/case/'index.html').read_text()
   for s in ['id="viewer"','id="play"','id="seek"','id="speed"','player-shell.js','sebaju-player.js',f'data-case-id="{case}"']:
    self.assertIn(s,html)

 @unittest.skipUnless(EVIDENCE_AVAILABLE, "Supply SEBAJU_EVIDENCE_DIR to verify raw original traces")
 def test_modified_source_is_rejected(self):
  pack,_=pub.make_data('sebaju-jump-v1',EVIDENCE/'canonical_jump')
  self.assertEqual(pack['frame_count'],651)
  t=json.loads((EVIDENCE/'canonical_jump/physical_trace.json').read_text());r=json.loads((EVIDENCE/'canonical_jump/receipt.json').read_text())
  t['frames'][17]['qpos'][7]=float('nan')
  with self.assertRaises(AssertionError):pub.check_trace(t,r,'sebaju-jump-v1')

if __name__=='__main__':unittest.main()
