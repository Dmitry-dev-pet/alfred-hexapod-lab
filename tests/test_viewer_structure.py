import unittest
from html.parser import HTMLParser
from pathlib import Path
class Collect(HTMLParser):
 def __init__(self):super().__init__();self.ids=set();self.modes=set();self.urls=[]
 def handle_starttag(self,tag,attrs):
  attrs=dict(attrs)
  if 'id' in attrs:self.ids.add(attrs['id'])
  if 'data-mode' in attrs:self.modes.add(attrs['data-mode'])
  if tag=='script' and attrs.get('src'):self.urls.append(attrs['src'])
class UI(unittest.TestCase):
 def test_modes_controls_and_existing_shell(self):
  file=Path(__file__).resolve().parents[1]/'docs/hexapod/live-v5-motions/index.html'
  d=Collect();d.feed(file.read_text())
  for id in ['pilot-mode','pilot-pad','pilot-gait','pilot-stride','pilot-gain','pilot-ccw','pilot-cw','pilot-leg','pilot-lift','pose-height','pose-roll','pose-pitch','pose-yaw','pilot-stop','reset-camera','play','seek','speed']:
   self.assertIn(id,d.ids)
  self.assertEqual(d.modes,{'walk','body','leg','head'})
  self.assertIn('/hexapod/player-shell.js',d.urls)
  self.assertIn('/hexapod/live-v5-motions/app.js',d.urls)
 def test_app_uses_v4_tables_no_server_dynamics(self):
  s=(Path(__file__).resolve().parents[1]/'docs/hexapod/live-v5-motions/app.js').read_text()
  self.assertIn("fetch('../data/live-v4-gaits-kit.json'",s)
  self.assertIn("fetch('../data/live-v5-motion-kit.json'",s)
  self.assertIn("fetch('../data/cad-rig-live-v1-slow.json'",s)
  self.assertIn('pilot.walk.x',s)
  self.assertNotIn('WebSocket(',s);self.assertNotIn('mj_step(',s);self.assertNotIn('remotePhysics',s)
if __name__=='__main__':unittest.main()
