"""Structural assertions for new immutable V9 player route; no browser physics."""
from pathlib import Path
from html.parser import HTMLParser
import hashlib
import unittest

R=Path(__file__).resolve().parents[1]
PAGE=R/'docs/hexapod/freenove-combined-v9'
class Tags(HTMLParser):
 def __init__(self):super().__init__();self.ids=[];self.scripts=[];self.styles=[]
 def handle_starttag(self,tag,attrs):
  a=dict(attrs)
  if a.get('id'):self.ids.append(a['id'])
  if tag=='script' and a.get('src'):self.scripts.append(a['src'])
  if tag=='link' and a.get('rel')=='stylesheet':self.styles.append(a['href'])

class ViewerContract(unittest.TestCase):
 def test_two_pads_no_global_walk_body_switch(self):
  h=Tags();h.feed((PAGE/'index.html').read_text())
  self.assertEqual(len(h.ids),len(set(h.ids)))
  for name in ['pilot-pad','pilot-thumb','pilot-pad-right','pilot-thumb-right','steer-mode','right-mode','factory-gait','factory-tempo','pose-height','pose-yaw','factory-stop','reset-pose','reset-camera','viewer','play','seek','speed']:
   self.assertIn(name,h.ids)
  self.assertNotIn('factory-mode',h.ids)
  self.assertNotIn('pilot-ccw',h.ids)
  self.assertIn('/hexapod/freenove-combined-v9/app.js',h.scripts)
  self.assertIn('/hexapod/player-shell.js',h.scripts)
  self.assertIn('/hexapod/freenove-combined-v9/overlay.css',h.styles)
  self.assertIn('/hexapod/freenove-dual-v8/dual-stick.css',h.styles)
 def test_walk_never_blocked_by_right_mode(self):
  app=(PAGE/'app.js').read_text()
  self.assertIn('pilot.drive(i.drive.forward,i.drive.left,i.drive.turn)',app)
  self.assertIn('pilot.setRight(i.right)',app)
  self.assertIn("rightSelect.addEventListener('change'",app)
  self.assertIn("clearPad('right')",app)
  self.assertNotIn("pilot?.setMode",app)
  self.assertIn('new CombinedPilot(factory,ref,headKit)',app)
  self.assertIn("fetch('../data/live-v5-motion-kit.json'",app)
 def test_no_runtime_physics_and_source_provenance(self):
  src='\n'.join((PAGE/p).read_text() for p in ['app.js','combo-controller.js'])
  for text in ['WebSocket(','mj_step(',"fetch('https",'new Worker(','mujoco.Mj']:
   self.assertNotIn(text,src)
  v7=(R/'docs/hexapod/freenove-original-v7/factory-controller.js').read_bytes()
  self.assertEqual(hashlib.sha256(v7).hexdigest(),'336e7fe8e7a82904075c969498b62a27714986fba0e3ed303635bb98bddb9b0d')
  self.assertIn('FactoryPilot',src)
  self.assertIn('head_joint_signs',src)
 def test_assets_bounded(self):
  for name in ('index.html','app.js','combo-controller.js','overlay.css'):
   with self.subTest(name=name):
    self.assertGreater((PAGE/name).stat().st_size,100)
    self.assertLess((PAGE/name).stat().st_size,50000)

if __name__=='__main__':unittest.main()
