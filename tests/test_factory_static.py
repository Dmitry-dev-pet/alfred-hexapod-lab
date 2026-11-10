import unittest,json,re,hashlib
from pathlib import Path
from html.parser import HTMLParser
ROOT=Path(__file__).resolve().parents[1]
class Collector(HTMLParser):
 def __init__(self):super().__init__();self.ids=set();self.urls=[]
 def handle_starttag(self,tag,attrs):
  attr=dict(attrs)
  if attr.get('id'):self.ids.add(attr['id'])
  if tag=='script' and attr.get('src'):self.urls.append(attr['src'])
class FactoryStatic(unittest.TestCase):
 def test_static_player_contract(self):
  path=ROOT/'docs/hexapod/freenove-original-v7/index.html';html=path.read_text()
  d=Collector();d.feed(html)
  for key in ['viewer','factory-mode','factory-gait','factory-tempo','pilot-pad','pilot-thumb','pose-height','pose-roll','pose-pitch','pose-yaw','factory-stop','pilot-ccw','pilot-cw','reset-camera','play','seek','speed']:
   self.assertIn(key,d.ids)
  self.assertIn('/hexapod/player-shell.js',d.urls)
  self.assertIn('/hexapod/freenove-original-v7/app.js',d.urls)
  self.assertIn('/hexapod/live-v5-motions/',html)
 def test_no_runtime_physics(self):
  a=(ROOT/'docs/hexapod/freenove-original-v7/app.js').read_text();b=(ROOT/'docs/hexapod/freenove-original-v7/factory-controller.js').read_text()
  for word in ['WebSocket(','mj_step(','mujoco','drake','/api/physics','fetch(\'https']:
   # model notes can mention MuJoCo, but no actual imports / APIs.
   if word in ('mujoco','drake'):continue
   self.assertNotIn(word,a+b)
  for text in ['cad-rig-live-v1-slow.json','live-v4-gaits-kit.json','freenove-original-v7-kit.json','factory-controller.js']:
   self.assertIn(text,a+b)
 def test_data_provenance_and_limits(self):
  kit=json.loads((ROOT/'docs/hexapod/data/freenove-original-v7-kit.json').read_text())
  self.assertEqual(set(kit['gaits']),{'tripod','wave'})
  self.assertEqual(kit['cad_adapter']['neutral_root_plus_mm'],15)
  self.assertEqual(kit['factory_commands']['attitude_clamp_deg'],15)
  self.assertEqual(kit['factory_commands']['position_z_clamp_mm'],20)
  self.assertGreater(kit['max_abs_joint_rad'],1.7)
  self.assertLess(kit['max_ik_error_mm'],.8)
if __name__=='__main__':unittest.main()
