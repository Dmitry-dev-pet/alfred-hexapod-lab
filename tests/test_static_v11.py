"""Prevent previous V9 boot failure, root spin rendering, and missing dependencies."""
from pathlib import Path
from html.parser import HTMLParser
import re,unittest
ROOT=Path(__file__).resolve().parents[1]
H=ROOT/'docs/hexapod'
class Tags(HTMLParser):
 def __init__(self):super().__init__();self.scripts=[];self.css=[];self.ids=[]
 def handle_starttag(self,tag,attrs):
  o=dict(attrs)
  if 'id' in o:self.ids.append(o['id'])
  if tag=='script' and 'src' in o:self.scripts.append(o['src'])
  if tag=='link' and o.get('rel')=='stylesheet':self.css.append(o['href'])
class Static(unittest.TestCase):
 def test_import_dependencies(self):
  v=Tags();v.feed((H/'freenove-grounded-v11/index.html').read_text())
  self.assertIn('/hexapod/freenove-grounded-v11/app.js',v.scripts)
  self.assertEqual(len(v.ids),len(set(v.ids)))
  seen=set()
  def walk(p):
   p=p.resolve();self.assertTrue(p.is_file(),str(p))
   if p in seen:return
   seen.add(p)
   for name in re.findall(r"\b(?:import|export)\s+(?:[^;]*?\s+from\s+)?[\"']([^\"']+)[\"']",p.read_text()):
    if name.startswith('.'):
     q=(p.parent/name).resolve();self.assertTrue(q.is_relative_to(ROOT.resolve()));walk(q)
    else:self.assertIn(name,('three','three/addons/controls/OrbitControls.js','three/addons/loaders/STLLoader.js'))
  walk(H/'freenove-grounded-v11/app.js')
  self.assertGreaterEqual(len(seen),6)
  for name in v.css:
   self.assertTrue(name in ('/hexapod/player-shell.css','/hexapod/live-v2-offline/pilot.css') or (ROOT/'docs'/name.lstrip('/')).is_file(),name)
 def test_stance_lock_not_blind_root_rotation(self):
  s=(H/'freenove-grounded-v11/app.js').read_text()
  self.assertIn('groundLock.step(toes)',s)
  self.assertIn('PlantedCombinedPilot',s)
  self.assertNotIn('new THREE.Vector3(s.x,s.y,0)',s)
  self.assertIn('pose?.rootOffsetWorld',s)
  self.assertIn('planted-body-yaw-kit.json',s)
 def test_error_display(self):
  s=(H/'freenove-grounded-v11/index.html').read_text()
  self.assertIn("window.addEventListener('error'",s)
  self.assertIn('pilot-load',s)
 def test_no_runtime_physics(self):
  s='\n'.join((H/'freenove-grounded-v11'/n).read_text() for n in ('app.js','planted-controller.js','ground-lock.js'))
  for bad in ('mj_step(', 'WebSocket(', '/api/physics','new Worker('): self.assertNotIn(bad,s)
if __name__=='__main__':unittest.main()
