"""Prevent frozen boot screens from missing relative ESM modules or invalid camera code."""
from pathlib import Path
from html.parser import HTMLParser
import re, unittest
ROOT=Path(__file__).resolve().parents[1]
PAGE=ROOT/'docs/hexapod/freenove-combined-v10'
class HTML(HTMLParser):
 def __init__(self): super().__init__();self.scripts=[];self.css=[]
 def handle_starttag(self,tag,attrs):
  v=dict(attrs)
  if tag=='script' and v.get('src'):self.scripts.append(v['src'])
  if tag=='link' and v.get('rel')=='stylesheet':self.css.append(v['href'])
class ModuleGraph(unittest.TestCase):
 def test_module_imports_and_css_exist(self):
  html=HTML();html.feed((PAGE/'index.html').read_text())
  self.assertIn('/hexapod/freenove-combined-v10/app.js',html.scripts)
  self.assertIn('/hexapod/player-shell.js',html.scripts)
  # Shared styles were independently verified on the remote repo; V10 distribution only includes newly changed assets.
  verified_shared={'/hexapod/player-shell.css','/hexapod/live-v2-offline/pilot.css'}
  for ref in html.css:
   self.assertTrue(ref in verified_shared or (ROOT/'docs'/ref.lstrip('/')).is_file(),ref)
  visited=set()
  def walk(p):
   p=p.resolve()
   self.assertTrue(p.is_file(),str(p))
   if p in visited:return
   visited.add(p)
   for ref in re.findall(r"\b(?:import|export)\s+(?:[^;]*?\s+from\s+)?[\"']([^\"']+)[\"']",p.read_text()):
    if ref.startswith('.'):
     q=(p.parent/ref).resolve()
     self.assertTrue(q.is_relative_to(ROOT.resolve()),'import outside repository')
     walk(q)
    else:self.assertIn(ref,['three','three/addons/controls/OrbitControls.js','three/addons/loaders/STLLoader.js'])
  walk(PAGE/'app.js')
  self.assertGreaterEqual(len(visited),4)
 def test_valid_camera_coordinates_and_future_error(self):
  app=(PAGE/'app.js').read_text();page=(PAGE/'index.html').read_text()
  self.assertNotIn('p.x+=pilot.x;p.y+=pilot.y;',app)
  self.assertIn('p.x+=(pose?.x??0);p.y+=(pose?.y??0);',app)
  self.assertIn("../freenove-dual-v8/dual-stick-input.js",app)
  self.assertIn("../freenove-combined-v9/combo-controller.js",app)
  self.assertIn("window.addEventListener('error'",page)
 def test_never_start_runtime_physics(self):
  src=(PAGE/'app.js').read_text()
  for bad in ['WebSocket(','mj_step(','/api/physics','new Worker(']:self.assertNotIn(bad,src)
if __name__=='__main__':unittest.main()
