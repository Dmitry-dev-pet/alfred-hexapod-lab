"""Static/browser contract for Freenove Original dual-stick viewer."""
from html.parser import HTMLParser
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]
PAGE=ROOT/'docs/hexapod/freenove-dual-v8'
class Tags(HTMLParser):
    def __init__(self):
        super().__init__();self.ids=[];self.scripts=[];self.styles=[]
    def handle_starttag(self,tag,attrs):
        d=dict(attrs)
        if 'id' in d:self.ids.append(d['id'])
        if tag=='script' and 'src' in d:self.scripts.append(d['src'])
        if tag=='link' and d.get('rel')=='stylesheet':self.styles.append(d.get('href'))

class StaticDualStick(unittest.TestCase):
    def test_both_sticks_and_shell(self):
        h=(PAGE/'index.html').read_text()
        x=Tags();x.feed(h)
        self.assertEqual(len(x.ids),len(set(x.ids)),'Duplicate HTML ID')
        for expected in ('pilot-pad','pilot-thumb','pilot-pad-right','pilot-thumb-right',
                          'factory-mode','factory-gait','factory-tempo','factory-stop',
                          'pose-height','pose-yaw','viewer','play','seek','speed'):
            self.assertIn(expected,x.ids)
        self.assertIn('/hexapod/freenove-dual-v8/dual-stick.css',x.styles)
        self.assertIn('/hexapod/freenove-dual-v8/app.js',x.scripts)
        self.assertIn('/hexapod/player-shell.js',x.scripts)
        self.assertNotIn('pilot-ccw',x.ids)
        self.assertNotIn('pilot-cw',x.ids)
    def test_controller_stays_factory_original(self):
        app=(PAGE/'app.js').read_text()
        inp=(PAGE/'dual-stick-input.js').read_text()
        self.assertIn("'../freenove-original-v7/factory-controller.js'",app)
        self.assertIn("fetch('../data/freenove-original-v7-kit.json'",app)
        self.assertIn('new DualStickInput()',app)
        self.assertIn("['left','right']",app)
        self.assertIn('setPointerCapture(e.pointerId)',app)
        self.assertIn('lostpointercapture',app)
        self.assertIn("pilot.setBody('roll',i.roll)",app)
        self.assertIn("pilot.setBody('pitch',i.pitch)",app)
        self.assertIn('pointer===null',inp)
        self.assertIn("turn:right?-right.x",inp)
        for banned in ('WebSocket(', 'mj_step(', '/api/physics','new Worker(', 'GPUComputeRenderer'):
            self.assertNotIn(banned,app+inp)
    def test_files_finite(self):
        for name in ('index.html','app.js','dual-stick.css','dual-stick-input.js'):
            self.assertGreater((PAGE/name).stat().st_size,100)
            self.assertLess((PAGE/name).stat().st_size,50000)

if __name__=='__main__':unittest.main()
