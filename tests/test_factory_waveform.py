"""Vendor arithmetic invariant checks, no CAD physics required beyond SciPy import path."""
import unittest,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from build_factory_v7 import vendor_gait,VENDOR_POINTS
class SourceAlgorithm(unittest.TestCase):
 def test_vendor_strokes_cardinal(self):
  for gait,expected_horizontal in [('1',70.),('2',35*5/3)]:
   frames,F=vendor_gait(gait,0,35,0)
   self.assertEqual(F,48 if gait=='1' else 108)
   # For leg 0 manufacturer coordinate y traverses the full horizontal toe stroke.
   values=[p[0][1] for p in frames]
   measured=max(values)-min(values)
   self.assertAlmostEqual(measured,expected_horizontal,delta=2)
 def test_vendor_phase_groups(self):
  for gait,maximum in [('1',3),('2',1)]:
   frames,F=vendor_gait(gait,0,35,0)
   above=[sum(1 for p in f if p[2]>-24.999) for f in frames]
   self.assertLessEqual(max(above),maximum)
 def test_end_loop_no_horizontal_teleport(self):
  for gait in ('1','2'):
   frames,F=vendor_gait(gait,0,35,0)
   for j,p in enumerate(frames[-1]):
    self.assertAlmostEqual(p[0],VENDOR_POINTS[j][0],delta=.00001)
    self.assertAlmostEqual(p[1],VENDOR_POINTS[j][1],delta=.00001)
if __name__=='__main__':unittest.main()
