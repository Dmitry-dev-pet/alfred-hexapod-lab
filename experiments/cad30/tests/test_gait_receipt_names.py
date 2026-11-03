"""Regression test: changing a gait experiment parameter cannot overwrite canonical JSON."""
import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from locomotion_cad30 import receipt_filename
class GaitReceiptNames(unittest.TestCase):
 def test_nominal_historical_path_stable(self):
  self.assertEqual(receipt_filename('tripod','nominal'),'cad30_gait_tripod_nominal.json')
 def test_duration_changes_name(self):
  self.assertNotEqual(receipt_filename('tripod','nominal',seconds=2),receipt_filename('tripod','nominal'))
  self.assertIn('durations2',receipt_filename('tripod','nominal',seconds=2))
 def test_variations_have_unique_names(self):
  base=receipt_filename('tripod','nominal')
  for opts in ({'step':.020},{'lift':.012},{'period':3.2},{'friction':.6},{'seconds':2}):
   with self.subTest(opts=opts):self.assertNotEqual(receipt_filename('tripod','nominal',**opts),base)
 def test_unknown_cases_fail(self):
  with self.assertRaises(ValueError):receipt_filename('galumph','nominal')
if __name__=='__main__':unittest.main()
