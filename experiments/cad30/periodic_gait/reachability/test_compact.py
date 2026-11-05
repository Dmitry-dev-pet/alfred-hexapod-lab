"""Auditable compact checks. Passing does not certify reachability or walking."""
from pathlib import Path
import json,unittest
D=Path(__file__).resolve().parent
class CompactReceiptTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.summary=json.loads((D/'summary.json').read_text())
        cls.bounds=json.loads((D/'geometric_fourth_tip_bounds.json').read_text())
        cls.overlap=json.loads((D/'tilt_four_overlap.json').read_text())
    def test_scopes_and_disclaimer(self):
        self.assertEqual(self.summary['status'],'BOUNDED_SCREEN_ONLY_NOT_CERTIFIED_REGION')
        fixed=self.summary['results']['fixed_torso']
        self.assertEqual((fixed['test_per_case'],fixed['four_anchor_feasible'],fixed['three_anchor_ml_feasible'],fixed['three_anchor_mr_feasible']), (169,16,3,2))
        self.assertEqual((fixed['four_and_three_overlap_ml'],fixed['four_and_three_overlap_mr']),(0,0))
    def test_height_tilt_counts(self):
        r=self.summary['results']
        self.assertEqual((r['height']['tested_total'],r['height']['three_anchor_ml_feasible'],r['height']['three_anchor_mr_feasible']),(504,19,11))
        self.assertEqual((r['tilt']['tested_total'],r['tilt']['three_anchor_ml_feasible'],r['tilt']['three_anchor_mr_feasible']),(750,26,9))
    def test_all_rigid_link_bounds_for_selected_endpoints(self):
        a=self.bounds['records'];self.assertEqual(len(a),35)
        for x in a:
            self.assertTrue(x['rigid_link_bound_violated'])
            self.assertGreater(x['lifted_leg_hip_to_original_toe_mm'],x['link_length_upper_bound_mm'])
        for leg in ['ml','mr']:
            sub=[x for x in a if x['leg']==leg]
            self.assertEqual(len(sub),26 if leg=='ml' else 9)
    def test_no_four_foot_same_root_for_selected_endpoints(self):
        a=self.overlap['results'];self.assertEqual(len(a),35)
        self.assertFalse(any(x['four_pass'] for x in a))
        self.assertGreater(min(x['four_toe_error_mm'] for x in a if x['leg']=='ml'), 27)
        self.assertGreater(min(x['four_toe_error_mm'] for x in a if x['leg']=='mr'), 54)
if __name__=='__main__':unittest.main()
