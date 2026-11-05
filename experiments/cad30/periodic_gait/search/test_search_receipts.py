"""Regressions for bounded periodic CAD30 search; not physical gait evidence."""
from pathlib import Path
from collections import Counter
import json
import unittest

ROOT=Path(__file__).resolve().parent

class PeriodSearchReceiptTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.r=json.loads((ROOT/'GLOBAL_SEARCH_RECEIPT.json').read_text())
        cls.g=json.loads((ROOT/'search_2d_compact.json').read_text())
        cls.s=json.loads((ROOT/'search_stance_compact.json').read_text())
        cls.d=json.loads((ROOT/'periodic_3d_compact.json').read_text())

    def test_grid_contract(self):
        r=self.r['scope']
        total=len(r['middle_inward_mm'])*len(r['middle_forward_mm'])*len(r['rear_outward_mm'])*len(r['rear_forward_mm'])*r['foot_lift_orders']
        self.assertEqual(total,1728)
        self.assertEqual(self.g['tested_2d'],total)
        self.assertEqual(self.g['feasible_2d'],1728)

    def test_best_2d_cycles_consider_all_four_legs(self):
        for sample in self.g['top10']:
            self.assertEqual(set(sample['order']),{'ml','mr','bl','br'})
            self.assertEqual(len(sample['order']),4)

    def test_3d_failures_not_counted_as_gait(self):
        self.assertEqual(self.s['total'],72)
        self.assertEqual(self.s['start_pass'],48)
        self.assertEqual(self.d['count'],96)
        self.assertEqual(self.d['passing'],0)
        self.assertEqual(self.r['status'],'NOT_FOUND_WITHIN_BOUNDS_NOT_IMPOSSIBILITY')
        self.assertFalse(any(x['reason']=='SAMPLED_IK_STATIC_LP_ONLY' for x in self.d['records']))

    def test_numerical_first_failure_distribution(self):
        actual=Counter(x['stage'] for x in self.d['records'])
        self.assertEqual(dict(actual),self.r['first_3D_failure_stage_counts'])
        self.assertEqual(actual['prepare_br']+actual['prepare_bl']+actual['prepare_mr'],84)
        self.assertEqual(actual['swing_bl']+actual['swing_mr'],12)

    def test_projection_cannot_qualify_as_dynamic(self):
        self.assertIn('No complete G1 accepted candidate',self.r['uncertainties'][-1])

if __name__=='__main__':unittest.main()