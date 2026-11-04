"""Physical safety gates for six-to-four claw preparation, MuJoCo model only."""
import unittest
from experiments.cad30.six_grounded_prebalance_v1 import simulate_case

class SixGroundedPrebalanceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.good=simulate_case(shift=.038, z=.010, record=True, mu=1.1)

    def test_real_six_grounded_start(self):
        first=self.good['qpos_10hz'][0]
        self.assertEqual(first['t'],0.)
        self.assertEqual(set(first['contacts']),{'ml','mr','bl','br','fl','fr'})

    def test_four_foots_stay_supported_throughout(self):
        self.assertEqual(self.good['min_stance_contacts_all_steps'],4)
        self.assertEqual(self.good['min_contacts_all_steps'],4)
        self.assertEqual(self.good['motor_saturation_fraction'],0.)

    def test_front_load_is_zero_after_smooth_unload(self):
        self.assertEqual(self.good['after_unload_front_load_N'],0.)
        self.assertEqual(self.good['after_unload_front_contacts'],0)
        self.assertEqual(self.good['after_unload_four_support_contacts'],4)
        self.assertGreater(self.good['after_unload_four_margin_mm'],20.)

    def test_actual_claw_clearance(self):
        self.assertGreater(min(self.good['high_claw_clearances_mm'].values()),95.)
        self.assertEqual(set(self.good['last_contacts']),{'ml','mr','bl','br'})

    def test_no_fabricated_walk(self):
        self.assertFalse(any('swing' in row['phase'] for row in self.good['timeseries_50hz']))
        self.assertEqual(len(self.good['qpos_10hz']),69)
        self.assertEqual(self.good['qpos_10hz'][-1]['t'],6.8)

if __name__=='__main__':unittest.main()
