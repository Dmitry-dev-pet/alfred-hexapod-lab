"""Run public Pupper phase scheduler unchanged, then real CAD30 support probe."""
import os
import unittest
from pathlib import Path
import numpy as np
from audit_pupper_crawl import SlowCrawlConfig, GaitController,LEGS,audit


class TestReferenceScheduler(unittest.TestCase):
    def setUp(self):
        self.config=SlowCrawlConfig()
        self.controller=GaitController(self.config)

    def test_upstream_scheduler_eight_configured_phases(self):
        self.assertEqual(self.config.contact_phases.shape,(4,8))
        self.assertEqual(self.config.phase_length,400)

    def test_at_most_one_swing_on_each_tick(self):
        for t in range(self.config.phase_length*2):
            flags=np.asarray(self.controller.contacts(t),int)
            self.assertGreaterEqual(int(flags.sum()),3)
            self.assertLessEqual(int(flags.sum()),4)

    def test_each_leg_swings_exactly_once_per_period(self):
        phases=[self.controller.contacts(int(self.config.phase_ticks[:i].sum())) for i in range(8)]
        lifted=[LEGS[int(np.where(np.asarray(v)==0)[0][0])] for v in phases if np.sum(v)==3]
        self.assertEqual(tuple(lifted),self.config.swing_order)

    def test_invalid_gait_order_fails(self):
        with self.assertRaises(ValueError):SlowCrawlConfig(swing_order=('bl','bl','mr','ml'))


MODEL=Path(os.environ.get('CAD30_MODEL_XML','/mnt/data/hexcheck/source/mjcf/scene_local30_nominal.xml'))
POSE=Path(os.environ.get('CAD30_POSE_JSON','/mnt/data/hexapod_next/cad30_safe_pose_B_nominal.json'))
@unittest.skipUnless(MODEL.is_file() and POSE.is_file(),'Private CAD30 and saved pose missing')
class TestRealCAD30Reference(unittest.TestCase):
    def test_four_leg_contact_geometry_and_safety_veto(self):
        result=audit(MODEL,POSE)
        self.assertEqual(result['minimum_supporting_legs_by_schedule'],3)
        m=result['static_leave_one_leg_out_margin_mm']
        self.assertGreater(m['bl'],5)
        self.assertGreater(m['br'],5)
        self.assertLess(m['ml'],-80)
        self.assertLess(m['mr'],-80)
        self.assertFalse(result['hardware_motion_approved'])
        self.assertFalse(result['dynamic_walk_achieved'])

if __name__=='__main__':unittest.main()