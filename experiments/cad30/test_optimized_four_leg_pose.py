from pathlib import Path
"""Regression checks for bounded optimization of four-support body pose.
Restore pinned external CAD assets (not included in repository) before running.
"""
import unittest
import numpy as np
import mujoco
from optimize_four_leg_pose_dynamics import run,ROOT,RAW
from optimize_four_leg_middle_liftoff import run as middle_lift
from front_pair_trick import SUPPORT,contact_points,margin
from hexlab.stand_geometry_v04 import load_legs
import json

class FourLegBodyPoseTests(unittest.TestCase):
    def test_unmodified_contact_polygon_geometry(self):
        m=mujoco.MjModel.from_xml_path(ROOT)
        d=mujoco.MjData(m);d.qpos[:]=json.loads(Path(RAW).read_text())['qpos']
        mujoco.mj_forward(m,d);pts=contact_points(m,d)
        self.assertTrue(all(pts[c] for c in SUPPORT))
        com=d.subtree_com[m.body('base_link_2').id][:2]
        self.assertLess(margin(com,[p for c in SUPPORT if c!='ml' for p in pts[c]]),-.07)
        self.assertGreater(margin(com,[p for c in SUPPORT if c!='bl' for p in pts[c]]),.01)

    def test_optimized_guard_is_contact_valid_but_not_swing_valid(self):
        r=run(0,-.09,.02,0,seconds=4)
        self.assertTrue(r['valid_four_contact_unsaturated'])
        self.assertLess(r['peak_tau_nm'],.55)
        self.assertLess(r['final_tilt_deg'],6)
        self.assertTrue(all(r['last_contacts'][c] for c in SUPPORT))
        self.assertTrue(all(not r['last_contacts'][c] for c in ('fl','fr')))
        self.assertGreater(min(r['front_min_clearance_last_m'].values()),.07)
        self.assertLess(r['final_leave_one_foot_margins_m']['ml'],0)
        self.assertGreater(r['final_leave_one_foot_margins_m']['ml'],-.04)

    def test_slippery_ground_breaks_guard_assumptions(self):
        r=run(0,-.09,.02,0,mu=.3,seconds=4)
        self.assertFalse(r['valid_four_contact_unsaturated'])
        self.assertGreater(r['saturation_step_fraction'],.1)

    def test_middle_lift_is_not_safe_due_to_torque_cap(self):
        r=middle_lift(.07,-.12,.045,8,mu=1.1)
        self.assertFalse(r['true_middle_lift_safe'])
        self.assertGreater(r['hold_last_middle_mesh_min_z'],.02)
        self.assertGreater(r['hold_last_com_margin_m'],.01)
        self.assertEqual(r['min_other_support_contact_count'],3)
        self.assertGreater(r['saturation_fraction'],.01)

if __name__=='__main__':
    unittest.main()
