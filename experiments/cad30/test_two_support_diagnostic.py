"""Focused CAD30 short contact diagnostics, no hardware, no default hosted CI physics.

When the externally stored CAD30 model and safe-state snapshot are unavailable,
physical integration tests are *SKIPPED*, not fabricated.
"""
from __future__ import annotations
import os
from pathlib import Path
import unittest
from two_support_diagnostic import motion, simulate

MODEL=Path(os.getenv('CAD30_MODEL_XML','/nonexistent_cad30.xml'))
SAFE=Path(os.getenv('CAD30_SAFE_STATE','/nonexistent_safe_state.json'))

class TimetableTest(unittest.TestCase):
    def test_stance_ramps(self):
        self.assertEqual(motion(.1)[1],'settle')
        self.assertAlmostEqual(motion(.3)[0],0)
        self.assertAlmostEqual(motion(1.1)[0],1)
        self.assertEqual(motion(1.6)[1],'reload')
        self.assertAlmostEqual(motion(2.2)[0],0)

@unittest.skipUnless(MODEL.is_file() and SAFE.is_file(),
                     'Private original CAD30 XML and real four-support qpos/qvel required')
class RealCAD30DiagnosticTest(unittest.TestCase):
    def test_original_four_support_hold(self):
        r=simulate(MODEL,SAFE,())
        self.assertEqual(r['contact_count_full_2ms'], {'4':1200})
        self.assertEqual(r['saturated_physics_steps'],0)
    def test_one_rear_actually_unloads(self):
        r=simulate(MODEL,SAFE,('bl',))
        self.assertEqual(r['hold_actual_contacts_match_requested_frames'],200)
        self.assertEqual(r['min_loaded_during_lift_hold'],3)
        self.assertLessEqual(r['max_abs_all_leg_joint_rad'],1.700001)
    def test_middle_command_must_not_be_marked_success(self):
        r=simulate(MODEL,SAFE,('ml',))
        self.assertEqual(r['hold_actual_contacts_match_requested_frames'],0)
        self.assertEqual(r['hold_commanded_leg_unloaded_physics_steps'],{})
        self.assertEqual(r['hold_unintended_leg_unloaded_physics_steps'].get('br'),200)
    def test_two_rear_commands_do_not_get_two_support(self):
        r=simulate(MODEL,SAFE,('bl','br'))
        self.assertEqual(r['contact_count_full_2ms'].get('2',0),0)
        self.assertEqual(r['hold_actual_contacts_match_requested_frames'],0)

if __name__=='__main__':unittest.main()