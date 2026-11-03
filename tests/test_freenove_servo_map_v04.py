"""Tests grounded in vendor control.py for Freenove servo channel wiring."""
import math
import unittest
from hexlab.freenove_servo_map_v04 import (
    VENDOR_LEGS, SERVO_CHANNELS, CAD_TO_VENDOR_LEG, channel_map, cad_joint_names,
    vendor_angles_to_servo_degrees, vendor_leg_for_cad
)


class FreenoveServoReferenceTests(unittest.TestCase):
    def test_18_nonduplicated_channels(self):
        m = channel_map()
        self.assertEqual(set(m), set(VENDOR_LEGS))
        flat = [x for channels in SERVO_CHANNELS.values() for x in channels]
        self.assertEqual(len(flat), 18)
        self.assertEqual(len(set(flat)), 18)

    def test_original_vendor_channels(self):
        self.assertEqual(SERVO_CHANNELS, {
            "fr":(15,14,13), "mr":(12,11,10), "br":(9,8,31),
            "bl":(22,23,27), "ml":(19,20,21), "fl":(16,17,18)
        })

    def test_vendor_handedness_without_calibration(self):
        for leg in VENDOR_LEGS[:3]:
            self.assertEqual(vendor_angles_to_servo_degrees(leg,(90,15,30)), (90,75,30))
        for leg in VENDOR_LEGS[3:]:
            self.assertEqual(vendor_angles_to_servo_degrees(leg,(90,15,30)), (90,105,150))

    def test_all_six_cad_names_mirror_vendor_side(self):
        self.assertEqual(CAD_TO_VENDOR_LEG, {
            "fr":"fl", "mr":"ml", "br":"bl",
            "bl":"br", "ml":"mr", "fl":"fr",
        })
        self.assertEqual(cad_joint_names(vendor_leg_for_cad("fl")),
                         ("motor15","motor14","motor13"))
        self.assertEqual(cad_joint_names(vendor_leg_for_cad("fr")),
                         ("motor16","motor17","motor18"))
        with self.assertRaises(ValueError):
            vendor_leg_for_cad("xyz")

    def test_clamp_and_calibration_signs(self):
        self.assertEqual(vendor_angles_to_servo_degrees("fr",(90,15,30),(5,2,4)),
                         (95,73,34))
        self.assertEqual(vendor_angles_to_servo_degrees("fl",(90,15,30),(5,2,4)),
                         (95,107,146))
        self.assertEqual(vendor_angles_to_servo_degrees("fr",(220,-220,200)),
                         (180,180,180))

    def test_cad_revolute_joint_names_by_channel(self):
        self.assertEqual(cad_joint_names("fr"),("motor15","motor14","motor13"))
        self.assertEqual(cad_joint_names("bl"),("motor22","motor23","motor27"))
        self.assertEqual(cad_joint_names("mr"),("motor12","motor11","motor10"))

    def test_nan_and_wrong_joint_count_rejected(self):
        with self.assertRaises(ValueError):
            vendor_angles_to_servo_degrees("fr",(0,0,math.nan))
        with self.assertRaises(ValueError):
            vendor_angles_to_servo_degrees("fr",(0,0))
        with self.assertRaises(ValueError):
            vendor_angles_to_servo_degrees("unknown",(0,0,0))


if __name__=="__main__":
    unittest.main()
