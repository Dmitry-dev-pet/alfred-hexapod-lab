"""Regression tests for a purely geometric sufficient condition. No robot gait PASS."""
from __future__ import annotations
import json
import math
import unittest
from pathlib import Path
from planar_certificate import (certificate, search_orders, signed_margin, hull,
                                eroded_triangle, project_to_polygon, add)

HERE=Path(__file__).resolve().parent

class PlanarCertificateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture=json.loads((HERE/'recorded_4foot_fixture.json').read_text())
        cls.feet={k:tuple(map(float,v[:2])) for k,v in fixture['feet_world_xyz_m'].items()}
        cls.com=tuple(map(float,fixture['COM_xyz_m'][:2]))

    def test_rectangle_constructs_geometric_period(self):
        feet={'ml':(-.15,.15),'mr':(.15,.15),'bl':(-.15,-.15),'br':(.15,-.15)}
        cert=certificate(feet,(0.,0.),('ml','br','mr','bl'),.020,.010)
        self.assertEqual(cert['status'],'PLANAR_GEOMETRY_CERTIFIED_ONLY')
        self.assertGreaterEqual(cert['minimum_swing_tripod_margin_mm'],10.-1e-6)
        self.assertGreaterEqual(cert['min_four_support_segment_bound_mm'],10.-1e-6)
        self.assertEqual(cert['final_COM_mm'],[0.,20.])
        self.assertEqual(len(cert['phases']),4)

    def test_recorded_case_all_orders_are_only_planar(self):
        c=search_orders(self.feet,self.com,.020,.010)
        self.assertEqual(c['status'],'PLANAR_GEOMETRY_CERTIFIED_ONLY')
        self.assertEqual(c['n_orders_checked'],24)
        self.assertEqual(c['n_geometrically_accepted'],24)
        self.assertEqual(c['best']['status'],'PLANAR_GEOMETRY_CERTIFIED_ONLY')
        self.assertIn('NOT robot',c['best']['important'])

    def test_margin_certificate_every_phase_and_segment(self):
        c=certificate(self.feet,self.com,('ml','mr','bl','br'),.020,.010)
        self.assertGreaterEqual(c['min_four_support_segment_bound_mm'],10.-1e-5)
        self.assertGreaterEqual(c['minimum_swing_tripod_margin_mm'],10.-1e-5)
        self.assertEqual(len(c['phases']),4)
        self.assertEqual(len(c['segments']),5)

    def test_necessary_first_middle_swing_com_displacement(self):
        # A true necessary distance for this fixed toe footprint / given 10mm safety margin.
        required={}
        for leg in self.feet:
            tri=hull([v for k,v in self.feet.items() if k!=leg])
            safe=eroded_triangle(tri,.010)
            closest=project_to_polygon(self.com,safe)
            required[leg]=math.dist(self.com,closest)*1000
        self.assertAlmostEqual(required['ml'],92.0893,delta=.002)
        self.assertAlmostEqual(required['mr'],101.4631,delta=.002)
        self.assertAlmostEqual(required['bl'],0.,delta=.002)
        self.assertAlmostEqual(required['br'],0.,delta=.002)

    def test_translation_invariance(self):
        shift=(1.3,-.7)
        moved={leg:add(v,shift) for leg,v in self.feet.items()}
        r1=certificate(self.feet,self.com,('bl','br','ml','mr'),.020,.010)
        r2=certificate(moved,add(self.com,shift),('bl','br','ml','mr'),.020,.010)
        self.assertAlmostEqual(r1['total_planar_COM_travel_mm'],r2['total_planar_COM_travel_mm'],delta=1e-5)
        self.assertAlmostEqual(r1['max_single_COM_shift_mm'],r2['max_single_COM_shift_mm'],delta=1e-5)

    def test_invalid_or_large_margin_fails(self):
        self.assertEqual(certificate(self.feet,self.com,('ml','mr','bl','br'),.020,.030)['status'],'PLANAR_FAIL')
        with self.assertRaises(ValueError):
            certificate(self.feet,self.com,('ml','mr','bl','br'),0.,.010)
        with self.assertRaises(ValueError):
            certificate(self.feet,self.com,('ml','mr','bl','bl'),.020,.010)

    def test_no_feet_teleported_before_swing(self):
        cert=certificate(self.feet,self.com,('bl','br','ml','mr'),.02,.010)
        for phase in cert['phases']:
            leg=phase['swing_leg']
            self.assertAlmostEqual(phase['new_foot_world_mm'][0],self.feet[leg][0]*1000,delta=.0002)
            self.assertAlmostEqual(phase['new_foot_world_mm'][1],self.feet[leg][1]*1000+20,delta=.0002)

    def test_planar_output_never_claims_dynamic_or_ik_success(self):
        c=search_orders(self.feet,self.com,.02,.010)
        self.assertNotIn('dynamic',c['best'])
        self.assertNotIn('full_gait_pass',c['best'])
        self.assertIn('NOT robot IK',c['best']['important'])

if __name__=='__main__':unittest.main()