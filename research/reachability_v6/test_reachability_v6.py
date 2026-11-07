"""Research contract tests; do not treat IK/proxy success as a physical PASS."""
import json, math, unittest
from pathlib import Path
from reachability_sweep_v6 import DIRECTIONS, foot_curve, direction_offsets

ROOT=Path(__file__).resolve().parent
R=ROOT/'results'

class ReachabilityContract(unittest.TestCase):
    def test_toe_trajectory_contract(self):
        self.assertEqual(foot_curve(0,True),(-.5,0.))
        self.assertAlmostEqual(foot_curve(.5,True)[1],1.)
        self.assertAlmostEqual(foot_curve(1,True)[0],.5)
        self.assertAlmostEqual(foot_curve(.5,False)[0],0.)
        for u in [0,.1,.25,.5,.75,1]:
            x,h=foot_curve(u,True)
            self.assertTrue(-.5-1e-9<=x<=.5+1e-9)
            self.assertTrue(0<=h<=1)
    def test_offline_grid_contract(self):
        for path,count in [(R/'walking_ik_grid_v6.json',60),
                           (R/'lift40'/'walking_ik_grid_v6.json',60),
                           (R/'height40'/'walking_ik_grid_v6.json',28),
                           (R/'height25'/'walking_ik_grid_v6.json',56)]:
            with self.subTest(path=path):
                d=json.loads(path.read_text())
                self.assertEqual(d['schema'],'cad30-offline-walking-ik-grid-v6')
                self.assertFalse(d['physical_pass'])
                self.assertEqual(d['status'],'PRELIMINARY_GEOMETRY_ONLY')
                self.assertEqual(len(d['raw']),count)
                self.assertEqual(set(d['scan']['directions']),set(x['direction'] for x in d['raw']))
                for case in d['raw']:
                    self.assertEqual(case['all_leg_ik_pass'],all(z['ok'] for z in case['legs'].values()))
                    self.assertEqual(len(case['legs']),6)
                    for leg in case['legs'].values():
                        self.assertTrue(math.isfinite(leg['max_ik_error_mm']))
                        self.assertTrue(math.isfinite(leg['max_abs_joint_rad']))
    def test_principle_stance_matters(self):
        def row(path,h,l,d,step):
            x=json.loads(path.read_text())
            return next(z['all_leg_ik_pass'] for z in x['raw'] if z['body_height_offset_mm']==h and z['foot_lift_mm']==l and z['direction']==d and z['stride_mm']==step)
        self.assertTrue(row(R/'walking_ik_grid_v6.json',0,16,'fwd',45))
        self.assertFalse(row(R/'walking_ik_grid_v6.json',0,16,'fwd',55))
        self.assertFalse(row(R/'lift40'/'walking_ik_grid_v6.json',0,40,'fwd',15))
        self.assertTrue(row(R/'height25'/'walking_ik_grid_v6.json',25,40,'fwd',70))
        self.assertTrue(row(R/'height25'/'walking_ik_grid_v6.json',25,16,'fwd',90))
    def test_support_schedule_proxy(self):
        d=json.loads((R/'gait_support_proxy_v6.json').read_text())
        self.assertEqual(d['status'],'COM_PROJECTION_PROXY_ONLY')
        self.assertEqual(len(d['results']),210)
        expected={'wave':5,'ripple':4,'tripod':3}
        for r in d['results']:
            self.assertGreaterEqual(r['min_schedule_supports'],expected[r['gait']])
            self.assertTrue(math.isfinite(r['min_geometric_com_margin_mm']))
        self.assertLess(next(r['min_geometric_com_margin_mm'] for r in d['results'] if r['gait']=='ripple' and r['direction']=='fwd' and r['stride_mm']==35),0)

if __name__=='__main__':unittest.main()
