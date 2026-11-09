import copy
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
from research.navbot_en01.linkage import solve, audit
from research.navbot_en01.linkage_model import create,load_evidence,set_configuration

HAS_PHYSICS = all(importlib.util.find_spec(name) is not None for name in ('mujoco','numpy'))


class FourBarTests(unittest.TestCase):
    def setUp(self):
        self.e=load_evidence();self.p=self.e['sides']['cad_z_positive']['points_xy_mm']

    def test_original_axis_correspondences_and_mirror(self):
        self.assertLess(self.e['mirror_xy_error_mm'],1e-8)
        for s in self.e['sides'].values():
            self.assertEqual(len(s['axis_evidence']),13)
            self.assertAlmostEqual(s['lengths_mm']['BC'],12,places=8)
            self.assertAlmostEqual(s['lengths_mm']['AD'],24,places=8)
            self.assertAlmostEqual(s['lengths_mm']['BW'],50,places=8)

    def test_rest_pose_and_all_link_lengths_over_101_poses(self):
        for i in range(101):
            q=-.22+.87*i/100
            s=solve(self.p,-q)
            self.assertLess(s['closure_residual_mm'],1e-10)
            if i==0: self.assertGreater(s['W'][1],self.p['W'][1])
        s=solve(self.p,0)
        for k in 'ABCDW':self.assertLess(math.dist(s[k],self.p[k]),1e-10)

    def test_rejects_singular_unreachable_nonfinite_and_wrong_source(self):
        for value in [.6,float('nan'),float('inf')]:
            with self.assertRaises(ValueError):solve(self.p,value)
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp)/'fake.stp';p.write_text('not the original STEP')
            with self.assertRaises(ValueError):audit(p)
        e=copy.deepcopy(self.e);e['source']['git_blob_sha1']='0'*40
        with self.assertRaises(ValueError):create(e)

    @unittest.skipUnless(HAS_PHYSICS, 'MuJoCo/numpy required; use configured research runtime')
    def test_mujoco_independent_forward_kinematics(self):
        import mujoco
        import numpy as np
        m=mujoco.MjModel.from_xml_string(create(self.e,fixed_base=True,gravity=False));d=mujoco.MjData(m)
        self.assertEqual(m.nu,4);self.assertEqual(m.neq,2)
        self.assertAlmostEqual(float(sum(m.body_mass)),.322,places=12)
        A=self.p['A']
        for q in np.linspace(-.22,.65,101):
            s=set_configuration(m,d,self.p,float(q))
            for side in ['left','right']:
                b=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,side+'_wheel')
                self.assertAlmostEqual(d.xpos[b,0],(s['W'][0]-A[0])/1000,places=11)
                self.assertAlmostEqual(d.xpos[b,2],m.body_pos[1,2]+(s['W'][1]-A[1])/1000-.015,places=11)
                a=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,side+'_closure_calf')
                c=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,side+'_closure_rod')
                self.assertLess(float(np.linalg.norm(d.site_xpos[a]-d.site_xpos[c])),1e-11)

    @unittest.skipUnless(HAS_PHYSICS, 'MuJoCo/numpy required; use configured research runtime')
    def test_export_rejects_forged_poses_and_model(self):
        # A short genuine trace is sufficient for failure-path integrity checks.
        from research.navbot_en01.linkage_probe import run
        from research.navbot_en01.linkage_export import build
        with tempfile.TemporaryDirectory() as temp:
            src=Path(temp)/'trace.json';run('stand',src,duration=.04)
            doc=json.loads(src.read_text());doc['summary']['duration_s']=6
            doc['frames'][0]['body_poses'][0][0]+=.01;src.write_text(json.dumps(doc))
            with self.assertRaisesRegex(ValueError,'disagree'):build(src,Path(temp)/'out.json')
            doc['summary']['model_sha256']='0'*64;src.write_text(json.dumps(doc))
            with self.assertRaisesRegex(ValueError,'hash mismatch'):build(src,Path(temp)/'out.json')


if __name__=='__main__':unittest.main()
