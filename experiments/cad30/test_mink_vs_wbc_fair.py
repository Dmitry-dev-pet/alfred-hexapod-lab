"""Reproducible direct-MuJoCo kinematics tests; no robot hardware and no STL in repo."""
from __future__ import annotations
import os
from pathlib import Path
import unittest
import numpy as np
import mujoco
from benchmark_mink_vs_wbc import CASES,benchmark,setup_problem,solve_custom,solve_mink,contact_sites

MODEL=Path(os.getenv('CAD30_MODEL_XML','source/mjcf/scene_local30_nominal.xml'))
REPLAY=Path(os.getenv('CAD30_REPLAY_ZIP','FNK0052_claw_tuck_ABC_2026-11-04_v2_no_CAD.zip'))

@unittest.skipUnless(MODEL.is_file() and REPLAY.is_file(), 'must restore private CAD30 XML and recorded B pose')
class TestMatchedTasks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model,cls.qpos,cls.markers,cls.recon=contact_sites(MODEL,REPLAY)
        cls.common=setup_problem(cls.model,cls.qpos,None)
    def test_sites_match_true_recorded_contacts(self):
        self.assertEqual(self.model.nmesh,30)
        self.assertLess(self.recon,1e-8)
    def test_all_10_cases_have_same_exact_base(self):
        for name,targ in CASES.items():
            with self.subTest(case=name):
                target=np.asarray(targ,dtype=float)
                mine=solve_custom(self.model,self.qpos,target,self.common)
                mink=solve_mink(self.model,self.qpos,target,self.common)
                self.assertTrue(mine['ok'],mine.get('reason'))
                self.assertTrue(mink['ok'],mink.get('reason'))
                self.assertLessEqual(mink['max_frozen_base_velocity'],1e-10)
                self.assertLess(mine['max_foot_drift_mm'],.02)
                self.assertLess(mink['max_foot_drift_mm'],.02)
                self.assertLessEqual(mine['max_joint_delta_rad'],.0200000001)
                self.assertLessEqual(mink['max_joint_delta_rad'],.0200000001)
                self.assertTrue(np.allclose(mine['q'][:3],mink['q'][:3],atol=1e-11))
                self.assertTrue(np.allclose(mine['q'][3:7],mink['q'][3:7],atol=1e-11))
    def test_front_and_head_joint_poses_are_identical(self):
        vec=np.asarray(CASES['forward_down'])
        mine=solve_custom(self.model,self.qpos,vec,self.common)
        upstream=solve_mink(self.model,self.qpos,vec,self.common)
        for candidate in (mine,upstream):
            self.assertTrue(np.allclose(candidate['q'][self.common['qa'][12:]],self.qpos[self.common['qa'][12:]],atol=1e-12))
            other=set(range(7,len(self.qpos)))-set(self.common['qa'].tolist())
            for i in other:self.assertAlmostEqual(candidate['q'][i],self.qpos[i],delta=1e-12)
    def test_invalid_large_base_request_is_vetoed_by_custom(self):
        bad=np.array([0.,.003,0.,0.,0.,0.])
        self.assertFalse(solve_custom(self.model,self.qpos,bad,self.common)['ok'])
    def test_summary_is_explicitly_kinematic_only(self):
        r=benchmark(MODEL,REPLAY,n_repeat=2)
        self.assertEqual(r['status'],'KINEMATIC_MATCHED_TASK_PASS')
        self.assertEqual(r['summary']['cases_total'],10)
        self.assertEqual(r['summary']['both_pass'],10)
        self.assertIn('joint',r['model_joint_limits'])
        self.assertTrue(any('no physical simulation' in w.lower() for w in r['limitations']))

if __name__ == '__main__': unittest.main()