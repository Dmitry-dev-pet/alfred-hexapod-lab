"""Deterministic read-only tests of native Lite3 ONNX metadata and action-to-servo map."""
import unittest
import sys
import os
from pathlib import Path
import numpy as np
import mujoco

ROOT = Path(os.environ.get('LITE3_RESEARCH_ROOT', str(Path(__file__).resolve().parent)))
sys.path.insert(0,str(ROOT))
from lite3_onnx_joint_audit import (get_model,make_policy,mapping,build_observation,
                                    compare_policies,commands,scenario,ONNX,PT,sha256)


class Lite3PpoCPUTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.m=get_model()
        cls.s,cls.joints,cls.meta=make_policy()

    def test_pinned_sources(self):
        self.assertEqual(sha256(ONNX),'773a1bbd0fee45ec38604d979d5fd6fb339ae15d13bfb5d6e2056a171d2a3422')
        self.assertEqual(sha256(PT),'cca3a020163309790f8dc8ecb7553ff0e98e75d02e78a1773ad3734ca535a67c')

    def test_output_to_actuator_mapping_uses_joint_order(self):
        q_idx,v_idx,a_idx,joints,actuators=mapping(self.m,self.joints)
        self.assertEqual(joints,self.joints)
        self.assertEqual(len(set(a_idx.tolist())),14)
        self.assertNotEqual(joints,actuators)
        self.assertEqual(sum(a!=b for a,b in zip(joints,actuators)),14)
        for k,name in enumerate(self.joints):
            self.assertEqual(self.m.joint(name).qposadr[0],q_idx[k])
            self.assertEqual(actuators[a_idx[k]],name)
        self.assertEqual(joints[-2:],['mainarm_joint','forearm_joint'])
        self.assertEqual(actuators[:2],['mainarm_joint','forearm_joint'])

    def test_one_hot_action_servo_target_goes_to_named_joint(self):
        m=self.m
        _,_,a_idx,_,actuators=mapping(m,self.joints)
        defaults=np.array([float(v) for v in self.meta['default_joint_pos'].split(',')])
        for i in range(14):
            ctrl=np.zeros(m.nu)
            ctrl[a_idx]=defaults
            ctrl[a_idx[i]]+=0.25
            self.assertAlmostEqual(ctrl[a_idx[i]],defaults[i]+.25)
            self.assertEqual(actuators[a_idx[i]],self.joints[i])

    def test_metadata_input_output_contract(self):
        self.assertEqual(self.s.get_inputs()[0].shape,[1,53])
        self.assertEqual(self.s.get_outputs()[0].shape,[1,14])
        self.assertEqual(self.meta['observation_names'],
                         'base_ang_vel,projected_gravity,command,phase,joint_pos,joint_vel,actions')
        self.assertEqual(self.meta['action_scale'],'0.25')

    def test_observation_size_at_rest(self):
        m=self.m;d=mujoco.MjData(m)
        q_idx,v_idx,a_idx,_,_=mapping(m,self.joints)
        defaults=np.array([float(v) for v in self.meta['default_joint_pos'].split(',')])
        d.qpos[2]=.1
        d.qpos[q_idx]=defaults
        mujoco.mj_forward(m,d)
        obs=build_observation(d,np.zeros(3),q_idx,v_idx,defaults,np.zeros(14))
        self.assertEqual(obs.shape,(53,))
        self.assertTrue(np.isfinite(obs).all())
        self.assertTrue(np.allclose(obs[9:11],0)) # standing phase mask
        self.assertTrue(np.allclose(obs[11:25],0)) # relative joint positions

    def test_onnx_vs_pt_published_files_are_not_equivalent(self):
        data=compare_policies(self.s,seed=20261106,samples=201)
        self.assertEqual(data['comparison_status'],'MISMATCH')
        self.assertGreater(data['max_abs_action_difference'],0.1)

    def test_midstride_commands_change_inside_phase(self):
        self.assertEqual(len(scenario('midstride')),6)
        boundaries=[s[0] for s in scenario('midstride')[1:]]
        self.assertTrue(all(abs((t/.4)-round(t/.4))>.05 for t in boundaries))
        self.assertEqual(commands(2.,'midstride').tolist(),[.25,0.,0.])
        self.assertAlmostEqual(float(commands(3.2,'midstride')[1]),.13,places=5)

    def test_real_mujoco_model_compiles(self):
        d=mujoco.MjData(self.m)
        mujoco.mj_step(self.m,d)
        self.assertTrue(np.isfinite(d.qpos).all())
        self.assertEqual(self.m.nu,14)


if __name__=='__main__':unittest.main()