#!/usr/bin/env python3
"""Focused integration checks on the real pinned model, no GPU required."""
import argparse
import contextlib
import io
from pathlib import Path
import unittest

import mujoco
import numpy as np

import bam_backlash_cpu as bench


class ModelContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.upstream = bench.upstream_inference(ROOT)
        cls.upstream.ort.InferenceSession = bench.cpu_session
        with contextlib.redirect_stdout(io.StringIO()):
            bam = cls.upstream.load_bam_model(200, 7.4, None)
            cls.model, cls.data, cls.ctrl, _ = cls.upstream.load_mujoco_with_bam(
                str(ROOT / 'src/mjlab_microduck/robot/microduck/scene_backlash.xml'),
                bam, .005, .1, 6.0)

    def setUp(self):
        m, d = self.model, self.data
        mujoco.mj_resetDataKeyframe(m, d, mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_KEY,'STAND'))
        self.ctrl.reset(d.qpos)
        self.ctrl.last_ts = d.time
        mujoco.mj_forward(m,d)

    def test_interleaved_servos_and_torque_model(self):
        m = self.model
        self.assertEqual((m.nq,m.nv,m.nu),(35,34,14))
        bq,bv = bench.backlash_indices(m)
        sq=m.jnt_qposadr[m.actuator_trnid[:,0]]
        self.assertFalse(set(bq)&set(sq))
        self.assertTrue(np.allclose(m.jnt_range[m.actuator_trnid[:,0]+1],[-np.pi/180,np.pi/180]))
        self.assertTrue(np.all(m.actuator_biastype==mujoco.mjtBias.mjBIAS_NONE))
        self.assertTrue(np.allclose(m.actuator_gainprm[:,0],1))
        self.assertTrue(np.all(m.dof_armature[self.ctrl.dof_indexes]>0))

    def test_encoder_observations_and_firmware_feedback(self):
        m,d,c=self.model,self.data,self.ctrl
        # Independent reference BAM control law without the adapter.
        actuator=c.model.actuator
        original=actuator.compute_control
        with contextlib.redirect_stdout(io.StringIO()):
            policy=self.upstream.PolicyInference(m,d,walking_onnx_path=str(POLICY),bam_ctrl=c,
                                                 use_projected_gravity=True,new_cmd_obs=True)
        bq,bv=bench.install_encoder_view(policy,c,m,d)
        sq,sv=np.array(policy.joint_qpos_indices),np.array(policy.joint_qvel_indices)
        d.qpos[bq]=np.linspace(-.01,.01,14);d.qvel[bv]=np.linspace(-.1,.1,14)
        before=d.qpos.copy()
        obs=policy.get_observations()
        self.assertEqual(obs.shape,(61,))
        np.testing.assert_allclose(obs[6:20],(d.qpos[sq]+d.qpos[bq]).astype(np.float32)-policy.default_pose)
        np.testing.assert_allclose(obs[20:34],d.qvel[sv]+d.qvel[bv],atol=1e-7)
        target=d.qpos[sq]+.05
        actual=actuator.compute_control(target,d.qpos[sq],d.qvel[sv],.005)
        expected=original(target,d.qpos[sq]+d.qpos[bq],d.qvel[sv],.005)
        np.testing.assert_array_equal(actual,expected)
        self.assertGreater(np.max(np.abs(actual-original(target,d.qpos[sq],d.qvel[sv],.005))),0)
        np.testing.assert_array_equal(before,d.qpos)
        actuator.compute_control=original

    def test_live_step_has_friction_and_voltage_bound(self):
        m,d,c=self.model,self.data,self.ctrl
        for _ in range(100):
            c.update();mujoco.mj_step(m,d)
        self.assertTrue(np.isfinite(d.qpos).all())
        self.assertTrue(np.all(m.dof_frictionloss[c.dof_indexes]>0))
        self.assertTrue(np.all(np.abs(d.actuator_force)<=m.actuator_forcerange[:,1]+1e-9))
        self.assertFalse(any(w.number for w in d.warning))

    def test_seeded_recipe_is_repeatable_and_distinct(self):
        a=bench.perturbation('roll180',20261109)
        self.assertEqual(a,bench.perturbation('roll180',20261109))
        self.assertNotEqual(a,bench.perturbation('roll180',20261110))
        self.assertNotEqual(a,bench.perturbation('pitch180',20261109))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--rl-root',type=Path,required=True);p.add_argument('--policy',type=Path,required=True)
    args,rest=p.parse_known_args();ROOT=args.rl_root;POLICY=args.policy
    unittest.main(argv=['check_bam_backlash.py']+rest,verbosity=2)
