"""Actual offline MuJoCo tests: no Gym/SB3/PyTorch dependency required."""
import math
import unittest
from pathlib import Path
import hashlib
import numpy as np
from physics import PupperCPU,git_blob_sha,load_cpu_model,ORIGINAL_XML,XML_BLOB,JOINTS,FOOT_SITES,DT,DECIMATION


class StanfordPhysicsContract(unittest.TestCase):
    def test_pinned_model_and_12_actuators(self):
        self.assertEqual(git_blob_sha(ORIGINAL_XML.read_bytes()),XML_BLOB)
        m=load_cpu_model()
        self.assertEqual((m.nq,m.nv,m.nu),(19,18,12))
        self.assertAlmostEqual(sum(m.body_mass),3.17,places=5)
        self.assertAlmostEqual(m.opt.timestep,.004,places=8)

    def test_seedless_physical_stance_and_observation(self):
        robot=PupperCPU()
        ob=robot.reset()
        self.assertEqual(ob.shape,(960,))
        self.assertEqual(ob.dtype,np.float32)
        np.testing.assert_array_equal(ob.reshape(20,48)[0,36:48],np.zeros(12))
        np.testing.assert_allclose(ob.reshape(20,48)[0,3:6],[0,0,-1],atol=1e-7)
        self.assertEqual(robot.control_steps,0)
        for _ in range(250):
            ob,m=robot.step(np.zeros(12,dtype=np.float32))
        self.assertAlmostEqual(robot.data.time,5,places=7)
        # Prior standalone Pupper stance preflight: z≈0.1565585m after 5s.
        self.assertAlmostEqual(m['height_m'],0.15655850,delta=0.0002)
        self.assertEqual(m['feet_loaded'],4)
        self.assertFalse(m['fell'])

    def test_deterministic_servo_response_and_inertia(self):
        def go():
            x=PupperCPU()
            x.reset()
            x.set_command((.15,.08,.15))
            result=[]
            for i in range(30):
                action=np.zeros(12,dtype=np.float32)
                action[0]=.12
                action[4]=-.13
                obs,m=x.step(action)
                result.append(x.data.qpos.copy())
            return np.asarray(result)
        np.testing.assert_array_equal(go(),go())

    def test_rejects_invalid_actions_and_commands(self):
        x=PupperCPU()
        with self.assertRaises(ValueError):x.set_command((5,0,0))
        with self.assertRaises(ValueError):x.step(np.full(12,np.nan))
        with self.assertRaises(ValueError):x.step([0,1])

    def test_real_contact_signal_not_synthetic(self):
        x=PupperCPU()
        x.reset()
        self.assertEqual(x.metrics()['feet_loaded'],0)
        for _ in range(50):_,m=x.step(np.zeros(12,dtype=np.float32))
        self.assertGreaterEqual(m['feet_loaded'],2)
        self.assertTrue(np.isfinite(m['actuator_max_nm']))
        self.assertTrue(np.isfinite(m['torque_norm']))


if __name__=='__main__':unittest.main()
