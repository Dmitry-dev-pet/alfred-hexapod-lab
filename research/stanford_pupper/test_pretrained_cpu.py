"""No-training Stanford Pupper original-policy CPU reproduction checks."""
import hashlib
import json
import math
from pathlib import Path
import unittest
import numpy as np
from infer_pupper import load_policy, net, gait_ref, launch, POLICY_SHA256
from export_pretrained_viewer import export

ROOT=Path(__file__).resolve().parent
CHECK_SHA = '662a6c60f4a283cdf2ce5d18bcbfd49553d12615e853d8e040a20eeba899f573'
REPLAY_SHA='23270ffcf273fe8be9805424d0760e3654ba5379be67ddb0c5ea03f47c6386ca'

class PretrainedPupperTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.policy,cls.weights=load_policy(ROOT/'default_policy.json')
        cls.trace=json.loads((ROOT/'trial_5s.json').read_text())
        cls.viewer=json.loads((ROOT/'pupper-pretrained-5s-replay.json').read_text())
    def test_pinned_policy_and_shapes(self):
        self.assertEqual(hashlib.sha256((ROOT/'default_policy.json').read_bytes()).hexdigest(),POLICY_SHA256)
        self.assertEqual(self.policy['in_shape'],[None,960])
        self.assertEqual([w.shape for w,_,_ in self.weights], [(960,256),(256,128),(128,128),(128,128),(128,12)])
        self.assertEqual(self.policy['observation_history'],20)
        self.assertEqual(self.policy['single_observation_size'],48)
        self.assertEqual(len(self.policy['gait_reference']['trot_table']),self.policy['gait_reference']['n_samples'])
        x=np.zeros(960,dtype=np.float32);x[5::48]=-1.
        a=net(self.weights,x)
        b=net(self.weights,x)
        self.assertTrue(np.array_equal(a,b))
        self.assertTrue(np.all(np.isfinite(a)))
    def test_original_gait_reference(self):
        for t in (0,.02,.4,1.26,4.97):
            z=gait_ref(self.policy,t,.2,0,0)
            self.assertEqual(z.shape,(12,))
            self.assertTrue(np.all(np.isfinite(z)))
        stop=gait_ref(self.policy,1.,0,0,0)
        self.assertTrue(np.array_equal(stop,np.zeros(12,dtype=np.float32)))
    def test_physics_closed_loop_deterministic(self):
        summary,frames,events=launch(self.policy,self.weights,cmd_vx=.2,duration=5.,delay=4)
        self.assertEqual(summary,self.trace['summary'])
        self.assertEqual(frames,self.trace['frames'])
        self.assertEqual(events,self.trace['policy_events'])
        self.assertEqual(summary['physics_steps'],1250)
        self.assertEqual(summary['frames'],250)
        self.assertEqual(summary['fall_guard_steps'],0)
        self.assertLess(summary['peak_servo_torque_Nm'],3.)
        self.assertGreater(summary['forward_dx_m'],.40)
        self.assertLess(summary['forward_dx_m'],.55)
        self.assertEqual(hashlib.sha256((ROOT/'trial_5s.json').read_bytes()).hexdigest(),CHECK_SHA)
    def test_exact_exported_fk_replay(self):
        d=self.viewer
        self.assertEqual(d['schema'],'pupper-v3-mujoco-cpu-pretrained-walk-replay-v1')
        self.assertTrue(d['is_walk'])
        self.assertEqual(d['control']['policy_sha256'],POLICY_SHA256)
        self.assertEqual(d['control']['training_performed_by_us'],False)
        self.assertEqual(d['source']['trace_sha256'],CHECK_SHA)
        self.assertEqual(d['physics_steps'],1250)
        self.assertEqual(len(d['visual_meshes']),13)
        self.assertEqual(len(d['frames']),250)
        self.assertTrue(all(len(row)==109 and all(math.isfinite(v) for v in row) for row in d['frames']))
        self.assertAlmostEqual(d['frames'][-1][1]-d['frames'][0][1],d['summary']['forward_m'],places=7)
        self.assertEqual(hashlib.sha256((ROOT/'pupper-pretrained-5s-replay.json').read_bytes()).hexdigest(),REPLAY_SHA)

if __name__=='__main__':
    unittest.main()