"""Fast offline source-contract and determinism checks for full teleop CPU runner."""
from pathlib import Path
import unittest
import numpy as np
from full_controls import (
    wrap_to_pi, quaternion_yaw, HeadingHold, tilt_target, select_segment,
    validate_schedule, ref_with_slot, rollout, load_policy, ROOT,
)


class FullControlsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.p,cls.w=load_policy(ROOT/'default_policy.json')

    def test_clip_and_game_caps_contract(self):
        self.assertEqual(self.p['command_clip'],{'vx':[-1.5,1.5],'vy':[-.5,.5],'wz':[-2.,2.]})
        self.assertEqual(self.p['jump_slot']['walk_speed_cap'],.49)
        self.assertEqual(self.p['jump_slot']['run_speed_cap'],1.5)
        self.assertEqual(self.p['jump_slot']['hop_speed'],.5)

    def test_heading_hold_matches_source_edges(self):
        h=HeadingHold(self.p['heading_hold'])
        self.assertEqual(h.apply(.1,0,0,.4),.4)  # explicit yaw bypasses hold
        self.assertAlmostEqual(h.apply(.1,.2,0,0),0)  # capture quiet edge
        self.assertAlmostEqual(h.apply(.2,.2,0,0),-.15)
        self.assertAlmostEqual(h.apply(.2,.2,0,.25),.25)  # release
        self.assertAlmostEqual(h.apply(.4,.2,0,0),0)  # recapture after turn
        self.assertAlmostEqual(wrap_to_pi(-3*np.pi),-np.pi)
        self.assertAlmostEqual(wrap_to_pi(3*np.pi),np.pi)

    def test_pose_inverse_world_up(self):
        np.testing.assert_allclose(tilt_target(0,0),[0,0,1],atol=1e-6)
        np.testing.assert_allclose(tilt_target(.3,0),[0,np.sin(.3),np.cos(.3)],atol=1e-6)
        np.testing.assert_allclose(tilt_target(0,.2),[-np.sin(.2),0,np.cos(.2)],atol=1e-6)

    def test_schedule_validation_and_edges(self):
        v=[{'t':0},{'t':1,'vx':.2},{'t':2,'wz':-.25}]
        validate_schedule(v,4)
        self.assertEqual(select_segment(v,1.99)['vx'],.2)
        self.assertEqual(select_segment(v,2.00)['wz'],-.25)
        with self.assertRaises(ValueError):validate_schedule([{'t':0},{'t':.001}],4)
        with self.assertRaises(ValueError):validate_schedule([{'t':0},{'t':1,'vx':float('nan')}],4)

    def test_jump_reference_has_effect_inside_window_only(self):
        cmd=(0.,0.,0.)
        initial=ref_with_slot(self.p,2.50,cmd,None,False)
        end=ref_with_slot(self.p,2.50,cmd,1.5,False) # outside active
        np.testing.assert_array_equal(initial,end)
        active=ref_with_slot(self.p,1.75,cmd,1.5,False)
        self.assertTrue(np.all(np.isfinite(active)))
        self.assertGreater(float(np.max(np.abs(active))),.02)

    def test_end_to_end_determinism(self):
        seq=[{'t':0},{'t':1,'vx':.15,'vy':.08,'wz':.12},{'t':3}]
        a=rollout(self.p,self.w,seq,4)
        b=rollout(self.p,self.w,seq,4)
        self.assertEqual(a['summary'],b['summary'])
        self.assertEqual(a['frames'],b['frames'])
        self.assertEqual(a['policy_commands'],b['policy_commands'])
        self.assertEqual(a['summary']['executed_steps'],1000)
        self.assertEqual(a['summary']['frames'],200)
        self.assertEqual(a['summary']['status'],'NO_FALL_GUARD')


if __name__=='__main__':unittest.main()
