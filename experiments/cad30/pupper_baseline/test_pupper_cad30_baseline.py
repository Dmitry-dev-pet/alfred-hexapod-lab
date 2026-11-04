"""Pupper reuse checks. Live CAD30 integration SKIPs when private model is absent."""
import os,sys,unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
HERE=Path(__file__).resolve().parent
VENDOR=HERE/'vendor'/'stanford_pupper'
if not VENDOR.is_dir():
 VENDOR=HERE.parent/'vendor'/'stanford_pupper'
sys.path[:0]=[str(HERE),str(VENDOR)]
from StanceController import StanceController
from SwingLegController import SwingController

class PupperOfficialControllerTests(unittest.TestCase):
 def test_stance_is_stationary_for_zero_velocity(self):
  config=SimpleNamespace(dt=.05,z_time_constant=1.)
  state=SimpleNamespace(foot_locations=np.array([[0.,.1],[-.2,.2],[0.,0.]]),height=0.)
  cmd=SimpleNamespace(horizontal_velocity=np.array([0.,0.]),yaw_rate=0.)
  self.assertTrue(np.allclose(StanceController(config).next_foot_location(0,state,cmd),state.foot_locations[:,0]))

 def test_stance_moves_foot_targets_opposite_body_command(self):
  config=SimpleNamespace(dt=.05,z_time_constant=1.)
  state=SimpleNamespace(foot_locations=np.array([[0.],[-.2],[0.]]),height=0.)
  cmd=SimpleNamespace(horizontal_velocity=np.array([0.,-.02]),yaw_rate=0.)
  target=StanceController(config).next_foot_location(0,state,cmd)
  self.assertAlmostEqual(target[1]-state.foot_locations[1,0],.001,places=8)

 def test_swing_generates_20mm_foot_advance_and_lift(self):
  initial=np.array([[-.14],[ -.2],[0.]])
  cfg=SimpleNamespace(dt=.05,stance_ticks=20,swing_ticks=40,z_clearance=.012,alpha=.5,beta=0.,default_stance=initial)
  cmd=SimpleNamespace(horizontal_velocity=np.array([0.,.04]),yaw_rate=0.,height=0.)
  state=SimpleNamespace(foot_locations=initial.copy())
  swing=SwingController(cfg);zmax=0
  for k in range(cfg.swing_ticks):
   state.foot_locations[:,0]=swing.next_foot_location(k/cfg.swing_ticks,0,state,cmd)
   zmax=max(zmax,state.foot_locations[2,0])
  self.assertAlmostEqual(state.foot_locations[1,0]-initial[1,0],.02,places=6)
  self.assertAlmostEqual(zmax,.012,places=7)

@unittest.skipUnless((Path(os.environ.get('CAD30_ROOT','nonexistent'))/'source/mjcf/scene_local30_nominal.xml').is_file(), 'Private CAD30 not locally restored')
class PhysicalCAD30Tests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  from pupper_four_first_swing import run_trial
  cls.report=run_trial(shift_forward=.02)

 def test_6ground_to_4ground_torque_and_angles(self):
  prep=self.report['four_support_preparation']
  self.assertEqual(prep['status'],'FOUR_CONTACT_STAND_FROM_SIX')
  self.assertEqual(prep['initial_Pupper_stance_status'],'PHYSICAL_PREBALANCE_PASS')
  self.assertGreaterEqual(prep['min_stance_loaded_at_each_2ms_step'],4)
  self.assertLess(prep['max_joint_abs_rad'],1.7)
  self.assertLess(prep['max_model_torque_nm'],.7)

 def test_front_manipulator_pair_airborne(self):
  for z in self.report['four_support_preparation']['front_toe_heights_mm'].values():
   self.assertGreater(z,60.)
  self.assertLess(self.report['end_contacts']['fl'],.08)
  self.assertLess(self.report['end_contacts']['fr'],.08)

 def test_pupper_one_leg_real_liftoff_and_advance(self):
  self.assertTrue(self.report['swing_liftoff_logged'])
  self.assertGreater(self.report['physical_tip_forward_mm'],15.)
  self.assertLess(self.report['physical_tip_forward_mm'],25.)
  self.assertLess(self.report['max_mink_target_error_mm'],.3)
  self.assertGreaterEqual(self.report['minimum_loaded_leg_count'],3)

 def test_end_four_contacts_actually_reloaded(self):
  self.assertTrue(self.report['actual_one_step_gate_pass'])
  self.assertTrue(self.report['four_legs_reloaded_at_end'])
  self.assertEqual(self.report['saturation_steps'],0)
  self.assertLess(self.report['max_torque_Nm'],.7)
  self.assertLess(self.report['swing_max_joint_abs_rad'],1.7)

 def test_next_middle_lift_is_vetoed_by_physical_geometry(self):
  margins=self.report['geometry_after_one_step']['static_three_toe_margins_mm']
  self.assertLess(margins['mr'],-50.)
  self.assertGreater(margins['br'],5.)
  self.assertFalse(self.report['full_cycle_walked'])

if __name__=='__main__':unittest.main()