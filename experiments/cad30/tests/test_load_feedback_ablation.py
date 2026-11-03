"""Unit and physical smoke checks for local load-gated leg coordination."""
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import mujoco
from load_feedback_ablation import (
    CODES, MAX_SELF_LOAD_MULTIPLIER, MIN_NEIGHBOR_LOAD_MULTIPLIER,
    LegState, local_force_liftoffs,normal_contact_forces,
)
from hexlab.stand_geometry_v04 import load_legs
from hexlab.stand_v04 import static_pose,hold_under_gravity
from hexlab.step_one_leg_v04 import floor_contacts


class LoadGateTests(unittest.TestCase):
    def setUp(self):
        self.contacts=set(CODES)
        self.states={c:LegState(c,stance_progress=0.2) for c in CODES}
        self.states['fl'].stance_progress=1.0
        self.priority={c:i for i,c in enumerate(CODES)}
        self.forces={c:3. for c in CODES}

    def _decide(self,mode):
        return local_force_liftoffs(self.states,self.contacts,mode,self.priority,1.0,self.forces,3.)

    def test_no_force_gate_baseline(self):
        self.forces['fl']=12.
        self.assertEqual(self._decide('neighbor_inhibit'),['fl'])

    def test_reject_heavily_loaded_leg(self):
        self.forces['fl']=MAX_SELF_LOAD_MULTIPLIER*3+0.25
        self.assertEqual(self._decide('neighbor_force_gate'),[])
        self.assertEqual(self._decide('neighbor_force_gate_supported'),[])

    def test_accept_unloaded_leg(self):
        self.forces['fl']=2.
        self.assertEqual(self._decide('neighbor_force_gate'),['fl'])

    def test_reject_if_either_neighbor_is_not_supported(self):
        self.forces['fl']=2.
        self.forces['fr']=MIN_NEIGHBOR_LOAD_MULTIPLIER*3-0.2
        self.assertEqual(self._decide('neighbor_force_gate'),['fl'])
        self.assertEqual(self._decide('neighbor_force_gate_supported'),[])

    def test_no_contact_means_no_lift_even_with_zero_force(self):
        self.contacts.remove('fl')
        self.forces['fl']=0.
        self.assertEqual(self._decide('neighbor_force_gate'),[])

    def test_invalid_mode(self):
        with self.assertRaises(ValueError):
            self._decide('global_tripod')

    def test_neighbor_inhibition_preserved(self):
        self.states['fr'].phase='swing'
        self.assertEqual(self._decide('neighbor_force_gate'),[])


class PhysicalLoadTests(unittest.TestCase):
    def test_standing_contact_normal_sum_matches_weight(self):
        root=Path(__file__).resolve().parents[1]
        model=mujoco.MjModel.from_xml_path(str(root/'source/mjcf/scene_local30_nominal.xml'))
        data=mujoco.MjData(model)
        legs=load_legs(model,data,mujoco)
        static_pose(model,data,mujoco,legs,.145,.15)
        hold=hold_under_gravity(model,data,mujoco,data.qpos.copy(),1.)
        self.assertGreaterEqual(hold['last_0p4s_min_contact_legs'],6)
        forces=normal_contact_forces(model,data)
        self.assertEqual(len(forces),6)
        self.assertEqual(set(forces),floor_contacts(model,data,mujoco))
        self.assertTrue(all(x>0 for x in forces.values()))
        self.assertAlmostEqual(sum(forces.values()),float(sum(model.body_mass))*9.81,delta=.03)
        self.assertAlmostEqual(sum(forces.values()),17.7057,delta=.05)


if __name__=='__main__':unittest.main()
