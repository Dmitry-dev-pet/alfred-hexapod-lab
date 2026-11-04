import pathlib
import sys
import unittest
import numpy as np

HERE=pathlib.Path(__file__).resolve().parents[1]
ROOT=next(p for p in HERE.parents if (p/'hexlab').is_dir())
for p in (HERE,ROOT,ROOT/'experiments'):
    if str(p) not in sys.path:sys.path.insert(0,str(p))
from cad30_observation import read_observation, command_bounded_torques, CAD_LEG_CODES
from failure_one_leg_cad30 import initialize


class Cad30AdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ctx=initialize()
        # initialize() reaches a 6-contact standing pose with gravity; do not
        # pretend to validate physical hardware sensors.

    def test_observation_shapes_and_stand_forces(self):
        ctx=self.ctx
        state=read_observation(ctx['model'],ctx['data'],legs=ctx['legs'])
        self.assertEqual(set(state['legs']),set(CAD_LEG_CODES))
        for code,x in state['legs'].items():
            self.assertEqual(len(x['joint_angles_rad']),3)
            self.assertEqual(len(x['joint_rates_rad_per_s']),3)
            self.assertEqual(len(x['toe_marker_world_m']),3)
            self.assertTrue(x['tibia_floor_contact'],code)
            self.assertGreaterEqual(x['tibia_normal_force_N'],0)
        total=sum(x['tibia_normal_force_N'] for x in state['legs'].values())
        self.assertAlmostEqual(total,17.706,delta=2.5)

    def test_bounded_torque_command(self):
        ctx=self.ctx
        output=command_bounded_torques(ctx['model'],ctx['data'],{c:[99,0,-99] for c in CAD_LEG_CODES},legs=ctx['legs'])
        self.assertGreater(output['saturated_motor_count'],0)
        self.assertLessEqual(max(abs(v) for v in output['control_nm']),.7000001)

    def test_reject_missing_leg(self):
        ctx=self.ctx
        with self.assertRaises(ValueError):
            command_bounded_torques(ctx['model'],ctx['data'],{'fl':[0,0,0]},legs=ctx['legs'])

    def test_reject_nan(self):
        ctx=self.ctx
        cmd={c:[0,0,0] for c in CAD_LEG_CODES}
        cmd['fl']=[0,float('nan'),0]
        with self.assertRaises(ValueError):
            command_bounded_torques(ctx['model'],ctx['data'],cmd,legs=ctx['legs'])

if __name__=='__main__': unittest.main()