"""Independent analytical rolling sphere check of the optional floor law."""
import unittest,math
import numpy as np
import mujoco
from manual_trip import floor_torque

class PassiveFloorTest(unittest.TestCase):
    def test_rotated_sphere_coasts_with_analytical_deceleration_and_no_added_energy(self):
        angle=.37;q=[math.cos(angle/2),0,0,math.sin(angle/2)]
        xml=f'<mujoco><option timestep=".001" integrator="implicitfast"/><worldbody><geom name="floor" type="plane" size="5 5 .1" friction=".9 0 0" condim="3"/><body name="ball" pos="0 0 .1" quat="{q[0]} 0 0 {q[3]}"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia=".0066666666667 .0066666666667 .0066666666667"/><geom type="sphere" size=".1" friction=".9 0 0" condim="3"/></body></worldbody></mujoco>'
        m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);mujoco.mj_forward(m,d)
        d.qvel[0]=.1;d.qvel[3:6]=d.xmat[m.body('ball').id].reshape(3,3).T@np.array([0.,1.,0.])
        b=.00005;force=np.zeros(6);velocity=np.zeros(6);bid=m.body('ball').id;max_power=0.
        for _ in range(10000):
            d.xfrc_applied[:]=0;mujoco.mj_forward(m,d)
            normal=0.
            for j in range(d.ncon):mujoco.mj_contactForce(m,d,j,force);normal+=max(0.,float(force[0]))
            mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,bid,velocity,0)
            torque=floor_torque(velocity[:3],normal,b);max_power=max(max_power,float(torque@velocity[:3]))
            d.xfrc_applied[bid,3:]=torque;mujoco.mj_step(m,d)
        expected=.1-10*b*9.81/(.1*(1+2/3))
        self.assertAlmostEqual(d.qvel[0],expected,delta=.001)
        self.assertLessEqual(max_power,1e-15);self.assertFalse(any(d.warning.number))
        print('Coasting v at 10 s, simulated / analytical:',d.qvel[0],expected)

if __name__=='__main__':unittest.main()
