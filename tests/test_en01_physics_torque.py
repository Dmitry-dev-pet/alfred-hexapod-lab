import unittest, json, math
from pathlib import Path
try:
    import numpy as np
    import mujoco
    from research.navbot_en01.physics_torque import ComputedTorque, inverse_segment
    from research.navbot_en01.com_torque import COMTorque
except ImportError as exc:
    raise unittest.SkipTest('Optional MuJoCo/NumPy research runtime required') from exc


class PhysicsTorqueTests(unittest.TestCase):
    def setup_case(self,kind=ComputedTorque):
        root=Path(__file__).resolve().parents[1]
        if not (root/'results/en01_tuck_v8/model.xml').exists() or not (root/'results/en01_tuck_v8/raw/back-tuck-neg-500_raw.json').exists():
            self.skipTest('Verified external v8 model and raw trace required')
        m=mujoco.MjModel.from_xml_string((root/'results/en01_tuck_v8/model.xml').read_text())
        d=mujoco.MjData(m)
        r=json.loads((root/'results/en01_tuck_v8/raw/back-tuck-neg-500_raw.json').read_text())
        f=min(r['frames'],key=lambda f:abs(f['t']-1.26))
        d.qpos[:]=f['qpos'];d.qvel[:]=f['qvel'];d.ctrl[:]=f['ctrl'];d.time=f['t'];mujoco.mj_forward(m,d)
        hq=[int(m.jnt_qposadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'hip_'+s)]) for s in ('left','right')]
        hv=[int(m.jnt_dofadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'hip_'+s)]) for s in ('left','right')]
        return m,d,kind(m,1,[0,2],hq,hv,[1,3],.8)

    def test_bounded_inverse_never_extrapolates(self):
        samples=[(-.03,40),(0,1),(.03,-37)]
        self.assertEqual(inverse_segment(100,samples),-.03)
        self.assertEqual(inverse_segment(-100,samples),.03)
        self.assertAlmostEqual(inverse_segment(20.5,samples),-.015)

    def test_virtual_dynamics_never_modify_live_state(self):
        _,d,c=self.setup_case()
        before={n:getattr(d,n).copy() for n in ('qpos','qvel','ctrl','qacc','qacc_warmstart','qfrc_applied','xfrc_applied')}
        t=d.time;torque,info=c.command(d,[.02,.02])
        self.assertEqual(t,d.time)
        for n,a in before.items():np.testing.assert_array_equal(a,getattr(d,n))
        self.assertTrue(all(abs(t)<=.03 for t in torque));self.assertEqual(c.queries,4)

    def test_response_sign_and_acceleration_scale(self):
        _,d,c=self.setup_case()
        a0=c.acceleration(d,0,[.02,.02]);a1=c.acceleration(d,-.015,[.02,.02]);a2=c.acceleration(d,-.03,[.02,.02])
        self.assertLess(a0,a1);self.assertLess(a1,a2);self.assertTrue(15<a1<25)

    def test_com_target_is_geometrically_derived(self):
        _,d,c=self.setup_case(COMTorque)
        _,info=c.command(d,[.02,.02])
        self.assertTrue(-.12<info['com_trim_rad']<-.05)
        self.assertAlmostEqual(info['target_pitch_rad'],info['com_trim_rad'])


if __name__=='__main__':unittest.main()
