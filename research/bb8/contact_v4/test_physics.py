"""Independent conservation check on the actual archived coupled model."""
import json,os,unittest
from pathlib import Path
import numpy as np
import mujoco

class ClosedSystemTest(unittest.TestCase):
    def test_internal_drive_cannot_translate_closed_system_center_of_mass(self):
        base=Path(os.environ.get('BB8_BASELINE',''))
        if not (base/'nominal-model.xml').is_file():
            self.skipTest('Set BB8_BASELINE to the archived v3 validation directory')
        m=mujoco.MjModel.from_xml_path(str(base/'nominal-model.xml'))
        self.assertEqual(m.nu,2)
        self.assertEqual({m.joint(int(j)).name for j in m.actuator_trnid[:,0]}, {'left_axle','right_axle'})
        self.assertEqual(int(m.jnt_type[m.joint('shell_free').id]),int(mujoco.mjtJoint.mjJNT_FREE))
        self.assertEqual(int(m.jnt_type[m.joint('chassis_free').id]),int(mujoco.mjtJoint.mjJNT_FREE))
        m.opt.gravity[:]=0
        floor=m.geom('floor').id;m.geom_contype[floor]=0;m.geom_conaffinity[floor]=0
        initial=json.loads((base/'hold-nominal.json').read_text())['frames'][0]['qpos']
        errors=[]
        for dt in [.001,.0005]:
            m.opt.timestep=dt;d=mujoco.MjData(m);d.qpos[:]=initial;mujoco.mj_forward(m,d)
            def com():return (m.body_mass[:,None]*d.xipos).sum(axis=0)/m.body_mass.sum()
            start=com().copy()
            for _ in range(round(.2/dt)):
                d.ctrl[:]=.04;mujoco.mj_step(m,d)
            mujoco.mj_forward(m,d)
            errors.append(float(np.linalg.norm(com()-start)))
            self.assertFalse(any(d.warning.number))
            self.assertGreater(float(np.linalg.norm(d.qvel[12:14])),.01)
        # Rigid-body orientation integration has a finite-step COM residual.
        # Bound it at 10 micrometres over 0.2 s and require first-order refinement.
        self.assertLess(errors[0],1e-5)
        self.assertLess(errors[1],.6*errors[0])
        print('closed-system COM drift, 1 ms / 0.5 ms:',errors)

if __name__=='__main__':unittest.main()
