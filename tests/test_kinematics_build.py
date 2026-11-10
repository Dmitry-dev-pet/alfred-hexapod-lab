"""Offline V2 kit contracts. Optional FK oracle requires an authorized CAD model.

Set CAD30_SCENE=/path/to/scene_local30_nominal.xml to check the entire
exported kinematic tree against independent MuJoCo forward kinematics.
No MuJoCo import or private CAD is needed for the ordinary metadata test.
"""
from pathlib import Path
import json, os, unittest

DATA=Path(__file__).resolve().parents[1]/'docs/hexapod/data/live-v2-offline-kit.json'

class KinematicContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.kit=json.loads(DATA.read_text())
    def test_metadata(self):
        k=self.kit
        self.assertEqual(k['format'],'fnk0052-cad30-kinematic-control-v1')
        self.assertEqual(k['study'],'issue62-live-v2-offline')
        self.assertEqual(len(k['skeleton']),46)
        self.assertEqual(len(k['q_stand']),20)
        self.assertEqual(set(k['joint_tables']),{'fwd','back','left','right','ccw','cw'})
        for table in k['joint_tables'].values():
            self.assertEqual(len(table),48)
            self.assertTrue(all(len(row)==20 for row in table))
            self.assertTrue(all(max(abs(v) for v in row[:18])<1.7 for row in table))
    @unittest.skipUnless(os.environ.get('CAD30_SCENE'), 'requires pinned owner CAD30_SCENE + locally installed MuJoCo/scipy')
    def test_fk_matches_independent_mujoco_forward(self):
        import numpy as np
        import mujoco as mj
        from scipy.spatial.transform import Rotation as R
        def qrot(wxyz):return R.from_quat([*wxyz[1:],wxyz[0]])
        def reconstruct(kit,joints):
            out={0:(np.zeros(3),R.identity()),1:(np.array(kit['root_position']),qrot(kit['root_quat_wxyz']))}
            for b in kit['skeleton']:
                parent_p,parent_r=out[b['parent']];p=np.array(b['p']);rot=qrot(b['q'])
                if 'adr' in b:
                    angle=joints[b['adr']]-kit['q_stand'][b['adr']]
                    dq=R.from_rotvec(np.array(b['axis'])*angle)
                    anchor=np.array(b['anchor']);p=anchor+dq.apply(p-anchor);rot=dq*rot
                out[b['id']]=(parent_p+parent_r.apply(p),parent_r*rot)
            return out
        k=self.kit;m=mj.MjModel.from_xml_path(os.environ['CAD30_SCENE']);d=mj.MjData(m)
        for mode in ('fwd','left','ccw','back','right','cw'):
            for frame in (0,12,24,39):
                joints=k['joint_tables'][mode][frame]
                d.qpos[:3]=k['root_position'];d.qpos[3:7]=k['root_quat_wxyz'];d.qpos[7:]=joints
                mj.mj_forward(m,d)
                expected=reconstruct(k,joints)
                for bid in range(1,m.nbody):
                    p,r=expected[bid]
                    err=np.linalg.norm(p-d.xpos[bid]);rot_err=(r.inv()*qrot(d.xquat[bid])).magnitude()
                    self.assertLess(err,2e-6,f'body={bid} pos {err} mode={mode}')
                    self.assertLess(rot_err,2e-5,f'body={bid} quat {rot_err} mode={mode}')
if __name__=='__main__':unittest.main()
