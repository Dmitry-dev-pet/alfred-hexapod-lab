import hashlib,json,math,unittest
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
import mujoco
from model import make_model
from audit_mass import require_complete_mass_model

BASE=Path(__file__).parent

class ContactTests(unittest.TestCase):
    def test_mass_assumptions_need_explicit_opt_in(self):
        with self.assertRaisesRegex(ValueError,'masses are unverified'):make_model(level=1)
        manifest=json.loads((BASE/'mass_audit.json').read_text())
        with self.assertRaisesRegex(ValueError,'c54'):require_complete_mass_model(manifest)

    def test_independent_bodies_and_only_wheel_actuation(self):
        xml,_=make_model(level=1,allow_assumed_mass=True)
        m=mujoco.MjModel.from_xml_string(xml)
        self.assertEqual((m.nq,m.nv,m.nu),(16,14,2))
        self.assertEqual(m.body('chassis').parentid[0],0)
        self.assertEqual(m.body('shell').parentid[0],0)
        joints={m.joint(int(x)).name for x in m.actuator_trnid[:,0]}
        self.assertEqual(joints,{'left_axle','right_axle'})
        d=mujoco.MjData(m)
        for _ in range(20):mujoco.mj_step(m,d)
        self.assertAlmostEqual(d.time,.02)

    def test_shell_has_real_cavity_instead_of_solid_ball(self):
        xml,_=make_model(level=2,allow_assumed_mass=True)
        for radius,expect in [(.005,False),(.099,True)]:
            root=E.fromstring(xml);world=root.find('worldbody')
            b=E.SubElement(world,'body',name='probe',pos='0 0 .1001')
            E.SubElement(b,'freejoint');E.SubElement(b,'inertial',mass='.001',pos='0 0 0',diaginertia='.000001 .000001 .000001')
            E.SubElement(b,'geom',name='probe_geom',type='sphere',size=str(radius),contype='4',conaffinity='2')
            m=mujoco.MjModel.from_xml_string(E.tostring(root,encoding='unicode'));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
            pid=m.geom('probe_geom').id
            hits=any(pid in (c.geom1,c.geom2) for c in d.contact)
            self.assertEqual(hits,expect)

    def test_frictionless_control_preserves_floor_friction(self):
        xml,_=make_model(level=1,friction=0,allow_assumed_mass=True)
        m=mujoco.MjModel.from_xml_string(xml)
        self.assertEqual(m.geom('cavity0').condim[0],1)
        self.assertEqual(m.geom('left_ring0_0').condim[0],1)
        self.assertEqual(m.geom('floor').condim[0],3)

    def test_enclosing_radius_has_independent_lower_bound_certificate(self):
        fit=json.loads((BASE/'geometry_receipt.json').read_text())['fit']
        points=np.array(fit['support_points_mm']);weights=np.array(fit['support_weights'])
        center=-np.array(fit['rigid_translation_from_source_pose_mm'])
        self.assertTrue((weights>=0).all());self.assertAlmostEqual(weights.sum(),1.)
        np.testing.assert_allclose(weights@points,center,atol=1e-7)
        lower=math.sqrt(float(weights@(points*points).sum(1)-center@center))
        self.assertAlmostEqual(lower,fit['minimum_enclosing_radius_mm'],places=7)

if __name__=='__main__':unittest.main()
