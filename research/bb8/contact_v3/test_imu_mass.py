import inspect,json,math,struct,tempfile,unittest,hashlib
from pathlib import Path
import numpy as np
import mujoco
from masses import solid_moments,aggregate
from imu_control import Controller

BASE=Path(__file__).parent

class MassAndIMUTests(unittest.TestCase):
    def test_translated_box_mass_moments_have_metres_and_parallel_axis_removed(self):
        v=np.array([[-5,-10,-15],[5,-10,-15],[5,10,-15],[-5,10,-15],[-5,-10,15],[5,-10,15],[5,10,15],[-5,10,15]],float)+[15,-20,30]
        faces=[(0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,1,5),(0,5,4),(1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,0,4),(3,4,7)]
        blob=b'box'.ljust(80,b' ')+struct.pack('<I',len(faces))
        for ids in faces:blob+=struct.pack('<12fH',*[0.,0.,0.],*v[list(ids)].ravel(),0)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'box.stl';path.write_bytes(blob);volume,com,I=solid_moments(path,hashlib.sha256(blob).hexdigest())
        self.assertAlmostEqual(volume,.01*.02*.03)
        np.testing.assert_allclose(com,[.015,-.020,.030],atol=1e-12)
        expected=np.diag([(.02**2+.03**2)/12,(.01**2+.03**2)/12,(.01**2+.02**2)/12])
        np.testing.assert_allclose(I,expected,atol=1e-12)

    def test_aggregate_uses_parallel_axis_theorem(self):
        components=[]
        for group,x in [('chassis',-1.),('chassis',1.),('left',0.),('right',0.)]:
            components.append({'group':group,'mass_kg':1.,'center_m':[x,0,0],'inertia_COM_kg_m2':np.eye(3).tolist()})
        groups=aggregate(components)
        np.testing.assert_allclose(groups['chassis']['center_m'],[0,0,0])
        np.testing.assert_allclose(groups['chassis']['inertia_COM_kg_m2'],np.diag([2,4,4]))

    def test_battery_and_motor_have_explicit_estimated_masses(self):
        masses=[]
        for profile in ['light','nominal','heavy']:
            p=json.loads((BASE/'mass_profiles'/f'{profile}.json').read_text());masses.append(p['total_mass_without_head_kg'])
            self.assertFalse(p['hardware_calibrated']);self.assertTrue(p['head_excluded'])
            self.assertEqual(len(p['components']),49)
            battery=next(x for x in p['components'] if x['id']=='c54')
            self.assertEqual(battery['status'],'UNIDENTIFIED_BATTERY_ESTIMATE')
            self.assertAlmostEqual(sum(g['mass_kg'] for g in p['groups'].values()),p['total_mass_without_head_kg'])
            for g in p['groups'].values():self.assertGreater(np.linalg.eigvalsh(g['inertia_COM_kg_m2']).min(),0)
        self.assertEqual(masses,sorted(masses))

    def test_static_accelerometer_tilt_and_controller_sign(self):
        for angle in [-.15,-.07,0.,.08,.2]:
            c=Controller(zero=angle,kv=0,ki=0)
            output=c.update(np.zeros(3),np.array([-9.81*math.sin(angle),0.,9.81*math.cos(angle)]),np.zeros(2))
            self.assertAlmostEqual(c.pitch,angle,places=12);np.testing.assert_allclose(output,0,atol=1e-12)
        c=Controller(kv=0,ki=0);output=c.update(np.zeros(3),[-9.81*math.sin(.1),0,9.81*math.cos(.1)],np.zeros(2))
        self.assertGreater(output.mean(),0)

    def test_gyro_units_integrate_radians_without_acceleration_fusion(self):
        c=Controller(kv=0,ki=0);c.update(np.zeros(3),[0,0,9.81],np.zeros(2))
        for _ in range(100):c.update([0,.1,0],[0,0,0],np.zeros(2))
        self.assertAlmostEqual(c.pitch,.1,places=12)

    def test_controller_accepts_only_sensor_feedback_and_commands(self):
        self.assertEqual(list(inspect.signature(Controller.update).parameters),['self','gyro','accel','encoder','target_speed','target_yaw'])
        source=inspect.getsource(Controller)
        for word in ['qpos','qvel','xmat','xpos','mujoco']:self.assertNotIn(word,source)

    def test_compiled_body_mass_centers_and_inertias_match_component_aggregation(self):
        validation=BASE.parents[2]/'validation'
        if not (validation/'nominal-model.xml').is_file():
            self.skipTest('Emitted-model check requires the archived validation XML beside research/')
        profile=json.loads((BASE/'mass_profiles/nominal.json').read_text())
        m=mujoco.MjModel.from_xml_path(str(validation/'nominal-model.xml'))
        for group,spec in profile['groups'].items():
            body=m.body(group+'_wheel' if group in ['left','right'] else group).id
            self.assertAlmostEqual(m.body_mass[body],spec['mass_kg'],places=10)
            np.testing.assert_allclose(m.body_ipos[body],spec['center_m'],atol=1e-11)
            R=np.zeros(9);mujoco.mju_quat2Mat(R,m.body_iquat[body]);R=R.reshape(3,3)
            tensor=R@np.diag(m.body_inertia[body])@R.T
            np.testing.assert_allclose(tensor,spec['inertia_COM_kg_m2'],rtol=1e-9,atol=1e-12)

if __name__=='__main__':unittest.main()
