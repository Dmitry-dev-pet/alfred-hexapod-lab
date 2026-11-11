"""Reference checks; set OPENBENI_SOURCE to the pinned upstream checkout."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

try:
    import mujoco
    import numpy as np
    import onnxruntime
except ImportError:
    mujoco = None

SOURCE = Path(os.environ.get("OPENBENI_SOURCE", "/not-configured"))


@unittest.skipUnless(mujoco is not None and (SOURCE/".git").exists(), "Requires verified optional OpenBeni source/runtime")
class ReferenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location("beni_adapter", Path(__file__).with_name("evaluate.py"))
        cls.mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.mod)
        cls.temp = tempfile.TemporaryDirectory()
        cls.model, cls.config = cls.mod.prepare(SOURCE, Path(cls.temp.name)/"scene.xml")
        cls.ctrl = cls.mod.Controller(cls.model, cls.config, SOURCE/"assets/models/policy.onnx")

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_original_javascript_motor_law(self):
        cases = [[n,p,v,a] for n in self.mod.DEFAULT for p in [-2., .3, 1.1, 4.] for v,a in [(-15.,-5.),(0.,0.),(20.,8.)]]
        module = (SOURCE/"web/game/physics.mjs").as_uri()
        script = f"import {{motorTorque}} from '{module}'; const xs={json.dumps(cases)}; console.log(JSON.stringify(xs.map(([n,p,v,a])=>motorTorque(n,p,v,a,['L2','R2'].includes(n)?1.2:1))));"
        expected = json.loads(subprocess.check_output(["node", "--input-type=module", "-e", script], text=True))
        actual = [self.mod.torque(*x, self.config) for x in cases]
        np.testing.assert_allclose(actual, expected, atol=2e-15, rtol=2e-15)

    def test_observation_against_original_javascript(self):
        data = self.mod.initial_data(self.model, dict(id="fixture",axis=1,degrees=37.))
        data.qvel[:] = np.linspace(-.4,.5,self.model.nv)
        mujoco.mj_forward(self.model,data)
        self.ctrl.reset()
        self.ctrl.last = np.linspace(-.5,.5,7,dtype=np.float32)
        fixture=dict(qpos=data.qpos.tolist(),qvel=data.qvel.tolist(),sensordata=data.sensordata.tolist(),
                     joints={k:dict(q=int(v[0]),v=int(v[1])) for k,v in self.ctrl.joints.items()},
                     sensors={k:int(self.model.sensor(k).adr[0]) for k in ["imu_gyro","imu_quat","head_imu_gyro"]},last=self.ctrl.last.tolist())
        module=(SOURCE/"web/game/physics.mjs").as_uri()
        script=f"import {{Simulation}} from '{module}'; const f={json.dumps(fixture)};const s={{data:{{qpos:new Float64Array(f.qpos),qvel:new Float64Array(f.qvel),sensordata:new Float64Array(f.sensordata)}},joints:f.joints,sensors:f.sensors,lastAction:f.last,observationSize:40,jumpSupported:false}}; console.log(JSON.stringify(Array.from(Simulation.prototype.observation.call(s))));"
        expected=json.loads(subprocess.check_output(["node","--input-type=module","-e",script],text=True))
        np.testing.assert_array_equal(self.ctrl.observation(data),np.asarray(expected,np.float32))

    def test_initial_contact_and_torque_bounds(self):
        self.ctrl.reset()
        for axis in [0,1]:
            for angle in [-90,90,180]:
                d=self.mod.initial_data(self.model,dict(id="fixture",axis=axis,degrees=angle))
                self.assertAlmostEqual(self.mod.lowest_collision(self.model,d),.0002,places=10)
                self.assertFalse(self.mod.measurement(self.model,d)["ready"])
        d=self.mod.initial_data(self.model,dict(id="upright",axis=1,degrees=0))
        before=d.qpos.copy()
        for _ in range(4): self.ctrl.apply(d,np.ones(7,np.float32)*100)
        self.assertTrue(np.all(abs(d.ctrl)<=self.model.actuator_ctrlrange[:,1]+1e-12))
        np.testing.assert_array_equal(before,d.qpos)

    def test_recovery_requires_contiguous_hold(self):
        rows=[dict(t=i*.01,ready=i%100!=0,quiet=True,penetration_m=0,tilt_deg=0) for i in range(1001)]
        self.assertFalse(self.mod.analyse(rows,0)["recovered_1s"])
        rows[1:101]=[dict(r,ready=True) for r in rows[1:101]]
        self.assertTrue(self.mod.analyse(rows,0)["recovered_1s"])


if __name__ == "__main__": unittest.main()
