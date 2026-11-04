"""Independent CAD30 readiness checks. Tests do NOT imply Mink was executed."""
from __future__ import annotations
import os
from pathlib import Path
import unittest
import mujoco
import numpy as np

from mink_cad30_probe import (
    STANCE, actual_violations, benchmark, contact_sites, replay_pose, try_mink,
)

MODEL = Path(os.environ.get('CAD30_MODEL_XML', 'source/mjcf/scene_local30_nominal.xml'))
REPLAY = Path(os.environ.get('CAD30_REPLAY_ZIP', 'FNK0052_claw_tuck_ABC_2026-11-04_v2_no_CAD.zip'))

@unittest.skipUnless(MODEL.is_file() and REPLAY.is_file(), "private CAD30 + replay must be restored locally")
class TestReadiness(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model, cls.q, cls.anchors, cls.err = contact_sites(MODEL, REPLAY)

    def test_four_sites_are_exact_archived_contact_markers(self):
        self.assertEqual(self.model.nsite >= 4, True)
        self.assertLess(self.err, 1e-12)
        d=mujoco.MjData(self.model);d.qpos[:]=self.q;mujoco.mj_forward(self.model,d)
        for leg in STANCE:
            point=np.asarray(d.site_xpos[self.model.site('audit_foot_contact_'+leg).id]);
            self.assertLess(np.linalg.norm(point-self.anchors[leg]),1e-12)

    def test_model_unchanged_by_sites(self):
        original=mujoco.MjModel.from_xml_path(str(MODEL))
        self.assertEqual((original.nq,original.nv,original.nu,original.nmesh),
                         (self.model.nq,self.model.nv,self.model.nu,self.model.nmesh))
        self.assertTrue(np.array_equal(original.body_mass, self.model.body_mass))
        self.assertTrue(np.array_equal(original.body_inertia,self.model.body_inertia))

    def test_research_angle_violation_is_not_silenced(self):
        bad=actual_violations(self.model,self.q)
        self.assertEqual({x['joint'] for x in bad},{'motor31','motor27'})
        self.assertTrue(all(abs(x['position_rad'])>1.7 for x in bad))

    def test_absent_upstream_is_reported_unavailable_not_pass(self):
        try:import mink  # noqa
        except ImportError:
            status=try_mink(self.model,self.q,self.anchors)
            self.assertEqual(status['status'],'UNAVAILABLE')
            self.assertFalse(status['kinematic_gate_pass'])
            self.assertFalse(status['hardware_approved'])
        else:
            self.skipTest('Mink is installed; run explicit solver benchmark instead')

    def test_probe_metadata_remains_reproducible(self):
        data=benchmark(MODEL,REPLAY)
        self.assertEqual(data['mujoco_version'],'3.14.0')
        self.assertEqual(data['qpos_len'],27)
        self.assertEqual(data['nv'],26)
        self.assertFalse(data['mink'].get('hardware_approved'))

if __name__=='__main__':unittest.main()