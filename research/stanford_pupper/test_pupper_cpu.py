"""Source-locked regression tests for no-training Stanford Pupper CPU preflight."""
import hashlib
import math
from pathlib import Path
import unittest
import mujoco
from reproduce_pupper import (
    SOURCE_BLOB_SHA1, DEFAULT_POSE, PUPPER_JOINTS, FOOT_GEOMS,
    git_blob_sha, read_pinned_xml, build_cpu_mjcf, load_model, run_free_stance,
)

XML = Path(__file__).with_name('pupper_v3.xml')

class OriginalModelSmokeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.raw = read_pinned_xml(XML)
        cls.model = load_model(cls.raw)

    def test_source_exact_git_blob(self):
        self.assertEqual(git_blob_sha(self.raw), SOURCE_BLOB_SHA1)
        with self.assertRaisesRegex(ValueError, 'SHA mismatch'):
            read_pinned_xml(self._corrupted_copy())

    def _corrupted_copy(self):
        p = Path(__file__).with_name('_altered_model.xml')
        p.write_bytes(self.raw.replace(b'pupper_v3', b'pupper_vX', 1))
        self.addCleanup(p.unlink)
        return p

    def test_compile_collisions_and_original_mass(self):
        m = self.model
        self.assertEqual((m.nq, m.nv, m.nu), (19, 18, 12))
        self.assertAlmostEqual(sum(m.body_mass), 3.17, places=8)
        self.assertEqual(len(PUPPER_JOINTS), 12)
        self.assertEqual(len(FOOT_GEOMS), 4)
        self.assertEqual(m.opt.timestep, 0.004)
        self.assertEqual(m.opt.iterations, 2)
        self.assertEqual(m.opt.ls_iterations, 5)
        self.assertEqual(len(DEFAULT_POSE), 12)
        self.assertEqual(sum(1 for i in range(m.ngeom) if m.geom_type[i] == mujoco.mjtGeom.mjGEOM_SPHERE), 8)
        for name in FOOT_GEOMS:
            self.assertGreaterEqual(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name), 0)

    def test_no_mesh_collision_change(self):
        s = build_cpu_mjcf(self.raw)
        self.assertNotIn('type="mesh"', s)
        self.assertIn('leg_front_r_3_collision', s)
        self.assertIn('leg_back_l_3_foot_site', s)

    def test_physical_contacts_and_reproducibility(self):
        a = run_free_stance(self.model, duration_s=1)
        b = run_free_stance(self.model, duration_s=1)
        self.assertEqual(a, b, 'Identical initial conditions must replay deterministically')
        self.assertEqual(a['mj_step_count'], 250)
        self.assertEqual(a['stored_frames'], 50)
        self.assertTrue(math.isfinite(a['final_base_height_m']))
        self.assertGreaterEqual(a['any_contact_sample_count'], 1)
        self.assertLessEqual(a['peak_abs_actuator_force_Nm'], 3.00001)
        self.assertTrue(all(len(row['joint_q']) == 12 for row in a['frames']))

if __name__ == '__main__':
    unittest.main()
