import json, os, tempfile, unittest
from pathlib import Path
import mujoco
import numpy as np
from research.navbot_en01.contact_model import create
from research.navbot_en01.linkage_model import create as old_create, set_configuration, load_evidence
from research.navbot_en01.contact_mesh_audit import MeshAudit

MESH = os.environ.get('EN01_CAD_MESH')
DATA = os.environ.get('EN01_CONTACT_DATA')
OLD = os.environ.get('EN01_JUMP_RAW_TRACE')


@unittest.skipUnless(MESH, 'Pinned external CAD mesh supplied separately')
class ContactRepairTests(unittest.TestCase):
    def test_inertias_are_preserved_and_contact_pairs_are_enabled(self):
        old = mujoco.MjModel.from_xml_string(old_create())
        new = mujoco.MjModel.from_xml_string(create(MESH))
        self.assertAlmostEqual(sum(new.body_mass), .322)
        for name in ('body_mass', 'body_inertia', 'body_ipos', 'body_iquat'):
            np.testing.assert_allclose(getattr(old, name), getattr(new, name), atol=1e-14, rtol=0)
        self.assertEqual(new.nmesh, 28)
        for name in ('hip_left', 'hip_right'):
            jid = mujoco.mj_name2id(new, mujoco.mjtObj.mjOBJ_JOINT, name)
            np.testing.assert_allclose(new.jnt_range[jid], [0., .65])
        for name in ('cad_43', 'cad_62'):
            gid = mujoco.mj_name2id(new, mujoco.mjtObj.mjOBJ_GEOM, name)
            self.assertEqual(new.geom_contype[gid] & new.geom_conaffinity[gid], 4)

    def test_continuous_full_torque_is_stopped_without_pose_clipping(self):
        model = mujoco.MjModel.from_xml_string(create(MESH, fixed_base=True, gravity=False, floor_contact=False))
        points = load_evidence()['sides']['cad_z_positive']['points_xy_mm']
        extrema, peak_contact = [], 0.
        for sign in (-1, 1):
            data = mujoco.MjData(model)
            set_configuration(model, data, points, .1)
            self.assertEqual(data.ncon, 0)
            low, high = .1, .1
            for _ in range(12000):
                data.ctrl[:] = [sign*.14709975, 0, sign*.14709975, 0]
                mujoco.mj_step(model, data)
                low = min(low, float(min(data.qpos[[0, 4]])))
                high = max(high, float(max(data.qpos[[0, 4]])))
                force = np.zeros(6)
                for i in range(data.ncon):
                    mujoco.mj_contactForce(model, data, i, force)
                    peak_contact = max(peak_contact, float(force[0]))
            self.assertGreaterEqual(low, 0.)
            self.assertLessEqual(high, .65)
            extrema.append((low, high))
        self.assertLess(extrema[0][0], .02)
        self.assertGreater(extrema[1][1], .63)
        self.assertGreater(peak_contact, .01)

    def test_untrusted_cad_is_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td)/'mesh.json'
            path.write_text('{}')
            with self.assertRaisesRegex(ValueError, 'mesh mismatch'):
                create(path)

    @unittest.skipUnless(OLD, 'Frozen v4 trace supplied separately')
    def test_independent_mesh_check_detects_old_folded_record(self):
        record = json.loads(Path(OLD).read_bytes())
        frame = min(record['frames'], key=lambda f: min(f['qpos'][7], f['qpos'][11]))
        audit = MeshAudit(MESH, old_create())
        self.assertTrue(audit.intersections(frame['body_poses']))

    @unittest.skipUnless(DATA, 'Final v5 battery supplied separately')
    def test_all_cases_have_unclipped_geometry_checks_and_source_timing(self):
        rows = json.loads((Path(DATA)/'RESULTS.json').read_bytes())
        self.assertEqual(len(rows), 9)
        for row in rows:
            summary, audit = row['summary'], row['export']
            self.assertGreaterEqual(min(summary['hip_min_all_steps_rad']), 0.)
            self.assertLessEqual(max(summary['hip_max_all_steps_rad']), .65)
            self.assertEqual(audit['cad_surface_audit']['unexpected_intersection_frames'], 0)
            self.assertEqual(audit['qpos_body_residual'], 0)
        jump = next(r for r in rows if r['name'] == 'jump-p5')['summary']
        np.testing.assert_allclose([e['t'] for e in jump['command_events']], [1.5, 1.645, 2.45], atol=1e-10)
        self.assertGreater(jump['stable_landing_s'], 2.)
        self.assertLess(jump['max_closure_error_m'], .0001)

    @unittest.skipUnless(DATA, 'Final v5 battery supplied separately')
    def test_forged_render_pose_is_rejected_before_cad_audit(self):
        from research.navbot_en01.contact_export import export
        root = Path(DATA)
        record = json.loads((root/'raw/jump-p5_raw.json').read_bytes())
        record['frames'][0]['body_poses'][2][0] += .02
        with tempfile.TemporaryDirectory() as td:
            path = Path(td)/'forged.json'
            path.write_text(json.dumps(record))
            with self.assertRaisesRegex(ValueError, 'poses disagree'):
                export(path, Path(td)/'replay.json', model_path=root/'model-0.00025.xml', mesh_path=MESH)


if __name__ == '__main__':
    unittest.main()
