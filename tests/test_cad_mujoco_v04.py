"""Static checks for pinned external Freenove CAD reconstruction.

No network or MuJoCo required; remote CAD is downloaded and compiled by the
separate GitHub Actions integration workflow.
"""
import hashlib
import math
import unittest

from hexlab.cad_mujoco_v04 import (
    LOCK, SCENARIOS, _box_inertia, _cylinder_inertia, git_blob_sha, pinned_records
)
from hexlab.community_mass_v04 import load, SCENARIOS_PATH


class CadReconstructionUnitTests(unittest.TestCase):
    def test_git_blob_hash_uses_null_byte(self):
        self.assertEqual(git_blob_sha(b"test"),
                         hashlib.sha1(b"blob 4" + bytes((0,)) + b"test").hexdigest())

    def test_asset_sha_lock_contains_real_cad(self):
        spec = load(LOCK)
        self.assertEqual(spec["pr_commit"], "5c9a84c16335b30036e451c691d2c544abf4bc72")
        self.assertEqual(len(spec["meshes"]), 32)
        self.assertEqual(len(pinned_records(spec)), 34)
        self.assertTrue(any(x["name"] == "tibia.stl" for x in spec["meshes"]))
        self.assertTrue(any(x["name"] == "bigservo1.stl" for x in spec["meshes"]))
        self.assertTrue(all(len(x["sha_git_blob"]) == 40 for x in spec["meshes"]))

    def test_inertia_box_is_positive(self):
        m = .046
        x, y, z = _box_inertia(m, (.085, .004, .056))
        self.assertTrue(all(math.isfinite(v) and v > 0 for v in (x,y,z)))
        self.assertAlmostEqual(x, m * (.004**2 + .056**2) / 12)

    def test_18650_cylindrical_inertia(self):
        x, y, z = _cylinder_inertia(.045, .009, .0325)
        self.assertAlmostEqual(x,y)
        self.assertGreater(x,z)
        self.assertAlmostEqual(z, .5 * .045 * .009**2)

    def test_mass_scenario_keys(self):
        mass = load(SCENARIOS_PATH)
        self.assertEqual(SCENARIOS, ("lower", "nominal", "upper"))
        for key in SCENARIOS:
            self.assertGreater(mass["calculations"]["scenario_totals_kg"][key],1.5)


if __name__ == "__main__":
    unittest.main()
