"""No-network regression tests for the Mini Wheelbot upstream CAD source gate."""
import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import unittest

SCRIPT = (Path(__file__).resolve().parents[1] /
          "research/mini_wheelbot_v2/audit_original_cad.py")
SPEC = importlib.util.spec_from_file_location("wheelbot_cad_gate", SCRIPT)
gate = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(gate)


def fixture_stl():
    return b" " * 80 + struct.pack("<I", 1) + struct.pack(
        "<12fH", 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0)


class OfficialCadGateTests(unittest.TestCase):
    def test_stl_bounds_reads_all_three_vertices(self):
        b = gate.stl_bounds(fixture_stl())
        self.assertEqual(b["triangles"], 1)
        self.assertEqual(b["min_raw"], [0, 0, 0])
        self.assertEqual(b["max_raw"], [1, 1, 0])
        self.assertEqual(b["units"], "not established (inspect CAD/STEP before scale)")

    def test_stl_rejects_truncation_and_nonfinite(self):
        with self.assertRaises(ValueError):
            gate.stl_bounds(fixture_stl()[:-1])
        broken = bytearray(fixture_stl())
        struct.pack_into("<f", broken, 84 + 12, float("nan"))
        with self.assertRaises(ValueError):
            gate.stl_bounds(bytes(broken))

    def test_git_blob_digest_is_content_addressed(self):
        b = fixture_stl()
        expected = hashlib.sha1(b"blob " + str(len(b)).encode() + bytes([0]) + b).hexdigest()
        self.assertEqual(gate.git_blob_sha1(b), expected)
        with self.assertRaises(ValueError):
            gate.verify_asset("hardware/mechanical/meshes/mini-wheelbot.stl", b)

    def test_calibrated_inertias_remain_separate_from_stl(self):
        params = dict(
            m_WR=.154800009999768, m_B=.22470645904855896,
            I_Wxz_Ryz=6.360999999966876e-5, I_Wy_Rx=.00010279151990686121,
            I_Bx=.0001099900000194522, I_By=.0004500099999982192,
            I_Bz=.0002850099999992028, r_W=.032, l_WB=.065,
            fric_magn=.002802071865854826, fric_slope=119.99742098981706)
        result = gate.audit_parameters(json.dumps(params).encode())
        self.assertAlmostEqual(result["computed_total_mass_kg"], .5343064790480949)
        params["m_B"] = -1
        with self.assertRaises(ValueError):
            gate.audit_parameters(json.dumps(params).encode())


if __name__ == "__main__":
    unittest.main()
