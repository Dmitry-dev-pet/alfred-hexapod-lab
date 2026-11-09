"""Pure stdlib tests; no network, MuJoCo, CAD vendor software or real STL needed."""
import importlib.util
import struct
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

MODULE = Path(__file__).resolve().parents[1] / "research/bb8/cad_audit.py"
spec = importlib.util.spec_from_file_location("bb8_cad_audit", MODULE)
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def sample_triangle():
    return (b"BB8 synthetic test only".ljust(80, b" ") + struct.pack("<I", 1)
            + struct.pack("<12fH", 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 2, 0, 0))


def sample_manifest(blob):
    return {"schema": audit.SCHEMA, "sources": [{"repo": "example/robot", "revision": "a" * 40,
            "license_status": "test-only", "files": [{"path": "Model/head.stl", "size": len(blob),
            "git_blob_sha": audit.git_blob_sha(blob)}]}]}


class CADSourceTests(unittest.TestCase):
    def test_binary_stl_bounds_and_unknown_units(self):
        box = audit.stl_bounds(sample_triangle())
        self.assertEqual(box["dimensions_native_units_xyz"], [1, 2, 0])
        self.assertFalse(box["units_proven"])

    def test_ascii_stl(self):
        raw = b"solid s\nfacet normal 0 0 1\nouter loop\nvertex -1 2 3\nvertex 0 0 0\nvertex 2 1 2\nendloop\nendfacet\nendsolid s"
        self.assertEqual(audit.stl_bounds(raw)["dimensions_native_units_xyz"], [3, 2, 3])

    def test_strict_blob_validation(self):
        raw = sample_triangle()
        entry = sample_manifest(raw)["sources"][0]["files"][0]
        audit.verify_payload(raw, entry)
        with self.assertRaises(ValueError):
            audit.verify_payload(raw[:-1] + b"x", entry)

    def test_reject_malicious_path_and_unpinned_revision(self):
        manifest = sample_manifest(sample_triangle())
        manifest["sources"][0]["files"][0]["path"] = "../secret"
        with self.assertRaises(ValueError):
            audit.validate_manifest(manifest)
        manifest = sample_manifest(sample_triangle())
        manifest["sources"][0]["revision"] = "main"
        with self.assertRaises(ValueError):
            audit.validate_manifest(manifest)

    def test_cache_verified_without_network(self):
        raw = sample_triangle()
        manifest = sample_manifest(raw)
        audit.validate_manifest(manifest)
        with TemporaryDirectory() as name:
            cache = Path(name)
            path = audit.local_path(cache, manifest["sources"][0], manifest["sources"][0]["files"][0])
            path.parent.mkdir(parents=True)
            path.write_bytes(raw)
            report = audit.execute(manifest, cache, "inspect")
            self.assertEqual(report["not_cached"], 0)
            self.assertEqual(report["files"][0]["geometry"]["dimensions_native_units_xyz"], [1, 2, 0])


if __name__ == "__main__":
    unittest.main()
