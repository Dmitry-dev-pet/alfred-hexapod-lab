"""Stdlib regression tests for the local-only Hexapod catalog updater."""
import hashlib
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / "tools" / "local_viewer_server.py"
SPEC = importlib.util.spec_from_file_location("hexapod_local_server", SCRIPT)
server = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(server)


class LibraryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / "hexapod").mkdir()
        (self.root / "hexapod/index.html").write_text("<main>tripod</main>")
        self.library = self.root / "hexapod/library.json"
        self.initial = {
            "schema": "hexapod-library/v1",
            "revision": 1,
            "entries": [
                {"id": "tripod-60", "title": "Tripod", "detail": "60 seconds",
                 "duration_s": 60, "href": "/hexapod/", "files": []}
            ]
        }
        self.library.write_text(json.dumps(self.initial))
        p = patch.multiple(server, ROOT=self.root, LIBRARY=self.library)
        p.start()
        self.addCleanup(p.stop)

    def test_path_validation_rejects_traversal(self):
        for name in ("../secret.json", "hexapod/../secret.js",
                     "hexapod/.hidden.json", "hexapod/evil.exe", "assets/stl/foo.stl"):
            with self.subTest(name=name), self.assertRaises(ValueError):
                server.safe_asset_path(name)
        self.assertEqual(server.safe_asset_path("hexapod/demo/app.js"),
                         self.root / "hexapod/demo/app.js")

    def test_catalog_validation_rejects_duplicates(self):
        catalog = json.loads(json.dumps(self.initial))
        catalog["entries"].append(dict(catalog["entries"][0]))
        with self.assertRaises(ValueError):
            server.validate_catalog(catalog)

    def test_refresh_adds_verified_demo_and_keeps_original(self):
        html = b"<html>new demo</html>"
        recording = b'{"format":"example-physical-replay","frames":[[0,1,2]]}'
        files = [("hexapod/new/index.html", html), ("hexapod/data/new.json", recording)]
        new_entry = {
            "id": "new-001", "title": "New MuJoCo record", "detail": "Test fixture",
            "duration_s": 12, "href": "/hexapod/new/",
            "files": [
                {"path": name, "sha256": hashlib.sha256(data).hexdigest(), "size": len(data)}
                for name, data in files
            ]
        }
        catalog = json.loads(json.dumps(self.initial))
        catalog["revision"] = 2
        catalog["entries"].append(new_entry)
        commit = "a" * 40
        mapping = {
            server.BRANCH_API: json.dumps({"commit": {"sha": commit}}).encode(),
            server.RAW_BASE + commit + "/docs/hexapod/library.json":
                json.dumps(catalog).encode(),
            **{server.RAW_BASE + commit + "/docs/" + name: data for name, data in files}
        }
        def fake_fetch(url, limit):
            data = mapping[url]
            if len(data) > limit:
                raise ValueError("Over-sized fixture")
            return data
        with patch.object(server, "remote_bytes", side_effect=fake_fetch):
            result = server.refresh_library()
        self.assertEqual(result["added"], 1)
        self.assertEqual(result["assets"], 2)
        self.assertEqual(json.loads(self.library.read_text())["revision"], 2)
        self.assertEqual((self.root / "hexapod/data/new.json").read_bytes(), recording)
        self.assertEqual((self.root / "hexapod/index.html").read_text(), "<main>tripod</main>")
        # Once synchronized, refreshing the same manifest must be idempotent.
        with patch.object(server, "remote_bytes", side_effect=fake_fetch):
            again = server.refresh_library()
        self.assertEqual(again["added"], 0)
        self.assertEqual(again["assets"], 0)

    def test_mismatched_integrity_does_not_modify_library(self):
        entry = {
            "id": "new-001", "title": "New record", "detail": "bad checksum",
            "duration_s": 12, "href": "/hexapod/new/",
            "files": [{"path": "hexapod/new/index.html", "sha256": "0" * 64, "size": 5}]
        }
        catalog = dict(self.initial, revision=2, entries=self.initial["entries"] + [entry])
        commit = "b" * 40
        mapping = {
            server.BRANCH_API: json.dumps({"commit": {"sha": commit}}).encode(),
            server.RAW_BASE + commit + "/docs/hexapod/library.json":
                json.dumps(catalog).encode(),
            server.RAW_BASE + commit + "/docs/hexapod/new/index.html": b"bad!!",
        }
        with patch.object(server, "remote_bytes", side_effect=lambda u, size: mapping[u]):
            with self.assertRaises(ValueError):
                server.refresh_library()
        self.assertEqual(json.loads(self.library.read_text())["revision"], 1)
        self.assertFalse((self.root / "hexapod/new/index.html").exists())


if __name__ == "__main__":
    unittest.main()
