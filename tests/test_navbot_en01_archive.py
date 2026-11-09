"""Offline EN01 archive import must be hash-exact, idempotent and fail closed."""
from __future__ import annotations

import hashlib
import json
import pathlib
import sys
import tempfile
import unittest
from zipfile import ZipFile

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from navbot_en01_replays import MODES, restore_verified_archive


class EN01ArchiveImportTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = pathlib.Path(self.temp.name)
        self.archive = self.root / "recorded-evidence.zip"
        self.raw = {
            mode: json.dumps({"mode": mode.replace("-", "_"), "source": "measured",
                              "frames": [1, 2], "unique": mode}).encode()
            for mode in MODES
        }
        self.hashes = {mode: hashlib.sha256(raw).hexdigest()
                       for mode, raw in self.raw.items()}

    def make_zip(self, *, omit=(), duplicate=None, corrupt=None):
        with ZipFile(self.archive, "w") as z:
            for mode, raw in self.raw.items():
                if mode in omit:
                    continue
                if mode == corrupt:
                    raw += b"corrupted"
                # Nested paths are safe: names are never extracted.
                z.writestr("nested/physical/" + mode + ".json", raw)
                if mode == duplicate:
                    z.writestr("other/" + mode + ".json", raw)
            z.writestr("irrelevant/metadata.json", b'{"not": "a verified trace"}')

    def test_restore_exact_sources_twice(self):
        self.make_zip()
        result = restore_verified_archive(self.archive, self.root, self.hashes)
        self.assertEqual(len(result), 4)
        self.assertEqual({r["mode"] for r in result}, set(MODES))
        for mode in MODES:
            p = self.root / "results/navbot_en01" / (
                mode.replace("-", "_") + "_verified.json")
            self.assertEqual(p.read_bytes(), self.raw[mode])
        self.assertEqual(result, restore_verified_archive(self.archive, self.root, self.hashes))

    def test_missing_one_prevents_all_writes(self):
        self.make_zip(omit=("jump",))
        with self.assertRaisesRegex(ValueError, "lacks verified"):
            restore_verified_archive(self.archive, self.root, self.hashes)
        self.assertFalse((self.root / "results").exists())

    def test_corrupted_trace_prevents_all_writes(self):
        self.make_zip(corrupt="stand")
        with self.assertRaisesRegex(ValueError, "lacks verified"):
            restore_verified_archive(self.archive, self.root, self.hashes)
        self.assertFalse((self.root / "results").exists())

    def test_duplicate_raw_member_rejected(self):
        self.make_zip(duplicate="stand")
        with self.assertRaisesRegex(ValueError, "Duplicate verified"):
            restore_verified_archive(self.archive, self.root, self.hashes)

    def test_existing_mutated_raw_file_is_never_overwritten(self):
        self.make_zip()
        target = self.root / "results/navbot_en01/stand_verified.json"
        target.parent.mkdir(parents=True)
        target.write_bytes(b"previous user's trace")
        with self.assertRaisesRegex(ValueError, "refusing overwrite"):
            restore_verified_archive(self.archive, self.root, self.hashes)
        self.assertEqual(target.read_bytes(), b"previous user's trace")


if __name__ == "__main__":
    unittest.main()
