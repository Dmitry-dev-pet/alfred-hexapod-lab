"""Offline, dependency-free tests of the owner-private Lite3 Mac install contract."""
from __future__ import annotations

import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tarfile
import tempfile
import unittest
from unittest.mock import patch
import zipfile

MODULE = Path(__file__).resolve().parents[1] / "scripts/research/install_lite3_mac.py"
spec = importlib.util.spec_from_file_location("lite3_mac_installer", MODULE)
installer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(installer)


class Lite3MacInstallTests(unittest.TestCase):
    def test_safe_paths(self):
        for path in ("../foo", "/etc/passwd", "a/../../foo", r"bad\winpath"):
            with self.assertRaises(ValueError):
                installer.checked_member_path(Path("/tmp/root"), path)
        self.assertEqual(
            installer.checked_member_path(Path("/tmp/root"), "a/b"),
            Path("/tmp/root/a/b"),
        )

    def test_private_bundle_verified_unpacked_and_idempotent(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            scene = b'<mujoco model="unit_test"/>'
            onnx = b"fake-onnx-bytes"
            tar_io = io.BytesIO()
            with tarfile.open(fileobj=tar_io, mode="w:gz") as tar:
                entries = {"src/assets/robots/lite3/xmls/scene_lite3.xml": scene}
                entries.update({
                    f"src/assets/robots/lite3/xmls/assets/M{i}.STL": b"stl"
                    for i in range(17)
                })
                entries["src/tasks/velocity/config/lite3_walk/env_cfgs.py"] = b"config"
                for name, data in entries.items():
                    item = tarfile.TarInfo(name)
                    item.size = len(data)
                    tar.addfile(item, io.BytesIO(data))
            files = {
                name: (onnx if name.endswith("policy.onnx") else b"test")
                for name in installer.ASSET_MEMBERS if not name.endswith(".tar.gz")
            }
            files["lite3-upstream-model-task.tar.gz"] = tar_io.getvalue()
            receipt = {
                "files": {
                    name: {"sha256": hashlib.sha256(data).hexdigest(), "size": len(data)}
                    for name, data in files.items()
                }
            }
            bundle = root / "private.zip"
            with zipfile.ZipFile(bundle, "w") as z:
                z.writestr("RECEIPT.json", json.dumps(receipt))
                for name, data in files.items():
                    z.writestr(name, data)
            with (
                patch.object(installer, "BUNDLE_SHA256", installer.sha256_path(bundle)),
                patch.object(installer, "UPSTREAM_SHA256", hashlib.sha256(tar_io.getvalue()).hexdigest()),
                patch.object(installer, "SCENE_SHA256", hashlib.sha256(scene).hexdigest()),
                patch.object(installer, "ONNX_SHA256", hashlib.sha256(onnx).hexdigest()),
            ):
                target = root / "original_assets"
                installer.install_assets(bundle, target)
                self.assertTrue(installer.valid_assets(target))
                self.assertEqual(len(list(target.rglob("*.STL"))), 17)
                installer.install_assets(bundle, target)
                self.assertTrue(
                    (target / "upstream/src/tasks/velocity/config/lite3_walk/env_cfgs.py").exists()
                )

    def test_bad_archive_is_rejected_without_side_effect(self):
        with tempfile.TemporaryDirectory() as td:
            archive = Path(td) / "untrusted.zip"
            archive.write_bytes(b"not-the-pinned-archive")
            with self.assertRaisesRegex(ValueError, "SHA256"):
                installer.install_assets(archive, Path(td) / "assets")
            self.assertFalse((Path(td) / "assets").exists())


if __name__ == "__main__":
    unittest.main()
