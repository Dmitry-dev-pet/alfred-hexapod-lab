"""Unit tests for Kaggle metadata. Never call the Kaggle API."""
import importlib.util
import unittest
from pathlib import Path

MODULE_PATH = Path(__file__).with_name("submit.py")
spec = importlib.util.spec_from_file_location("pupper_kaggle_submit", MODULE_PATH)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class KagglePreflightMetadataTests(unittest.TestCase):
    def test_private_gpu_and_internet(self):
        data = module.build_metadata("test-user")
        self.assertEqual(data["id"], "test-user/stanford-pupper-v3-mjlab-cuda-preflight")
        self.assertTrue(data["is_private"])
        self.assertTrue(data["enable_gpu"])
        self.assertTrue(data["enable_internet"])
        self.assertEqual(data["machine_shape"], "NvidiaTeslaT4")
        self.assertEqual(data["code_file"], "preflight.py")

    def test_bad_username_fails_closed(self):
        for name in ("", "user/name", "../../bad", "user name"):
            with self.subTest(name=name):
                with self.assertRaises(ValueError):
                    module.build_metadata(name)


if __name__ == "__main__":
    unittest.main()
