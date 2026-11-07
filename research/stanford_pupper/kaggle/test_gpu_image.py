"""Offline contract for fresh Kaggle GPU-image kernel. No GPU allocation."""
import unittest
from submit_gpu_image import gpu_image_metadata, OFFICIAL_KAGGLE_GPU_IMAGE


class GPUImageMetadataTests(unittest.TestCase):
    def test_official_gpu_image_and_privacy(self):
        m = gpu_image_metadata("dmitriiglukhov")
        self.assertEqual(m["id"], "dmitriiglukhov/stanford-pupper-v3-gpu-image-check")
        self.assertEqual(m["title"], "Stanford Pupper v3 GPU Image Check")
        self.assertEqual(m["docker_image"], OFFICIAL_KAGGLE_GPU_IMAGE)
        self.assertEqual(m["docker_image"], "gcr.io/kaggle-gpu-images/python")
        self.assertTrue(m["is_private"])
        self.assertTrue(m["enable_gpu"])
        self.assertTrue(m["enable_internet"])
        self.assertEqual(m["machine_shape"], "NvidiaTeslaT4")


if __name__ == "__main__":
    unittest.main()
