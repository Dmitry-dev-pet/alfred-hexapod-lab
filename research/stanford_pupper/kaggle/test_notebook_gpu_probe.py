import importlib.util
import json
from pathlib import Path
import unittest
spec = importlib.util.spec_from_file_location("notebook_gpu_probe", Path(__file__).with_name("notebook_gpu_probe.py"))
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


class NotebookGpuProbeTests(unittest.TestCase):
    def test_default_is_private_notebook_gpu_t4_without_cpu_pin(self):
        m = mod.metadata("dmitriiglukhov")
        self.assertEqual(m["kernel_type"], "notebook")
        self.assertTrue(m["enable_gpu"])
        self.assertEqual(m["machine_shape"], "NvidiaTeslaT4")
        self.assertEqual(m["docker_image_pinning_type"], "latest")
        self.assertNotIn("docker_image", m)
        self.assertFalse(m["enable_internet"])
        self.assertTrue(m["is_private"])
        self.assertEqual(m["title"].lower().replace(" ", "-"), mod.SLUG)

    def test_notebook_contract(self):
        d = mod.notebook()
        self.assertEqual(d["nbformat"], 4)
        self.assertEqual(d["cells"][0]["cell_type"], "code")
        self.assertIn("torch.cuda.is_available()", "".join(d["cells"][0]["source"]))
        self.assertNotIn("pip install", "".join(d["cells"][0]["source"]))
        json.dumps(d)

    def test_invalid_handle(self):
        with self.assertRaises(ValueError):
            mod.metadata("oops@email")


if __name__ == "__main__":
    unittest.main()
