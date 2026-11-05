import tempfile
from pathlib import Path
import unittest
import numpy as np
from compare_parity import parse_drake_probe

class DrakeProbeTests(unittest.TestCase):
    def test_parser_rejects_truncated(self):
        with tempfile.TemporaryDirectory() as folder:
            f=Path(folder)/"x.txt"
            f.write_text("meta 27 26 48 20 1.8048\n",encoding="utf-8")
            with self.assertRaises(ValueError): parse_drake_probe(f)

    def test_parser_accepts_complete_synthetic(self):
        with tempfile.TemporaryDirectory() as folder:
            f=Path(folder)/"x.txt"
            f.write_text("meta 27 26 48 20 1.8048\ncom 0 0 0\nq0 1 0\ngravity 0 0\nmrow 0 1 2\nmrow 1 3 4\nbodyxyz tibiaml 1 2 3\n",encoding="utf-8")
            r=parse_drake_probe(f)
            self.assertEqual(r["M"].shape,(2,2))
            np.testing.assert_allclose(r["bodies"]["tibiaml"],[1,2,3])

if __name__=="__main__": unittest.main()
