import tempfile
import unittest
from pathlib import Path
import numpy as np
from g0b_contact_audit import drake_probe, LEGS, sha256

class G0BContractTests(unittest.TestCase):
    def test_drake_parser_extracts_real_foot_name_and_last_float(self):
        with tempfile.TemporaryDirectory() as t:
            path=Path(t)/'probe.txt'
            path.write_text('meta 27 26 20\nactuator motor12_ctrl 0.7 0.002\n'
                            'mass_diagonal 1.0 2.0\n'
                            'ground_pair big_hexapod scene::tibiaml_collision_0 floor -2.274e-6\n'
                            'drake_fz tibiaml 0.45\npoint_contact_count 1\n')
            v=drake_probe(path)
            self.assertAlmostEqual(v['gaps']['ml'],-2.274e-6)
            self.assertAlmostEqual(v['fz']['ml'],0.45)
            self.assertEqual(v['point_contact_count'],1)
            self.assertEqual(v['mass_diag'].tolist(),[1.,2.])
    def test_drake_parser_refuses_incomplete(self):
        with tempfile.TemporaryDirectory() as t:
            path=Path(t)/'bad.txt';path.write_text('meta 27 26 20\n')
            with self.assertRaisesRegex(ValueError,'Incomplete'):drake_probe(path)
    def test_sha_is_content_based(self):
        with tempfile.TemporaryDirectory() as t:
            path=Path(t)/'x';path.write_bytes(b'hello')
            self.assertEqual(sha256(path),'2cf24dba5fb0a30e26e83b2ac5b9e29e1b161e5c1fa7425e73043362938b9824')
    def test_expected_six_legs(self):
        self.assertEqual(set(LEGS),{'ml','mr','bl','br','fl','fr'})

if __name__=='__main__':unittest.main()
