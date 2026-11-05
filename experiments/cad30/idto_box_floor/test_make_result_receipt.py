import unittest
from make_result_receipt import extract

class ReceiptTests(unittest.TestCase):
    def test_rejects_failed_or_incomplete(self):
        self.assertEqual(extract('',0)['status'],'SOLVER_NOT_COMPLETED')
        self.assertEqual(extract('|  0 | 12 |\n',1)['status'],'SOLVER_NOT_COMPLETED')

    def test_rejects_missing_fields(self):
        with self.assertRaises(ValueError):
            extract('|  0 | 12 |\nMax actuated torque : 0.1\n',0)

    def test_parses_saved_local_result_but_never_certifies_gait(self):
        from pathlib import Path
        r=extract((Path(__file__).parent/'results/idto_60_iter_stdout.log').read_text(),0)
        self.assertEqual(r['iteration_rows'],60)
        self.assertFalse(r['full_periodic_cycle'])
        self.assertFalse(r['three_support_gate_checked'])
        self.assertAlmostEqual(r['requested_terminal_base_y_displacement_mm'],20., places=6)
        self.assertGreater(r['planned_terminal_base_y_displacement_mm'],0.)
        self.assertLess(r['planned_terminal_base_y_displacement_mm'],20.)
if __name__=='__main__': unittest.main()