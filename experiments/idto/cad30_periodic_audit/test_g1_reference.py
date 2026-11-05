"""Unit/regression tests. No physical controller or gait PASS asserted."""
import importlib.util
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
import numpy as np
from generate_reference import sample_nodes,mujoco_to_drake,generate
from inverse_dynamics_gate import read_trace

SOURCE=Path(os.getenv('CAD30_MATH_TRACE_ZIP', '/nonexistent/CAD30-math-trace.zip'))
class WholePeriodDraftTests(unittest.TestCase):
    def test_q_axis_mapping(self):
        q=np.arange(27)
        d=mujoco_to_drake(q)
        self.assertTrue(np.array_equal(d[:4],[3,4,5,6]))
        self.assertTrue(np.array_equal(d[4:7],[0,1,2]))
        self.assertTrue(np.array_equal(d[7:],q[7:]))

    def test_every_swing_present_and_periodic_nominal(self):
        if not SOURCE.exists():self.skipTest("CAD30_MATH_TRACE_ZIP is not available")
        nodes=read_trace(SOURCE)
        indices,p=sample_nodes(nodes,steps=28)
        self.assertEqual(len(indices),29)
        self.assertEqual(indices[0],106);self.assertEqual(indices[-1],217)
        for leg in ('ml','mr','bl','br'):
            self.assertTrue(any(x['phase']==f'swing_{leg}' for x in p))
        a,b=mujoco_to_drake(p[0]['qpos']),mujoco_to_drake(p[-1]['qpos'])
        self.assertAlmostEqual(b[5]-a[5],.02,places=8)
        self.assertLess(np.max(abs(b[7:25]-a[7:25])),.03)

    def test_source_summary_is_not_a_physical_gait(self):
        for x in ('inverse_gate_0.06.json','inverse_gate_015.json'):
            path=Path(__file__).parent/x
            if not path.exists():self.skipTest('receipt not available')
            r=json.loads(path.read_text())
            self.assertFalse(r['g1_verified_period'])
            self.assertFalse(r['g2_free_base_simulated'])
            self.assertEqual(r['nodes_tested'],218)
            self.assertEqual(r['feasible_samples'],218)

    def test_invalid_trajectory_rejected(self):
        with self.assertRaises(ValueError):mujoco_to_drake([1,2,3])
        if SOURCE.exists():
            with self.assertRaises(ValueError):sample_nodes(read_trace(SOURCE),steps=3)

    def test_output_contains_no_cad_meshes(self):
        if not SOURCE.exists():self.skipTest("CAD30_MATH_TRACE_ZIP is not available")
        with tempfile.TemporaryDirectory() as td:
            m=generate(SOURCE,td,steps=28)
            self.assertEqual(m['requested_swings'],['ml','mr','bl','br'])
            self.assertEqual(len(list(Path(td).glob('*'))),3)
            import csv
            with open(Path(td)/'nominal_4swing_drake.csv') as f:rows=list(csv.reader(f))
            self.assertEqual(len(rows),29)
            self.assertTrue(all(len(row)==27 for row in rows))

if __name__=='__main__':unittest.main()