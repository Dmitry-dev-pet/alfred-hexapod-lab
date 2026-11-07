"""Cross-check the 20x48 NumPy history rotation with the C++14 sequence Stanford uses.

This verifies the history-buffer math, NOT numerical parity of the complete C++
neural controller or trained policy.
"""
import shutil
import subprocess
import tempfile
from pathlib import Path
import unittest

import numpy as np

from history_contract import advance_history

CPP = r"""
#include <algorithm>
#include <iostream>
#include <vector>
int main() {
    std::vector<float> observation(20 * 48);
    for (int i = 0; i < 20 * 48; i++) observation[i] = static_cast<float>(i);
    std::rotate(observation.rbegin(), observation.rbegin() + 48, observation.rend());
    for (int i = 0; i < 12; i++)
        observation[24 + i] = static_cast<float>(i) * 0.125f;
    for (float v : observation) std::cout << v << '\n';
}
"""


class PupperHistoryCppParityTest(unittest.TestCase):
    @unittest.skipUnless(shutil.which('g++'), 'C++ compiler not installed')
    def test_history_rotation_matches_stanford_cpp14(self):
        with tempfile.TemporaryDirectory() as tempdir:
            src = Path(tempdir) / 'native.cpp'
            exe = Path(tempdir) / 'native'
            src.write_text(CPP)
            subprocess.run(
                ['g++', '-std=c++14', '-O2', str(src), '-o', str(exe)],
                check=True, capture_output=True, text=True,
            )
            expected = np.fromstring(
                subprocess.check_output([str(exe)], text=True), sep=' '
            ).reshape(20, 48)
        actual = np.arange(20 * 48, dtype=np.float32).reshape(20, 48)
        advance_history(
            actual, np.arange(12, dtype=np.float32) * np.float32(0.125)
        )
        np.testing.assert_array_equal(actual, expected)


if __name__ == '__main__':
    unittest.main()
