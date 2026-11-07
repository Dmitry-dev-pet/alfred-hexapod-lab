import unittest
import numpy as np
from history_contract import advance_history, LAST_ACTION, GAIT_REFERENCE


class HistoryContractTests(unittest.TestCase):
    def test_rotates_frames_without_overwriting_gait_reference(self):
        h = np.arange(20 * 48, dtype=np.float32).reshape(20, 48)
        original = h.copy()
        action = np.linspace(-0.8, 0.8, 12, dtype=np.float32)
        advance_history(h, action)
        np.testing.assert_array_equal(h[1], original[0])
        np.testing.assert_array_equal(h[2:], original[1:-1])
        np.testing.assert_array_equal(h[0, :24], original[-1, :24])
        np.testing.assert_array_equal(h[0, 36:48], original[-1, 36:48])
        np.testing.assert_array_equal(h[0, LAST_ACTION], action)
        np.testing.assert_array_equal(h[1, GAIT_REFERENCE], original[0, GAIT_REFERENCE])
        self.assertFalse(np.array_equal(h[1, GAIT_REFERENCE], action))

    def test_invalid_shapes_are_rejected(self):
        with self.assertRaises(ValueError):
            advance_history(np.zeros((19, 48), np.float32), np.zeros(12, np.float32))
        with self.assertRaises(ValueError):
            advance_history(np.zeros((20, 48), np.float32), np.zeros(11, np.float32))
        with self.assertRaises(ValueError):
            advance_history(np.zeros((20, 48), np.float32),
                            np.full(12, np.nan, dtype=np.float32))


if __name__ == '__main__':
    unittest.main()
