"""Match the post-inference history rotation in Stanford's pinned C++ controller."""
import numpy as np

OBSERVATION_HISTORY = 20
FRAME_SIZE = 48
LAST_ACTION = slice(24, 36)
GAIT_REFERENCE = slice(36, 48)


def advance_history(history: np.ndarray, next_last_action: np.ndarray) -> None:
    """After inference, rotate newest-first history; write next frame's action.

    Mirrors pinned Stanford neural_controller.cpp: std::rotate(rbegin, rbegin+48,
    rend), then writes the faded policy output at obs_layout_.last_action.
    """
    if history.shape != (OBSERVATION_HISTORY, FRAME_SIZE):
        raise ValueError('expected 20x48 observation history')
    action = np.asarray(next_last_action, dtype=np.float32)
    if action.shape != (12,) or not np.isfinite(action).all():
        raise ValueError('expected 12 finite policy actions')
    oldest = history[-1].copy()
    history[1:] = history[:-1].copy()
    history[0] = oldest
    history[0, LAST_ACTION] = action
