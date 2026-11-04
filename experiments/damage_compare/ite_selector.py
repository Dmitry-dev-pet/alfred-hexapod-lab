"""A small independent, numpy-only IT&E-style Bayesian candidate selector.

This reproduces the *principle* described by Cully et al. and resibots/pyite:
keep a prior fitness map from an undamaged robot, then use a GP over the
observed (damaged - predicted) residual and an acquisition function.

NOT a port of resibots/pyite, NOT MAP-Elites, NOT the 2015 published experiment.
We use only six handcrafted candidate controllers at this stage. All training
measurements must come from an intact robot; no oracle for damaged outcomes.
"""
from __future__ import annotations

import numpy as np


def matern52(a: np.ndarray, b: np.ndarray, rho: float = 0.4) -> np.ndarray:
    """Stationary Matérn 5/2 kernel, unit prior variance, normalized descriptors."""
    a, b = np.atleast_2d(np.asarray(a, float)), np.atleast_2d(np.asarray(b, float))
    if a.shape[1] != b.shape[1] or rho <= 0:
        raise ValueError('Descriptor dimensions or rho')
    d = np.sqrt(np.maximum(0., ((a[:, None, :] - b[None, :, :]) ** 2).sum(axis=-1)))
    t = np.sqrt(5.) * d / rho
    return (1. + t + t * t / 3.) * np.exp(-t)


def posterior_residual(descriptors: np.ndarray, observations: list[tuple[int, float]],
                       rho: float = 0.4, noise: float = 0.04) -> tuple[np.ndarray, np.ndarray]:
    """GP posterior residual mean and standard deviation at every candidate.

    Observations contain (repertoire_index, observed_score_minus_prior_score).
    Noise is the *variance* of measurement noise, not its standard deviation.
    """
    x = np.asarray(descriptors, float)
    if x.ndim != 2 or len(x) == 0 or not np.isfinite(x).all():
        raise ValueError('Finite 2D descriptors required')
    if noise <= 0:
        raise ValueError('Noise variance must be positive')
    if not observations:
        return np.zeros(len(x)), np.ones(len(x))
    ids = np.array([int(p[0]) for p in observations], dtype=int)
    y = np.array([float(p[1]) for p in observations], float)
    if len(set(ids.tolist())) != len(ids) or ids.min() < 0 or ids.max() >= len(x):
        raise ValueError('Duplicate/out-of-bounds tested index')
    if not np.isfinite(y).all():
        raise ValueError('Nonfinite observed scores')
    xt = x[ids]
    k = matern52(xt, xt, rho=rho) + noise * np.eye(len(ids))
    cross = matern52(x, xt, rho=rho)
    chol = np.linalg.cholesky(k)
    mean = cross @ np.linalg.solve(chol.T, np.linalg.solve(chol, y))
    v = np.linalg.solve(chol, cross.T)
    var = np.maximum(0., 1. - np.sum(v * v, axis=0))
    return mean, np.sqrt(var)


def select_next(descriptors: np.ndarray, intact_scores: np.ndarray,
                tested: list[tuple[int, float]], kappa: float = 0.30,
                rho: float = 0.40, noise: float = 0.04) -> tuple[int, dict]:
    """Select an untested candidate by prior fitness + posterior residual + UCB.

    Follows the pyite decision structure; makes no claim of parameter equivalence.
    ``tested`` records *damaged* performance; only indices are visible to model
    before a trial, preventing damaged-map leakage.
    """
    x = np.asarray(descriptors, float)
    scores = np.asarray(intact_scores, float)
    if scores.shape != (len(x),) or not np.isfinite(scores).all():
        raise ValueError('Intact score count mismatch or nonfinite')
    if kappa < 0:
        raise ValueError('Kappa must be >=0')
    observations = [(i, actual - float(scores[i])) for i, actual in tested]
    mu, std = posterior_residual(x, observations, rho, noise)
    acq = scores + mu + kappa * std
    for i, _ in tested:
        acq[i] = -np.inf
    if not np.isfinite(acq).any():
        raise StopIteration('All candidates have been tested')
    index = int(np.argmax(acq))
    return index, {
        'acquisition': [float(round(v, 6)) if np.isfinite(v) else None for v in acq],
        'posterior_delta_mean': mu.round(6).tolist(),
        'posterior_std': std.round(6).tolist(),
        'selected_index': index,
        'tested_indices': [int(i) for i, _ in tested],
    }