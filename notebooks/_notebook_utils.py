"""Lightweight helpers shared by teaching notebooks.

These functions intentionally stay small and dependency-light so notebooks remain
portable in a standard Anaconda environment.
"""

from __future__ import annotations

from typing import Iterable

import numpy as np
from matplotlib.patches import Ellipse


def set_seed(seed: int) -> int:
    """Set NumPy's global random seed and return the seed.

    Parameters
    ----------
    seed:
        Integer random seed for reproducible notebook runs.
    """
    np.random.seed(seed)
    return seed


def wrap_angle(theta: float | np.ndarray) -> float | np.ndarray:
    """Wrap angle(s) to [-pi, pi).

    Works with scalars and NumPy arrays.
    """
    return (theta + np.pi) % (2.0 * np.pi) - np.pi


def assert_shape(name: str, arr: np.ndarray, expected: tuple[int, ...]) -> None:
    """Raise ValueError if ``arr.shape`` does not match ``expected``."""
    if arr.shape != expected:
        raise ValueError(f"{name} has shape {arr.shape}, expected {expected}")


def print_matrix(name: str, M: np.ndarray, fmt: str = "{: .3f}") -> None:
    """Pretty-print a 2D matrix with a configurable number format."""
    if M.ndim != 2:
        raise ValueError(f"{name} must be 2D, got ndim={M.ndim}")

    print(f"{name} (shape={M.shape}):")
    for row in M:
        print("  " + " ".join(fmt.format(v) for v in row))


def plot_cov_ellipse(
    center: Iterable[float],
    cov: np.ndarray,
    ax=None,
    n_std: float = 2.0,
    **kwargs,
):
    """Plot a 2D covariance ellipse and return ``(ax, ellipse)``.

    Parameters
    ----------
    center:
        2D center ``(x, y)``.
    cov:
        2x2 covariance matrix.
    ax:
        Optional matplotlib axis. Uses current axis if ``None``.
    n_std:
        Radius scale in standard deviations.
    **kwargs:
        Forwarded to ``matplotlib.patches.Ellipse``.
    """
    center = np.asarray(center, dtype=float).reshape(2)
    cov = np.asarray(cov, dtype=float)
    assert_shape("cov", cov, (2, 2))

    eigvals, eigvecs = np.linalg.eigh(cov)
    eigvals = np.clip(eigvals, 0.0, None)
    order = np.argsort(eigvals)[::-1]
    eigvals = eigvals[order]
    eigvecs = eigvecs[:, order]

    width, height = 2.0 * n_std * np.sqrt(eigvals)
    angle_deg = np.degrees(np.arctan2(eigvecs[1, 0], eigvecs[0, 0]))

    if ax is None:
        import matplotlib.pyplot as plt

        ax = plt.gca()

    ellipse_defaults = {"fill": False, "linewidth": 2.0}
    ellipse_defaults.update(kwargs)
    ellipse = Ellipse(xy=center, width=width, height=height, angle=angle_deg, **ellipse_defaults)
    ax.add_patch(ellipse)
    return ax, ellipse
