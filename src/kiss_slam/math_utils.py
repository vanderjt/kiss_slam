"""Math helpers used by SLAM, simulation, and visualization."""

from __future__ import annotations

import numpy as np
from matplotlib.patches import Ellipse


def wrap_angle(angle_rad: float) -> float:
    """Wrap angle to `[-pi, pi)`.

    Parameters
    ----------
    angle_rad:
        Input angle in radians.
    """
    return (angle_rad + np.pi) % (2.0 * np.pi) - np.pi


def wrap_angle_array(angles_rad: np.ndarray) -> np.ndarray:
    """Vectorized angle wrapping to `[-pi, pi)`."""
    return (angles_rad + np.pi) % (2.0 * np.pi) - np.pi


def numerical_jacobian(
    func,
    x: np.ndarray,
    eps: float = 1e-6,
) -> np.ndarray:
    """Compute Jacobian of `func(x)` using central finite differences.

    This helper is intended for tests and model validation.
    """
    y0 = np.asarray(func(x), dtype=float)
    jac = np.zeros((y0.size, x.size), dtype=float)
    for index in range(x.size):
        dx = np.zeros_like(x)
        dx[index] = eps
        y_plus = np.asarray(func(x + dx), dtype=float)
        y_minus = np.asarray(func(x - dx), dtype=float)
        jac[:, index] = (y_plus - y_minus) / (2.0 * eps)
    return jac


def covariance_ellipse(
    mean_xy: np.ndarray,
    covariance_xy: np.ndarray,
    n_std: float = 2.0,
    **kwargs,
) -> Ellipse:
    """Create a Matplotlib ellipse patch for 2D covariance.

    Parameters
    ----------
    mean_xy:
        2D center point.
    covariance_xy:
        2x2 covariance matrix.
    n_std:
        Number of standard deviations for ellipse radius.
    """
    eigenvalues, eigenvectors = np.linalg.eigh(covariance_xy)
    order = np.argsort(eigenvalues)[::-1]
    eigenvalues = eigenvalues[order]
    eigenvectors = eigenvectors[:, order]

    width, height = 2.0 * n_std * np.sqrt(np.maximum(eigenvalues, 0.0))
    angle_deg = np.degrees(np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0]))
    return Ellipse(xy=mean_xy, width=width, height=height, angle=angle_deg, fill=False, **kwargs)
