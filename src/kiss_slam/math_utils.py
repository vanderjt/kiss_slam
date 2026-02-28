"""Math helpers used by SLAM, simulation, and visualization."""

from __future__ import annotations

import numpy as np
from matplotlib.patches import Ellipse


def wrap_angle(angle_rad: float) -> float:
    """Wrap angle to ``[-pi, pi)``."""
    return (angle_rad + np.pi) % (2.0 * np.pi) - np.pi


def wrap_angle_array(angles_rad: np.ndarray) -> np.ndarray:
    """Vectorized angle wrapping to ``[-pi, pi)``."""
    return (angles_rad + np.pi) % (2.0 * np.pi) - np.pi


def mahalanobis_distance(error: np.ndarray, covariance: np.ndarray) -> float:
    """Compute squared Mahalanobis distance ``e.T @ S^-1 @ e``."""
    return float(error.T @ np.linalg.solve(covariance, error))


def numerical_jacobian(func, x: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """Compute Jacobian of ``func(x)`` using central finite differences."""
    y0 = np.asarray(func(x), dtype=float)
    jac = np.zeros((y0.size, x.size), dtype=float)
    for index in range(x.size):
        dx = np.zeros_like(x)
        dx[index] = eps
        y_plus = np.asarray(func(x + dx), dtype=float)
        y_minus = np.asarray(func(x - dx), dtype=float)
        jac[:, index] = (y_plus - y_minus) / (2.0 * eps)
    return jac


def covariance_ellipse(mean_xy: np.ndarray, covariance_xy: np.ndarray, n_std: float = 2.0, **kwargs) -> Ellipse:
    """Create a Matplotlib ellipse patch for a 2D covariance matrix."""
    center_xy, width, height, angle_deg = covariance_ellipse_params(mean_xy, covariance_xy, n_std=n_std)
    return Ellipse(xy=center_xy, width=width, height=height, angle=angle_deg, fill=False, **kwargs)


def covariance_ellipse_params(
    mean_xy: np.ndarray,
    covariance_xy: np.ndarray,
    n_std: float = 2.0,
) -> tuple[np.ndarray, float, float, float]:
    """Return covariance ellipse geometry for efficient artist updates.

    Returns
    -------
    tuple[np.ndarray, float, float, float]
        ``(center_xy, width, height, angle_deg)`` for Matplotlib-style ellipses.
    """
    eigenvalues, eigenvectors = np.linalg.eigh(covariance_xy)
    order = np.argsort(eigenvalues)[::-1]
    eigenvalues = eigenvalues[order]
    eigenvectors = eigenvectors[:, order]

    width, height = 2.0 * n_std * np.sqrt(np.maximum(eigenvalues, 0.0))
    angle_deg = np.degrees(np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0]))
    return np.asarray(mean_xy, dtype=float), float(width), float(height), float(angle_deg)
