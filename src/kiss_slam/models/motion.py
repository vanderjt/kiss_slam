r"""Motion model interfaces and baseline implementations.

This module provides a unicycle motion model for state
:math:`x=[x, y, \theta]^T` and control input :math:`u=[v,\omega]^T`.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from kiss_slam.math_utils import wrap_angle
from kiss_slam.types import ControlInput


def unicycle_predict_state(pose: np.ndarray, control: ControlInput, dt: float) -> np.ndarray:
    """Predict the next pose using first-order unicycle integration."""
    x, y, yaw = pose
    v, w = control.v, control.w
    x_next = x + v * dt * np.cos(yaw)
    y_next = y + v * dt * np.sin(yaw)
    yaw_next = wrap_angle(yaw + w * dt)
    return np.array([x_next, y_next, yaw_next], dtype=float)


def unicycle_jacobians(pose: np.ndarray, control: ControlInput, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """Return analytic Jacobians ``Fx=df/dx`` and ``Fu=df/du``."""
    _, _, yaw = pose
    v = control.v
    fx = np.eye(3, dtype=float)
    fx[0, 2] = -v * dt * np.sin(yaw)
    fx[1, 2] = v * dt * np.cos(yaw)

    fu = np.zeros((3, 2), dtype=float)
    fu[0, 0] = dt * np.cos(yaw)
    fu[1, 0] = dt * np.sin(yaw)
    fu[2, 1] = dt
    return fx, fu


@dataclass(slots=True)
class DifferentialDriveMotionModel:
    """Unicycle-style motion model adapter with a class-friendly API."""

    def predict_state(self, pose: np.ndarray, control: ControlInput, dt: float) -> np.ndarray:
        """Predict next robot pose."""
        return unicycle_predict_state(pose=pose, control=control, dt=dt)

    def jacobians(self, pose: np.ndarray, control: ControlInput, dt: float) -> tuple[np.ndarray, np.ndarray]:
        """Return Jacobians wrt state and control."""
        return unicycle_jacobians(pose=pose, control=control, dt=dt)
