"""Motion model interfaces and baseline implementations."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from kiss_slam.math_utils import wrap_angle
from kiss_slam.types import ControlInput


@dataclass(slots=True)
class DifferentialDriveMotionModel:
    """Unicycle-style motion model for 2D robot state.

    State convention: `[x, y, yaw]`.
    Control convention: `v` (m/s), `w` (rad/s).
    """

    def predict_state(self, pose: np.ndarray, control: ControlInput, dt: float) -> np.ndarray:
        """Predict next robot pose with first-order integration."""
        x, y, yaw = pose
        v, w = control.v, control.w
        x_next = x + v * dt * np.cos(yaw)
        y_next = y + v * dt * np.sin(yaw)
        yaw_next = wrap_angle(yaw + w * dt)
        return np.array([x_next, y_next, yaw_next], dtype=float)

    def jacobians(self, pose: np.ndarray, control: ControlInput, dt: float) -> tuple[np.ndarray, np.ndarray]:
        """Return Jacobians `(Fx, Fu)` for the motion model.

        - `Fx = d f / d pose`
        - `Fu = d f / d [v, w]`
        """
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
