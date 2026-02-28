"""Measurement model interfaces and baseline implementations."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from kiss_slam.math_utils import wrap_angle
from kiss_slam.types import Measurement


@dataclass(slots=True)
class RangeBearingMeasurementModel:
    """Range-bearing measurement model for 2D landmarks.

    Robot state: `[x, y, yaw]`.
    Landmark state: `[lx, ly]`.
    Measurement: `[range, bearing]`.
    """

    min_range: float = 1e-9

    def predict(self, robot_state: np.ndarray, landmark_state: np.ndarray) -> np.ndarray:
        """Predict ideal range-bearing measurement."""
        dx = landmark_state[0] - robot_state[0]
        dy = landmark_state[1] - robot_state[1]
        rng = np.hypot(dx, dy)
        rng = max(rng, self.min_range)
        bearing = wrap_angle(np.arctan2(dy, dx) - robot_state[2])
        return np.array([rng, bearing], dtype=float)

    def jacobians(self, robot_state: np.ndarray, landmark_state: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Return Jacobians `(Hr, Hl)`.

        - `Hr = d h / d robot_state`
        - `Hl = d h / d landmark_state`
        """
        dx = landmark_state[0] - robot_state[0]
        dy = landmark_state[1] - robot_state[1]
        q = max(dx * dx + dy * dy, self.min_range**2)
        rng = np.sqrt(q)

        hr = np.array(
            [
                [-dx / rng, -dy / rng, 0.0],
                [dy / q, -dx / q, -1.0],
            ],
            dtype=float,
        )
        hl = np.array(
            [
                [dx / rng, dy / rng],
                [-dy / q, dx / q],
            ],
            dtype=float,
        )
        return hr, hl

    def initialize_landmark(self, robot_state: np.ndarray, measurement: Measurement) -> np.ndarray:
        """Triangulate landmark position from robot pose + range-bearing measurement."""
        theta = robot_state[2] + measurement.bearing_rad
        lx = robot_state[0] + measurement.range_m * np.cos(theta)
        ly = robot_state[1] + measurement.range_m * np.sin(theta)
        return np.array([lx, ly], dtype=float)
