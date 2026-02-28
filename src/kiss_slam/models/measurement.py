r"""Range-bearing landmark measurement model for 2D EKF-SLAM."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from kiss_slam.math_utils import wrap_angle
from kiss_slam.types import Measurement


def expected_range_bearing(robot_state: np.ndarray, landmark_state: np.ndarray, min_range: float = 1e-9) -> np.ndarray:
    """Compute expected measurement ``h(x)=[range, bearing]``."""
    dx = landmark_state[0] - robot_state[0]
    dy = landmark_state[1] - robot_state[1]
    rng = max(np.hypot(dx, dy), min_range)
    bearing = wrap_angle(np.arctan2(dy, dx) - robot_state[2])
    return np.array([rng, bearing], dtype=float)


def range_bearing_jacobians(
    robot_state: np.ndarray,
    landmark_state: np.ndarray,
    min_range: float = 1e-9,
) -> tuple[np.ndarray, np.ndarray]:
    """Return Jacobians ``Hr = dh/dx_r`` and ``Hl = dh/dx_l``."""
    dx = landmark_state[0] - robot_state[0]
    dy = landmark_state[1] - robot_state[1]
    q = max(dx * dx + dy * dy, min_range**2)
    rng = np.sqrt(q)

    hr = np.array([[-dx / rng, -dy / rng, 0.0], [dy / q, -dx / q, -1.0]], dtype=float)
    hl = np.array([[dx / rng, dy / rng], [-dy / q, dx / q]], dtype=float)
    return hr, hl


def initialize_landmark_from_measurement(robot_state: np.ndarray, measurement: Measurement) -> np.ndarray:
    """Triangulate landmark state from robot pose and range-bearing reading."""
    theta = robot_state[2] + measurement.bearing_rad
    lx = robot_state[0] + measurement.range_m * np.cos(theta)
    ly = robot_state[1] + measurement.range_m * np.sin(theta)
    return np.array([lx, ly], dtype=float)


def init_landmark_jacobians(robot_state: np.ndarray, measurement: Measurement) -> tuple[np.ndarray, np.ndarray]:
    """Jacobian of landmark init wrt robot pose and measurement.

    For initialization map function :math:`l=g(x_r,z)`:

    .. math::
        l_x = x + r\\cos(\\theta+b),\\quad l_y = y + r\\sin(\\theta+b)
    """
    theta = robot_state[2] + measurement.bearing_rad
    rng = measurement.range_m
    gx = np.array([[1.0, 0.0, -rng * np.sin(theta)], [0.0, 1.0, rng * np.cos(theta)]], dtype=float)
    gz = np.array([[np.cos(theta), -rng * np.sin(theta)], [np.sin(theta), rng * np.cos(theta)]], dtype=float)
    return gx, gz


@dataclass(slots=True)
class RangeBearingMeasurementModel:
    """Measurement model adapter class for EKF-SLAM."""

    min_range: float = 1e-9

    def predict(self, robot_state: np.ndarray, landmark_state: np.ndarray) -> np.ndarray:
        return expected_range_bearing(robot_state, landmark_state, min_range=self.min_range)

    def jacobians(self, robot_state: np.ndarray, landmark_state: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        return range_bearing_jacobians(robot_state, landmark_state, min_range=self.min_range)

    def initialize_landmark(self, robot_state: np.ndarray, measurement: Measurement) -> np.ndarray:
        return initialize_landmark_from_measurement(robot_state, measurement)

    def initialization_jacobians(self, robot_state: np.ndarray, measurement: Measurement) -> tuple[np.ndarray, np.ndarray]:
        return init_landmark_jacobians(robot_state, measurement)
