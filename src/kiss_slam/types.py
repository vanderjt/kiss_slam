r"""Common typed dataclasses used across kiss_slam.

The module intentionally keeps containers lightweight and dependency-free so the
core filtering logic can remain easy to unit test.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable

import numpy as np


@dataclass(slots=True)
class Pose2D:
    """2D robot pose in world frame.

    Attributes
    ----------
    x, y:
        Position in meters.
    yaw:
        Heading in radians.
    """

    x: float
    y: float
    yaw: float

    def as_array(self) -> np.ndarray:
        """Return ``[x, y, yaw]`` as a numpy array."""
        return np.array([self.x, self.y, self.yaw], dtype=float)


@dataclass(slots=True)
class Landmark2D:
    """2D landmark state with a stable integer identifier."""

    landmark_id: int
    x: float
    y: float

    def as_array(self) -> np.ndarray:
        """Return ``[x, y]`` as a numpy array."""
        return np.array([self.x, self.y], dtype=float)


@dataclass(slots=True)
class ControlInput:
    """Unicycle control input.

    We use the control vector :math:`u=[v, \\omega]` where ``v`` is linear
    velocity in m/s and ``w`` is yaw rate in rad/s.
    """

    v: float
    w: float

    def as_array(self) -> np.ndarray:
        """Return ``[v, w]`` as a numpy array."""
        return np.array([self.v, self.w], dtype=float)


@dataclass(slots=True)
class Measurement:
    """Range-bearing observation to a landmark.

    Attributes
    ----------
    landmark_id:
        Simulator-provided landmark correspondence. May be ``None`` for unknown
        correspondence mode.
    range_m:
        Measured range in meters.
    bearing_rad:
        Relative bearing wrt robot heading in radians.
    """

    landmark_id: int | None
    range_m: float
    bearing_rad: float

    def as_array(self) -> np.ndarray:
        """Return ``[range, bearing]`` as a numpy array."""
        return np.array([self.range_m, self.bearing_rad], dtype=float)


@dataclass(slots=True)
class EKFSLAMConfig:
    """Tunable EKF-SLAM parameters.

    Parameters
    ----------
    process_noise:
        Covariance on control input :math:`Q_u` for ``[v, w]``.
    measurement_noise:
        Covariance on range-bearing measurements :math:`R`.
    initial_pose:
        Initial robot pose estimate in the world frame.
    initial_pose_cov:
        Covariance for initial pose estimate.
    initial_landmark_cov:
        Fallback covariance used only when Jacobian-based initialization fails.
    mahalanobis_gate:
        Chi-square-like threshold for nearest-neighbor gating.
    use_joseph_form:
        If ``True``, update covariance using Joseph stabilized form.
    """

    process_noise: np.ndarray = field(default_factory=lambda: np.diag([0.05**2, 0.03**2]))
    measurement_noise: np.ndarray = field(default_factory=lambda: np.diag([0.2**2, np.deg2rad(3.0) ** 2]))
    initial_pose: Pose2D = field(default_factory=lambda: Pose2D(0.0, 0.0, 0.0))
    initial_pose_cov: np.ndarray = field(default_factory=lambda: np.diag([1e-3, 1e-3, 1e-3]))
    initial_landmark_cov: np.ndarray = field(default_factory=lambda: np.diag([10.0, 10.0]))
    mahalanobis_gate: float = 5.99
    use_joseph_form: bool = True


@dataclass(slots=True)
class SimulationStep:
    """Data generated for one simulator step."""

    true_pose: Pose2D
    control: ControlInput
    measurements: list[Measurement] = field(default_factory=list)


@dataclass(slots=True)
class Trajectory:
    """Convenience container for a sequence of ``Pose2D`` objects."""

    poses: list[Pose2D]

    @classmethod
    def from_iterable(cls, poses: Iterable[Pose2D]) -> "Trajectory":
        """Build from any iterable of poses."""
        return cls(poses=list(poses))
