"""Common typed dataclasses used across kiss_slam."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable

import numpy as np


@dataclass(slots=True)
class Pose2D:
    """2D robot pose in world frame."""

    x: float
    y: float
    yaw: float

    def as_array(self) -> np.ndarray:
        """Return `[x, y, yaw]` as a numpy array."""
        return np.array([self.x, self.y, self.yaw], dtype=float)


@dataclass(slots=True)
class Landmark2D:
    """2D landmark state."""

    landmark_id: int
    x: float
    y: float

    def as_array(self) -> np.ndarray:
        """Return `[x, y]` as a numpy array."""
        return np.array([self.x, self.y], dtype=float)


@dataclass(slots=True)
class ControlInput:
    """Robot control input for unicycle/differential drive abstraction."""

    v: float
    w: float

    def as_array(self) -> np.ndarray:
        """Return `[v, w]` as a numpy array."""
        return np.array([self.v, self.w], dtype=float)


@dataclass(slots=True)
class Measurement:
    """Range-bearing measurement to a landmark.

    Bearing is in radians, relative to robot heading.
    """

    landmark_id: int | None
    range_m: float
    bearing_rad: float

    def as_array(self) -> np.ndarray:
        """Return `[range, bearing]` as a numpy array."""
        return np.array([self.range_m, self.bearing_rad], dtype=float)


@dataclass(slots=True)
class SimulationStep:
    """Data generated for one simulation step."""

    true_pose: Pose2D
    control: ControlInput
    measurements: list[Measurement] = field(default_factory=list)


@dataclass(slots=True)
class Trajectory:
    """A convenience container for a sequence of poses."""

    poses: list[Pose2D]

    @classmethod
    def from_iterable(cls, poses: Iterable[Pose2D]) -> "Trajectory":
        """Build from an iterable of `Pose2D`."""
        return cls(poses=list(poses))
