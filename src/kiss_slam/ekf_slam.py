"""Baseline EKF-SLAM implementation for 2D landmark mapping."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from kiss_slam.data_association import KnownCorrespondenceAssociator
from kiss_slam.math_utils import wrap_angle
from kiss_slam.types import ControlInput, Measurement, Pose2D


@dataclass(slots=True)
class EKFSLAM:
    """EKF-SLAM estimator with explicit state layout.

    Joint state vector is:
    `[x, y, yaw, l0x, l0y, l1x, l1y, ...]`.
    """

    motion_model: object
    measurement_model: object
    associator: object = field(default_factory=KnownCorrespondenceAssociator)
    initial_pose_cov: np.ndarray = field(default_factory=lambda: np.diag([1e-3, 1e-3, 1e-3]))
    initial_landmark_cov: np.ndarray = field(default_factory=lambda: np.diag([10.0, 10.0]))

    def __post_init__(self) -> None:
        self._mu = np.zeros(3, dtype=float)
        self._sigma = self.initial_pose_cov.astype(float).copy()
        self._landmark_index: dict[int, int] = {}
        self._nis_values: list[float] = []

    @property
    def state(self) -> np.ndarray:
        """Current joint mean vector."""
        return self._mu

    @property
    def covariance(self) -> np.ndarray:
        """Current joint covariance matrix."""
        return self._sigma

    @property
    def nis_values(self) -> list[float]:
        """Collected normalized innovation squared values."""
        return self._nis_values

    def set_initial_pose(self, pose: Pose2D) -> None:
        """Set initial robot mean pose."""
        self._mu[:3] = pose.as_array()

    def predict(self, control: ControlInput, control_cov: np.ndarray, dt: float) -> None:
        """EKF predict step for robot pose and joint covariance."""
        robot = self._mu[:3]
        robot_next = self.motion_model.predict_state(robot, control, dt)
        fx, fu = self.motion_model.jacobians(robot, control, dt)

        n = self._mu.size
        g = np.eye(n, dtype=float)
        g[:3, :3] = fx
        r = fu @ control_cov @ fu.T

        self._mu[:3] = robot_next
        self._mu[2] = wrap_angle(self._mu[2])
        self._sigma = g @ self._sigma @ g.T
        self._sigma[:3, :3] += r

    def update(self, measurements: list[Measurement], measurement_cov: np.ndarray) -> None:
        """EKF measurement updates with association and on-the-fly landmark creation."""
        known_ids = set(self._landmark_index.keys())
        associated = self.associator.associate(measurements=measurements, known_landmark_ids=known_ids)
        for item in associated:
            landmark_id = item.landmark_id
            measurement = item.measurement

            if landmark_id is None and measurement.landmark_id is not None:
                landmark_id = measurement.landmark_id
            if landmark_id is None:
                # TODO: support anonymous landmark hypotheses.
                continue

            if landmark_id not in self._landmark_index:
                self._add_landmark(landmark_id, measurement)
                continue

            self._update_existing_landmark(landmark_id, measurement, measurement_cov)

    def robot_pose(self) -> np.ndarray:
        """Return robot state `[x, y, yaw]`."""
        return self._mu[:3].copy()

    def landmark_states(self) -> dict[int, np.ndarray]:
        """Return map of landmark id to `[x, y]` state estimate."""
        states: dict[int, np.ndarray] = {}
        for landmark_id, start in self._landmark_index.items():
            states[landmark_id] = self._mu[start : start + 2].copy()
        return states

    def _add_landmark(self, landmark_id: int, measurement: Measurement) -> None:
        landmark_xy = self.measurement_model.initialize_landmark(self._mu[:3], measurement)
        old_n = self._mu.size

        mu_new = np.zeros(old_n + 2, dtype=float)
        mu_new[:old_n] = self._mu
        mu_new[old_n:] = landmark_xy

        sigma_new = np.zeros((old_n + 2, old_n + 2), dtype=float)
        sigma_new[:old_n, :old_n] = self._sigma
        sigma_new[old_n:, old_n:] = self.initial_landmark_cov

        self._mu = mu_new
        self._sigma = sigma_new
        self._landmark_index[landmark_id] = old_n

    def _update_existing_landmark(self, landmark_id: int, measurement: Measurement, measurement_cov: np.ndarray) -> None:
        start = self._landmark_index[landmark_id]
        robot = self._mu[:3]
        landmark = self._mu[start : start + 2]

        z = measurement.as_array()
        z_hat = self.measurement_model.predict(robot, landmark)
        innovation = z - z_hat
        innovation[1] = wrap_angle(innovation[1])

        hr, hl = self.measurement_model.jacobians(robot, landmark)
        n = self._mu.size
        h = np.zeros((2, n), dtype=float)
        h[:, :3] = hr
        h[:, start : start + 2] = hl

        s = h @ self._sigma @ h.T + measurement_cov
        k = self._sigma @ h.T @ np.linalg.inv(s)

        self._mu = self._mu + k @ innovation
        self._mu[2] = wrap_angle(self._mu[2])
        identity = np.eye(n, dtype=float)
        self._sigma = (identity - k @ h) @ self._sigma

        nis = float(innovation.T @ np.linalg.inv(s) @ innovation)
        self._nis_values.append(nis)
