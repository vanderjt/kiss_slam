r"""Extended Kalman Filter SLAM for 2D landmark mapping.

State layout
------------
The joint Gaussian state is organized as:

.. math::
   \mu = [x, y, \theta, l_{1x}, l_{1y}, \ldots]^T

with covariance :math:`\Sigma` of matching shape.

Prediction model
----------------
We use a unicycle control model with control input
:math:`u=[v,\omega]` and timestep ``dt``:

.. math::
   x' = x + v\,dt\cos\theta,
   \quad y' = y + v\,dt\sin\theta,
   \quad \theta' = \theta + \omega dt

The EKF prediction applies Jacobians :math:`F_x = \partial f/\partial x` and
:math:`F_u = \partial f/\partial u` with control noise :math:`Q_u`.

Update model
------------
For each landmark :math:`l_i=[l_x,l_y]`, expected measurement is:

.. math::
   h(x,l_i)=\begin{bmatrix}
     r\\
     \phi
   \end{bmatrix}
   =\begin{bmatrix}
     \sqrt{(l_x-x)^2+(l_y-y)^2}\\
     \operatorname{atan2}(l_y-y,l_x-x)-\theta
   \end{bmatrix}

with measurement covariance :math:`R` and wrapped bearing innovations.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from kiss_slam.data_association import KnownCorrespondenceAssociator
from kiss_slam.math_utils import mahalanobis_distance, wrap_angle
from kiss_slam.types import ControlInput, EKFSLAMConfig, Measurement, Pose2D


@dataclass(slots=True)
class EKFSLAM:
    """2D landmark-based EKF-SLAM estimator."""

    motion_model: object
    measurement_model: object
    assoc: object = field(default_factory=KnownCorrespondenceAssociator)
    Q: np.ndarray | None = None
    R: np.ndarray | None = None
    config: EKFSLAMConfig = field(default_factory=EKFSLAMConfig)

    _mu: np.ndarray = field(init=False, repr=False)
    _sigma: np.ndarray = field(init=False, repr=False)
    _landmark_index: dict[int, int] = field(init=False, repr=False)
    _nis_values: list[float] = field(init=False, repr=False)
    _last_innovation_vectors: list[tuple[np.ndarray, np.ndarray]] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self.Q = self.config.process_noise if self.Q is None else np.asarray(self.Q, dtype=float)
        self.R = self.config.measurement_noise if self.R is None else np.asarray(self.R, dtype=float)

        self._mu = self.config.initial_pose.as_array().astype(float)
        self._sigma = self.config.initial_pose_cov.astype(float).copy()
        self._landmark_index: dict[int, int] = {}

        self._nis_values: list[float] = []
        self._last_innovation_vectors: list[tuple[np.ndarray, np.ndarray]] = []

    def predict(self, u: ControlInput, dt: float, control_cov: np.ndarray | None = None) -> None:
        """Run EKF prediction.

        Parameters
        ----------
        u:
            Unicycle control input ``[v, w]``.
        dt:
            Integration interval in seconds.
        control_cov:
            Optional override for control noise covariance ``Q_u``.
        """
        q_u = self.Q if control_cov is None else np.asarray(control_cov, dtype=float)

        robot_state = self._mu[:3]
        robot_pred = self.motion_model.predict_state(robot_state, u, dt)
        fx, fu = self.motion_model.jacobians(robot_state, u, dt)

        n = self._mu.size
        g = np.eye(n, dtype=float)
        g[:3, :3] = fx

        self._mu[:3] = robot_pred
        self._mu[2] = wrap_angle(self._mu[2])

        self._sigma = g @ self._sigma @ g.T
        self._sigma[:3, :3] += fu @ q_u @ fu.T

    def update(self, measurements: list[Measurement], measurement_cov: np.ndarray | None = None) -> None:
        """Run EKF correction for all measurements in sequence."""
        self._last_innovation_vectors = []
        if not measurements:
            return

        r = self.R if measurement_cov is None else np.asarray(measurement_cov, dtype=float)

        associations = self._associate(measurements, r)
        for item in associations:
            measurement = item.measurement
            landmark_id = item.landmark_id

            if landmark_id is None:
                continue

            if landmark_id not in self._landmark_index:
                self._initialize_landmark(landmark_id=landmark_id, measurement=measurement, measurement_cov=r)
                continue

            self._update_landmark(landmark_id=landmark_id, measurement=measurement, measurement_cov=r)

    def step(self, u: ControlInput, dt: float, measurements: list[Measurement]) -> None:
        """Convenience wrapper for one predict-update cycle."""
        self.predict(u=u, dt=dt)
        self.update(measurements=measurements)

    def get_state(self) -> tuple[Pose2D, dict[int, np.ndarray]]:
        """Return robot pose and dictionary of landmark states."""
        pose = Pose2D(float(self._mu[0]), float(self._mu[1]), float(self._mu[2]))
        return pose, self.landmark_states()

    def get_covariance(self) -> np.ndarray:
        """Return current joint covariance matrix."""
        return self._sigma.copy()

    @property
    def state(self) -> np.ndarray:
        return self._mu

    @property
    def covariance(self) -> np.ndarray:
        return self._sigma

    @property
    def nis_values(self) -> list[float]:
        return self._nis_values

    def robot_pose(self) -> np.ndarray:
        """Return estimated robot pose as array."""
        return self._mu[:3].copy()

    def innovation_vectors(self) -> list[tuple[np.ndarray, np.ndarray]]:
        """Return list of innovation vectors represented as ``(start_xy, end_xy)``."""
        return [(start.copy(), end.copy()) for start, end in self._last_innovation_vectors]

    def landmark_states(self) -> dict[int, np.ndarray]:
        """Return map from landmark ID to landmark ``[x,y]`` estimate."""
        return {lid: self._mu[start : start + 2].copy() for lid, start in self._landmark_index.items()}

    def landmark_covariance(self, landmark_id: int) -> np.ndarray:
        """Return 2x2 covariance for one landmark."""
        start = self._landmark_index[landmark_id]
        return self._sigma[start : start + 2, start : start + 2].copy()

    def _associate(self, measurements: list[Measurement], measurement_cov: np.ndarray) -> list:
        known_ids = set(self._landmark_index)

        if hasattr(self.assoc, "associate"):
            kwargs = {
                "measurements": measurements,
                "known_landmark_ids": known_ids,
                "robot_state": self._mu[:3],
                "landmark_states": self.landmark_states(),
                "innovation_cov_fn": lambda landmark_id: self._innovation_covariance(landmark_id, measurement_cov),
                "measurement_model": self.measurement_model,
                "measurement_cov": measurement_cov,
            }
            return self.assoc.associate(**kwargs)
        raise TypeError("Associator must provide an associate(...) method")

    def _innovation_covariance(self, landmark_id: int, measurement_cov: np.ndarray) -> np.ndarray:
        start = self._landmark_index[landmark_id]
        robot_state = self._mu[:3]
        landmark_state = self._mu[start : start + 2]

        hr, hl = self.measurement_model.jacobians(robot_state, landmark_state)
        n = self._mu.size
        h = np.zeros((2, n), dtype=float)
        h[:, :3] = hr
        h[:, start : start + 2] = hl
        return h @ self._sigma @ h.T + measurement_cov

    def _initialize_landmark(self, landmark_id: int, measurement: Measurement, measurement_cov: np.ndarray) -> None:
        landmark_xy = self.measurement_model.initialize_landmark(self._mu[:3], measurement)
        old_n = self._mu.size

        mu_new = np.zeros(old_n + 2, dtype=float)
        mu_new[:old_n] = self._mu
        mu_new[old_n:] = landmark_xy

        sigma_new = np.zeros((old_n + 2, old_n + 2), dtype=float)
        sigma_new[:old_n, :old_n] = self._sigma

        # Landmark initialization covariance via first-order linearization.
        gx, gz = self.measurement_model.initialization_jacobians(self._mu[:3], measurement)
        sigma_xr = self._sigma[:, :3]  # cross-covariance between full state and robot state.
        sigma_xl = sigma_xr @ gx.T
        sigma_ll = gx @ self._sigma[:3, :3] @ gx.T + gz @ measurement_cov @ gz.T

        sigma_new[:old_n, old_n:] = sigma_xl
        sigma_new[old_n:, :old_n] = sigma_xl.T
        sigma_new[old_n:, old_n:] = sigma_ll

        if not np.all(np.isfinite(sigma_new[old_n:, old_n:])):
            sigma_new[old_n:, old_n:] = self.config.initial_landmark_cov

        self._mu = mu_new
        self._sigma = sigma_new
        self._landmark_index[landmark_id] = old_n

    def _update_landmark(self, landmark_id: int, measurement: Measurement, measurement_cov: np.ndarray) -> None:
        start = self._landmark_index[landmark_id]
        robot_state = self._mu[:3]
        landmark_state = self._mu[start : start + 2]

        z = measurement.as_array()
        z_hat = self.measurement_model.predict(robot_state, landmark_state)
        innovation = z - z_hat
        innovation[1] = wrap_angle(innovation[1])
        self._last_innovation_vectors.append(self._innovation_segment(robot_state, z_hat, z))

        hr, hl = self.measurement_model.jacobians(robot_state, landmark_state)
        n = self._mu.size
        h = np.zeros((2, n), dtype=float)
        h[:, :3] = hr
        h[:, start : start + 2] = hl

        s = h @ self._sigma @ h.T + measurement_cov
        k = self._sigma @ h.T @ np.linalg.inv(s)

        self._mu = self._mu + k @ innovation
        self._mu[2] = wrap_angle(self._mu[2])

        i_n = np.eye(n, dtype=float)
        if self.config.use_joseph_form:
            residual = i_n - k @ h
            self._sigma = residual @ self._sigma @ residual.T + k @ measurement_cov @ k.T
        else:
            self._sigma = (i_n - k @ h) @ self._sigma

        self._nis_values.append(mahalanobis_distance(innovation, s))

    @staticmethod
    def _innovation_segment(robot_state: np.ndarray, predicted_z: np.ndarray, measured_z: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Map a measurement innovation to world-frame segment endpoints."""
        theta = float(robot_state[2])

        pred_angle = theta + float(predicted_z[1])
        meas_angle = theta + float(measured_z[1])
        robot_xy = robot_state[:2]

        start = robot_xy + np.array([predicted_z[0] * np.cos(pred_angle), predicted_z[0] * np.sin(pred_angle)], dtype=float)
        end = robot_xy + np.array([measured_z[0] * np.cos(meas_angle), measured_z[0] * np.sin(meas_angle)], dtype=float)
        return start, end
