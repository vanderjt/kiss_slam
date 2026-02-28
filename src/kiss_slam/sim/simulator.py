"""2D simulator for generating deterministic landmark-SLAM datasets.

The simulator provides:
- configurable landmark visibility (range + field-of-view)
- multiple trajectory generators
- noisy + biased odometry
- noisy range/bearing observations with dropout

The API is intentionally simple and extension-friendly so future sensors can be
added without changing callers.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

import numpy as np

from kiss_slam.math_utils import wrap_angle
from kiss_slam.types import ControlInput, Measurement, Pose2D, SimulationStep

TrajectoryMode = Literal["rectangle", "figure_eight", "random_smooth"]


@dataclass(slots=True)
class SimConfig:
    """Configuration for :class:`Simulator`.

    Attributes
    ----------
    dt:
        Fixed simulation timestep in seconds.
    sensor_range:
        Max landmark sensing distance in meters.
    fov_rad:
        Sensor field-of-view in radians.
    trajectory_mode:
        One of ``rectangle``, ``figure_eight``, ``random_smooth``.
    world_limits:
        ``((xmin, xmax), (ymin, ymax))`` used by random trajectory mode.
    nominal_v:
        Nominal linear speed for analytic trajectories.
    nominal_w:
        Nominal yaw-rate scale for analytic trajectories.
    control_noise_std:
        Standard deviation for sampled odometry noise ``[v, w]``.
    control_bias:
        Constant additive odometry bias ``[v, w]``.
    measurement_noise_std:
        Standard deviation ``[range, bearing]`` for sensor noise.
    measurement_dropout_prob:
        Probability of dropping an otherwise valid landmark observation.
    random_control_bounds:
        Bounds for random smooth trajectory ``(v_min, v_max, w_abs_max)``.
    """

    dt: float = 0.1
    sensor_range: float = 12.0
    fov_rad: float = np.pi
    trajectory_mode: TrajectoryMode = "figure_eight"
    world_limits: tuple[tuple[float, float], tuple[float, float]] = ((-20.0, 20.0), (-20.0, 20.0))
    nominal_v: float = 1.0
    nominal_w: float = 0.6
    control_noise_std: tuple[float, float] = (0.05, 0.03)
    control_bias: tuple[float, float] = (0.0, 0.0)
    measurement_noise_std: tuple[float, float] = (0.20, np.deg2rad(3.0))
    measurement_dropout_prob: float = 0.0
    random_control_bounds: tuple[float, float, float] = (0.3, 1.2, 1.0)
    steps: int = 200


@dataclass(slots=True)
class Simulator:
    """Generate controls, measurements, and ground-truth pose for EKF-SLAM demos."""

    world: object
    config: SimConfig = field(default_factory=SimConfig)
    seed: int | None = None
    initial_pose: Pose2D = field(default_factory=lambda: Pose2D(0.0, 0.0, 0.0))

    def __post_init__(self) -> None:
        self._rng = np.random.default_rng(self.seed)
        self._step_count = 0
        self._time_s = 0.0
        self._true_pose = self.initial_pose
        self._smooth_control_state = np.array([self.config.nominal_v, 0.0], dtype=float)

    @property
    def control_cov(self) -> np.ndarray:
        """Control covariance matrix derived from configured standard deviations."""
        std_v, std_w = self.config.control_noise_std
        return np.diag([std_v**2, std_w**2])

    @property
    def measurement_cov(self) -> np.ndarray:
        """Measurement covariance matrix derived from configured standard deviations."""
        std_r, std_b = self.config.measurement_noise_std
        return np.diag([std_r**2, std_b**2])

    def reset(self, seed: int | None = None) -> None:
        """Reset simulator state and random generator.

        Parameters
        ----------
        seed:
            If provided, use this seed. Otherwise the constructor seed is used.
        """
        seed_to_use = self.seed if seed is None else seed
        self._rng = np.random.default_rng(seed_to_use)
        self._step_count = 0
        self._time_s = 0.0
        self._true_pose = self.initial_pose
        self._smooth_control_state = np.array([self.config.nominal_v, 0.0], dtype=float)

    def step(self) -> tuple[ControlInput, float, list[Measurement], Pose2D]:
        """Advance simulation by one timestep.

        Returns
        -------
        tuple
            ``(u, dt, measurements, ground_truth_pose)`` where ``u`` is noisy
            odometry with optional bias.
        """
        control_true = self._trajectory_control(self._step_count, self._time_s)
        self._true_pose = self._propagate_pose(self._true_pose, control_true, self.config.dt)
        control_odom = self._sample_noisy_odometry(control_true)
        measurements = self._sample_measurements(self._true_pose)

        self._step_count += 1
        self._time_s += self.config.dt
        return control_odom, self.config.dt, measurements, self._true_pose

    def run(self, N: int | None = None) -> list[tuple[ControlInput, float, list[Measurement], Pose2D]]:
        """Run multiple steps and collect outputs from :meth:`step`.

        If ``N`` is omitted, ``config.steps`` is used for compatibility with
        older demo code.
        """
        total_steps = self.config.steps if N is None else N
        return [self.step() for _ in range(total_steps)]

    def run_steps(self, N: int) -> list[SimulationStep]:
        """Compatibility helper returning legacy ``SimulationStep`` objects."""
        outputs: list[SimulationStep] = []
        for _ in range(N):
            control, _, measurements, gt_pose = self.step()
            outputs.append(SimulationStep(true_pose=gt_pose, control=control, measurements=measurements))
        return outputs

    def _trajectory_control(self, step_idx: int, time_s: float) -> ControlInput:
        mode = self.config.trajectory_mode
        if mode == "rectangle":
            return self._rectangle_control(step_idx)
        if mode == "figure_eight":
            return self._figure_eight_control(time_s)
        if mode == "random_smooth":
            return self._random_smooth_control()
        raise ValueError(f"Unsupported trajectory mode: {mode}")

    def _rectangle_control(self, step_idx: int) -> ControlInput:
        """Piecewise control: straight segments connected by 90deg turns."""
        forward_steps = 40
        turn_steps = 10
        cycle = forward_steps + turn_steps
        phase = step_idx % cycle
        if phase < forward_steps:
            return ControlInput(v=self.config.nominal_v, w=0.0)
        turn_rate = (np.pi / 2.0) / (turn_steps * self.config.dt)
        return ControlInput(v=0.0, w=turn_rate)

    def _figure_eight_control(self, time_s: float) -> ControlInput:
        """Smooth figure-eight using sinusoidal yaw-rate oscillation."""
        v = self.config.nominal_v
        w = self.config.nominal_w * np.sin(0.4 * time_s)
        return ControlInput(v=v, w=w)

    def _random_smooth_control(self) -> ControlInput:
        """Random control sampled from a slowly varying first-order process."""
        v_min, v_max, w_abs_max = self.config.random_control_bounds
        target = np.array(
            [
                self._rng.uniform(v_min, v_max),
                self._rng.uniform(-w_abs_max, w_abs_max),
            ],
            dtype=float,
        )
        # Low-pass filter for smooth controls and better SLAM observability.
        alpha = 0.08
        self._smooth_control_state = (1.0 - alpha) * self._smooth_control_state + alpha * target

        # Soft boundary steering if we approach the configured world limits.
        xlim, ylim = self.config.world_limits
        margin = 2.0
        if self._true_pose.x > xlim[1] - margin:
            self._smooth_control_state[1] += 0.4
        elif self._true_pose.x < xlim[0] + margin:
            self._smooth_control_state[1] -= 0.4
        if self._true_pose.y > ylim[1] - margin:
            self._smooth_control_state[1] += 0.4
        elif self._true_pose.y < ylim[0] + margin:
            self._smooth_control_state[1] -= 0.4

        self._smooth_control_state[0] = float(np.clip(self._smooth_control_state[0], v_min, v_max))
        self._smooth_control_state[1] = float(np.clip(self._smooth_control_state[1], -w_abs_max, w_abs_max))
        return ControlInput(v=float(self._smooth_control_state[0]), w=float(self._smooth_control_state[1]))

    def _sample_noisy_odometry(self, control_true: ControlInput) -> ControlInput:
        std_v, std_w = self.config.control_noise_std
        bias_v, bias_w = self.config.control_bias
        noise = self._rng.normal(loc=[0.0, 0.0], scale=[std_v, std_w], size=2)
        return ControlInput(
            v=float(control_true.v + bias_v + noise[0]),
            w=float(control_true.w + bias_w + noise[1]),
        )

    def _sample_measurements(self, pose: Pose2D) -> list[Measurement]:
        range_std, bearing_std = self.config.measurement_noise_std
        measurements: list[Measurement] = []

        for landmark in self.world.landmarks:
            dx = landmark.x - pose.x
            dy = landmark.y - pose.y
            range_true = float(np.hypot(dx, dy))
            if range_true > self.config.sensor_range:
                continue

            bearing_true = wrap_angle(float(np.arctan2(dy, dx) - pose.yaw))
            if abs(bearing_true) > 0.5 * self.config.fov_rad:
                continue

            if self._rng.random() < self.config.measurement_dropout_prob:
                continue

            range_noisy = range_true + self._rng.normal(0.0, range_std)
            bearing_noisy = wrap_angle(bearing_true + self._rng.normal(0.0, bearing_std))
            measurements.append(
                Measurement(
                    landmark_id=landmark.landmark_id,
                    range_m=float(max(range_noisy, 0.0)),
                    bearing_rad=float(bearing_noisy),
                )
            )

        return measurements

    @staticmethod
    def _propagate_pose(pose: Pose2D, control: ControlInput, dt: float) -> Pose2D:
        x = pose.x + control.v * dt * np.cos(pose.yaw)
        y = pose.y + control.v * dt * np.sin(pose.yaw)
        yaw = wrap_angle(pose.yaw + control.w * dt)
        return Pose2D(x=float(x), y=float(y), yaw=float(yaw))


# Backward-compatible names used by earlier examples/tests.
SimulatorConfig = SimConfig
Simulator2D = Simulator
