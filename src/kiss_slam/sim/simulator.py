"""Simple 2D simulator for landmark-based SLAM."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from kiss_slam.math_utils import wrap_angle
from kiss_slam.types import ControlInput, Measurement, Pose2D, SimulationStep


@dataclass(slots=True)
class SimulatorConfig:
    """Configuration values for `Simulator2D`."""

    dt: float = 0.1
    steps: int = 200
    sensor_range: float = 12.0
    fov_rad: float = np.pi
    control_v: float = 1.0
    control_w: float = 0.2


@dataclass(slots=True)
class Simulator2D:
    """Generate synthetic controls and measurements from a known world."""

    world: object
    config: SimulatorConfig
    seed: int = 0
    control_cov: np.ndarray = field(default_factory=lambda: np.diag([0.05**2, 0.03**2]))
    measurement_cov: np.ndarray = field(default_factory=lambda: np.diag([0.2**2, (np.deg2rad(3.0)) ** 2]))

    def __post_init__(self) -> None:
        self._rng = np.random.default_rng(self.seed)
        self._true_pose = Pose2D(x=0.0, y=0.0, yaw=0.0)

    def run(self) -> list[SimulationStep]:
        """Run simulation and return all generated steps."""
        steps: list[SimulationStep] = []
        for idx in range(self.config.steps):
            control_true = self._control_command(step_idx=idx)
            self._true_pose = self._propagate_pose(self._true_pose, control_true, self.config.dt)
            noisy_control = self._sample_noisy_control(control_true)
            measurements = self._sample_measurements(self._true_pose)
            steps.append(
                SimulationStep(
                    true_pose=self._true_pose,
                    control=noisy_control,
                    measurements=measurements,
                )
            )
        return steps

    def _control_command(self, step_idx: int) -> ControlInput:
        curve = 0.5 * np.sin(0.03 * step_idx)
        return ControlInput(v=self.config.control_v, w=self.config.control_w * curve)

    def _sample_noisy_control(self, control_true: ControlInput) -> ControlInput:
        noise = self._rng.multivariate_normal(np.zeros(2), self.control_cov)
        return ControlInput(v=control_true.v + noise[0], w=control_true.w + noise[1])

    def _sample_measurements(self, pose: Pose2D) -> list[Measurement]:
        measurements: list[Measurement] = []
        for landmark in self.world.landmarks:
            dx = landmark.x - pose.x
            dy = landmark.y - pose.y
            rng = float(np.hypot(dx, dy))
            if rng > self.config.sensor_range:
                continue

            bearing = wrap_angle(float(np.arctan2(dy, dx) - pose.yaw))
            if abs(bearing) > self.config.fov_rad * 0.5:
                continue

            noise = self._rng.multivariate_normal(np.zeros(2), self.measurement_cov)
            measurements.append(
                Measurement(
                    landmark_id=landmark.landmark_id,
                    range_m=rng + noise[0],
                    bearing_rad=wrap_angle(bearing + noise[1]),
                )
            )
        return measurements

    @staticmethod
    def _propagate_pose(pose: Pose2D, control: ControlInput, dt: float) -> Pose2D:
        x = pose.x + control.v * dt * np.cos(pose.yaw)
        y = pose.y + control.v * dt * np.sin(pose.yaw)
        yaw = wrap_angle(pose.yaw + control.w * dt)
        return Pose2D(x=float(x), y=float(y), yaw=float(yaw))
