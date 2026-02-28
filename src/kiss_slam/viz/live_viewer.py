"""Live plotting helper for 2D SLAM demos."""

from __future__ import annotations

from dataclasses import dataclass, field

import matplotlib.pyplot as plt
import numpy as np

from kiss_slam.math_utils import covariance_ellipse


@dataclass(slots=True)
class LiveViewer:
    """Matplotlib-based visualizer for truth and EKF estimates."""

    show_covariance: bool = True
    fig: plt.Figure = field(init=False)
    ax: plt.Axes = field(init=False)

    def __post_init__(self) -> None:
        self.fig, self.ax = plt.subplots(figsize=(8, 8))

    def update(
        self,
        true_traj: np.ndarray,
        est_traj: np.ndarray,
        true_landmarks: np.ndarray,
        est_landmarks: np.ndarray,
        state_cov: np.ndarray,
    ) -> None:
        """Redraw current SLAM state."""
        self.ax.clear()
        self.ax.set_title("kiss_slam live view")
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.axis("equal")
        self.ax.grid(True, alpha=0.3)

        if true_traj.size:
            self.ax.plot(true_traj[:, 0], true_traj[:, 1], "k-", label="true trajectory")
        if est_traj.size:
            self.ax.plot(est_traj[:, 0], est_traj[:, 1], "b--", label="estimated trajectory")

        if true_landmarks.size:
            self.ax.scatter(true_landmarks[:, 0], true_landmarks[:, 1], c="k", marker="x", label="true landmarks")
        if est_landmarks.size:
            self.ax.scatter(est_landmarks[:, 0], est_landmarks[:, 1], c="tab:blue", marker="o", label="estimated landmarks")

        if self.show_covariance and state_cov.shape[0] >= 2:
            robot_xy = est_traj[-1, :2]
            ellipse = covariance_ellipse(robot_xy, state_cov[:2, :2], edgecolor="tab:blue")
            self.ax.add_patch(ellipse)

        self.ax.legend(loc="upper right")
        plt.pause(0.001)
