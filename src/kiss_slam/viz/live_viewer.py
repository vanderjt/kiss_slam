"""Live plotting helper for 2D SLAM demos."""

from __future__ import annotations

from dataclasses import dataclass, field

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.patches import Ellipse

from kiss_slam.math_utils import covariance_ellipse_params


@dataclass(slots=True)
class LiveViewer:
    """Efficient Matplotlib-based visualizer for truth and EKF estimates.

    Notes
    -----
    The viewer keeps Matplotlib artists alive and updates data in-place to keep
    frame updates lightweight in regular Anaconda environments.
    """

    show_ground_truth: bool = True
    show_covariance: bool = True
    show_innovations: bool = True
    heading_length_m: float = 0.9
    fig: plt.Figure = field(init=False)
    ax: plt.Axes = field(init=False)

    def __post_init__(self) -> None:
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.fig.canvas.manager.set_window_title("kiss_slam live viewer")

        self.ax.set_title("kiss_slam live view")
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.axis("equal")
        self.ax.grid(True, alpha=0.3)

        (self._true_traj_line,) = self.ax.plot([], [], "k-", lw=1.6, label="true trajectory")
        (self._est_traj_line,) = self.ax.plot([], [], "tab:blue", ls="--", lw=1.8, label="estimated trajectory")

        self._true_landmarks = self.ax.scatter([], [], c="k", marker="x", label="true landmarks")
        self._est_landmarks = self.ax.scatter([], [], c="tab:blue", marker="o", label="estimated landmarks")

        (self._heading_line,) = self.ax.plot([], [], color="tab:orange", lw=2.0, label="heading")

        self._robot_cov_ellipse = Ellipse((0.0, 0.0), width=0.0, height=0.0, angle=0.0, fill=False, edgecolor="tab:blue", lw=1.8)
        self.ax.add_patch(self._robot_cov_ellipse)

        self._landmark_cov_ellipses: dict[int, Ellipse] = {}
        self._innovation_lines = LineCollection([], colors="tab:red", linewidths=1.2, alpha=0.8, label="innovations")
        self.ax.add_collection(self._innovation_lines)

        self.ax.legend(loc="upper right")

    def update(
        self,
        true_traj: np.ndarray,
        est_traj: np.ndarray,
        current_pose: np.ndarray,
        true_landmarks: np.ndarray,
        est_landmarks: dict[int, np.ndarray],
        state_cov: np.ndarray,
        landmark_covariances: dict[int, np.ndarray],
        innovation_vectors: list[tuple[np.ndarray, np.ndarray]] | None = None,
    ) -> None:
        """Update existing artists with newest state estimate and measurements."""
        innovation_vectors = [] if innovation_vectors is None else innovation_vectors

        self._true_traj_line.set_data(true_traj[:, 0], true_traj[:, 1]) if true_traj.size else self._true_traj_line.set_data([], [])
        self._true_traj_line.set_visible(self.show_ground_truth)

        self._est_traj_line.set_data(est_traj[:, 0], est_traj[:, 1]) if est_traj.size else self._est_traj_line.set_data([], [])

        if self.show_ground_truth and true_landmarks.size:
            self._true_landmarks.set_offsets(true_landmarks)
        else:
            self._true_landmarks.set_offsets(np.empty((0, 2)))
        self._true_landmarks.set_visible(self.show_ground_truth)

        if est_landmarks:
            ids_sorted = sorted(est_landmarks)
            est_xy = np.array([est_landmarks[lid] for lid in ids_sorted], dtype=float)
            self._est_landmarks.set_offsets(est_xy)
        else:
            ids_sorted = []
            self._est_landmarks.set_offsets(np.empty((0, 2)))

        heading_end = self._heading_endpoint(current_pose)
        self._heading_line.set_data([current_pose[0], heading_end[0]], [current_pose[1], heading_end[1]])

        if self.show_covariance and state_cov.shape[0] >= 2:
            self._set_ellipse(self._robot_cov_ellipse, center_xy=current_pose[:2], cov_xy=state_cov[:2, :2])
            self._robot_cov_ellipse.set_visible(True)
        else:
            self._robot_cov_ellipse.set_visible(False)

        active_landmark_ids: set[int] = set()
        for landmark_id in ids_sorted:
            cov = landmark_covariances.get(landmark_id)
            if cov is None:
                continue
            active_landmark_ids.add(landmark_id)
            ellipse = self._landmark_cov_ellipses.get(landmark_id)
            if ellipse is None:
                ellipse = Ellipse((0.0, 0.0), width=0.0, height=0.0, angle=0.0, fill=False, edgecolor="tab:cyan", lw=1.0, alpha=0.8)
                self._landmark_cov_ellipses[landmark_id] = ellipse
                self.ax.add_patch(ellipse)

            if self.show_covariance:
                self._set_ellipse(ellipse, center_xy=est_landmarks[landmark_id], cov_xy=cov)
                ellipse.set_visible(True)
            else:
                ellipse.set_visible(False)

        for landmark_id, ellipse in self._landmark_cov_ellipses.items():
            if landmark_id not in active_landmark_ids:
                ellipse.set_visible(False)

        if self.show_innovations and innovation_vectors:
            segments = [[start_xy, end_xy] for start_xy, end_xy in innovation_vectors]
            self._innovation_lines.set_segments(segments)
            self._innovation_lines.set_visible(True)
        else:
            self._innovation_lines.set_segments([])
            self._innovation_lines.set_visible(False)

        self._autoscale_view(true_traj=true_traj, est_traj=est_traj, true_landmarks=true_landmarks, est_landmarks=est_landmarks)
        self.fig.canvas.draw_idle()
        plt.pause(0.001)

    def _autoscale_view(
        self,
        true_traj: np.ndarray,
        est_traj: np.ndarray,
        true_landmarks: np.ndarray,
        est_landmarks: dict[int, np.ndarray],
    ) -> None:
        points: list[np.ndarray] = []
        if true_traj.size and self.show_ground_truth:
            points.append(true_traj)
        if est_traj.size:
            points.append(est_traj)
        if true_landmarks.size and self.show_ground_truth:
            points.append(true_landmarks)
        if est_landmarks:
            points.append(np.array(list(est_landmarks.values()), dtype=float))

        if not points:
            return

        all_pts = np.vstack(points)
        x_min, y_min = np.min(all_pts, axis=0)
        x_max, y_max = np.max(all_pts, axis=0)
        pad = 1.5
        self.ax.set_xlim(x_min - pad, x_max + pad)
        self.ax.set_ylim(y_min - pad, y_max + pad)

    def _heading_endpoint(self, pose: np.ndarray) -> np.ndarray:
        return np.array(
            [
                pose[0] + self.heading_length_m * np.cos(pose[2]),
                pose[1] + self.heading_length_m * np.sin(pose[2]),
            ],
            dtype=float,
        )

    @staticmethod
    def _set_ellipse(ellipse: Ellipse, center_xy: np.ndarray, cov_xy: np.ndarray) -> None:
        center, width, height, angle = covariance_ellipse_params(center_xy, cov_xy, n_std=2.0)
        ellipse.set_center((center[0], center[1]))
        ellipse.width = width
        ellipse.height = height
        ellipse.angle = angle
