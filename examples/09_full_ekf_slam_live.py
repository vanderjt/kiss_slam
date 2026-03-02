"""Teach: full EKF-SLAM demo with clean CLI flags for repeatable experiments."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

import argparse

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import ArtistAnimation, PillowWriter

from kiss_slam.ekf_slam import EKFSLAM
from kiss_slam.models.measurement import RangeBearingMeasurementModel
from kiss_slam.models.motion import DifferentialDriveMotionModel
from kiss_slam.sim.simulator import SimConfig, Simulator2D
from kiss_slam.sim.world import World2D
from kiss_slam.types import EKFSLAMConfig
from kiss_slam.viz.live_viewer import LiveViewer


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Full EKF-SLAM live demo")
    parser.add_argument("--steps", type=int, default=180)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--no-viz", action="store_true")
    parser.add_argument("--save-gif", type=Path, default=None, help="Optional output path for GIF animation.")
    parser.add_argument("--show-innovations", action="store_true", help="Draw innovation vectors in viewer.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    world = World2D.random(seed=args.seed, n_landmarks=20, xlim=(-20, 20), ylim=(-20, 20))
    sim = Simulator2D(world=world, config=SimConfig(steps=min(args.steps, 200), dt=0.1), seed=args.seed)

    slam = EKFSLAM(
        motion_model=DifferentialDriveMotionModel(),
        measurement_model=RangeBearingMeasurementModel(),
        config=EKFSLAMConfig(process_noise=sim.control_cov, measurement_noise=sim.measurement_cov),
    )

    viewer = None if args.no_viz else LiveViewer(show_ground_truth=True, show_covariance=True, show_innovations=args.show_innovations)
    true_traj: list[np.ndarray] = []
    est_traj: list[np.ndarray] = []
    true_landmarks = np.array([[landmark.x, landmark.y] for landmark in world.landmarks], dtype=float)
    frames: list[np.ndarray] = []

    for control, dt, measurements, true_pose in sim.run(min(args.steps, 200)):
        slam.step(u=control, dt=dt, measurements=measurements)

        true_traj.append(true_pose.as_array())
        est_pose = slam.robot_pose()
        est_traj.append(est_pose)
        est_landmarks = slam.landmark_states()

        if viewer is not None:
            viewer.update(
                true_traj=np.array(true_traj),
                est_traj=np.array(est_traj),
                current_pose=est_pose,
                true_landmarks=true_landmarks,
                est_landmarks=est_landmarks,
                state_cov=slam.get_covariance(),
                landmark_covariances={landmark_id: slam.landmark_covariance(landmark_id) for landmark_id in est_landmarks},
                innovation_vectors=slam.innovation_vectors(),
            )
            if args.save_gif is not None:
                viewer.fig.canvas.draw()
                frame = np.frombuffer(viewer.fig.canvas.tostring_rgb(), dtype=np.uint8)
                frame = frame.reshape(viewer.fig.canvas.get_width_height()[::-1] + (3,))
                frames.append(frame.copy())

    if args.save_gif is not None and frames:
        args.save_gif.parent.mkdir(parents=True, exist_ok=True)
        fig = plt.figure(figsize=(6, 5))
        ims = []
        for frame in frames:
            ims.append([plt.imshow(frame, animated=True)])
        ani = ArtistAnimation(fig, ims, interval=66, blit=True)
        ani.save(args.save_gif, writer=PillowWriter(fps=15))
        plt.close(fig)
        print(f"Saved gif: {args.save_gif}")

    if slam.nis_values:
        print(f"Mean NIS: {np.mean(slam.nis_values):.3f}")
    print("Full demo complete.")

    if viewer is not None:
        plt.show(block=False)
        plt.pause(1.0)
        plt.close("all")


if __name__ == "__main__":
    main()
