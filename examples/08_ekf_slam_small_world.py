"""Teach: short EKF-SLAM run in a tiny world with optional live visualization."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np

from examples._example_utils import finalize_plot, make_parser
from kiss_slam.ekf_slam import EKFSLAM
from kiss_slam.models.measurement import RangeBearingMeasurementModel
from kiss_slam.models.motion import DifferentialDriveMotionModel
from kiss_slam.sim.simulator import SimConfig, Simulator2D
from kiss_slam.sim.world import World2D
from kiss_slam.types import EKFSLAMConfig
from kiss_slam.viz.live_viewer import LiveViewer


def main() -> None:
    args = make_parser("Small-world EKF-SLAM demo", default_steps=120, default_seed=4).parse_args()
    world = World2D.random(seed=args.seed, n_landmarks=5, xlim=(-10, 10), ylim=(-10, 10))
    sim = Simulator2D(world=world, config=SimConfig(steps=args.steps, dt=0.1), seed=args.seed)

    slam = EKFSLAM(
        motion_model=DifferentialDriveMotionModel(),
        measurement_model=RangeBearingMeasurementModel(),
        config=EKFSLAMConfig(process_noise=sim.control_cov, measurement_noise=sim.measurement_cov),
    )

    viewer = None if args.no_viz else LiveViewer(show_ground_truth=True, show_covariance=True, show_innovations=False)
    true_traj: list[np.ndarray] = []
    est_traj: list[np.ndarray] = []
    true_landmarks = np.array([[lm.x, lm.y] for lm in world.landmarks], dtype=float)

    for control, dt, measurements, true_pose in sim.run(args.steps):
        slam.step(u=control, dt=dt, measurements=measurements)
        true_traj.append(true_pose.as_array())
        est_pose = slam.robot_pose()
        est_traj.append(est_pose)
        if viewer is not None:
            est_landmarks = slam.landmark_states()
            viewer.update(
                true_traj=np.array(true_traj),
                est_traj=np.array(est_traj),
                current_pose=est_pose,
                true_landmarks=true_landmarks,
                est_landmarks=est_landmarks,
                state_cov=slam.get_covariance(),
                landmark_covariances={lid: slam.landmark_covariance(lid) for lid in est_landmarks},
                innovation_vectors=slam.innovation_vectors(),
            )

    if viewer is None:
        plt.figure(figsize=(5, 4))
        tt = np.vstack(true_traj)
        ee = np.vstack(est_traj)
        plt.plot(tt[:, 0], tt[:, 1], "k-", label="true")
        plt.plot(ee[:, 0], ee[:, 1], "b--", label="estimated")
        plt.scatter(true_landmarks[:, 0], true_landmarks[:, 1], marker="*", c="tab:orange", label="landmarks")
        plt.axis("equal")
        plt.grid(True)
        plt.legend()
        finalize_plot(False, title="08_ekf_slam_small_world")
    else:
        finalize_plot(False, title="08_ekf_slam_small_world")


if __name__ == "__main__":
    main()
