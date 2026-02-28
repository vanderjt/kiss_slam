r"""Run an end-to-end EKF-SLAM demo with simulation and live visualization."""

from __future__ import annotations

import numpy as np

from kiss_slam.ekf_slam import EKFSLAM
from kiss_slam.models.measurement import RangeBearingMeasurementModel
from kiss_slam.models.motion import DifferentialDriveMotionModel
from kiss_slam.sim.simulator import Simulator2D, SimulatorConfig
from kiss_slam.sim.world import World2D
from kiss_slam.types import EKFSLAMConfig
from kiss_slam.viz.live_viewer import LiveViewer


def main() -> None:
    world = World2D.random(seed=7, n_landmarks=20, xlim=(-20, 20), ylim=(-20, 20))
    sim = Simulator2D(world=world, config=SimulatorConfig(steps=180, dt=0.1), seed=42)

    slam = EKFSLAM(
        motion_model=DifferentialDriveMotionModel(),
        measurement_model=RangeBearingMeasurementModel(),
        config=EKFSLAMConfig(process_noise=sim.control_cov, measurement_noise=sim.measurement_cov),
    )

    viewer = LiveViewer(show_covariance=True)
    true_traj: list[np.ndarray] = []
    est_traj: list[np.ndarray] = []
    true_landmarks = np.array([[landmark.x, landmark.y] for landmark in world.landmarks], dtype=float)

    for control, dt, measurements, true_pose in sim.run():
        slam.step(u=control, dt=dt, measurements=measurements)

        true_traj.append(true_pose.as_array())
        est_traj.append(slam.robot_pose())
        est_landmarks = np.array(list(slam.landmark_states().values()), dtype=float)

        viewer.update(
            true_traj=np.array(true_traj),
            est_traj=np.array(est_traj),
            true_landmarks=true_landmarks,
            est_landmarks=est_landmarks if est_landmarks.size else np.empty((0, 2)),
            state_cov=slam.get_covariance(),
        )

    if slam.nis_values:
        print(f"Mean NIS: {np.mean(slam.nis_values):.3f}")

    print("Demo complete. Close the plot window to exit.")
    import matplotlib.pyplot as plt

    plt.show()


if __name__ == "__main__":
    main()
