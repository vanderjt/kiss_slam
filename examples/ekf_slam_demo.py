r"""Run an end-to-end EKF-SLAM demo with simulation and live visualization."""

from __future__ import annotations

import matplotlib.pyplot as plt
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

    viewer = LiveViewer(show_ground_truth=True, show_covariance=True, show_innovations=True)
    paused = False

    def on_key_press(event: plt.KeyEvent) -> None:
        nonlocal paused
        if event.key == "g":
            viewer.show_ground_truth = not viewer.show_ground_truth
            print(f"Ground truth: {'ON' if viewer.show_ground_truth else 'OFF'}")
        elif event.key == "c":
            viewer.show_covariance = not viewer.show_covariance
            print(f"Covariances: {'ON' if viewer.show_covariance else 'OFF'}")
        elif event.key == "i":
            viewer.show_innovations = not viewer.show_innovations
            print(f"Innovations: {'ON' if viewer.show_innovations else 'OFF'}")
        elif event.key == " ":
            paused = not paused
            print(f"Simulation: {'PAUSED' if paused else 'RUNNING'}")

    viewer.fig.canvas.mpl_connect("key_press_event", on_key_press)

    true_traj: list[np.ndarray] = []
    est_traj: list[np.ndarray] = []
    true_landmarks = np.array([[landmark.x, landmark.y] for landmark in world.landmarks], dtype=float)

    for control, dt, measurements, true_pose in sim.run():
        while paused and plt.fignum_exists(viewer.fig.number):
            plt.pause(0.05)

        slam.step(u=control, dt=dt, measurements=measurements)

        true_traj.append(true_pose.as_array())
        est_pose = slam.robot_pose()
        est_traj.append(est_pose)

        est_landmarks = slam.landmark_states()
        landmark_covariances = {landmark_id: slam.landmark_covariance(landmark_id) for landmark_id in est_landmarks}

        viewer.update(
            true_traj=np.array(true_traj),
            est_traj=np.array(est_traj),
            current_pose=est_pose,
            true_landmarks=true_landmarks,
            est_landmarks=est_landmarks,
            state_cov=slam.get_covariance(),
            landmark_covariances=landmark_covariances,
            innovation_vectors=slam.innovation_vectors(),
        )

        if not plt.fignum_exists(viewer.fig.number):
            break

    if slam.nis_values:
        print(f"Mean NIS: {np.mean(slam.nis_values):.3f}")

    print("Demo complete. Close the plot window to exit.")
    plt.show()


if __name__ == "__main__":
    main()
