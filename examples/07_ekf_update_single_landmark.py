"""Teach: one-landmark EKF-SLAM predict/update loop and estimate convergence."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

import matplotlib.pyplot as plt
import numpy as np

from _example_utils import finalize_plot, make_parser
from kiss_slam.ekf_slam import EKFSLAM
from kiss_slam.models.measurement import RangeBearingMeasurementModel
from kiss_slam.models.motion import DifferentialDriveMotionModel
from kiss_slam.types import ControlInput, EKFSLAMConfig, Measurement


def main() -> None:
    args = make_parser("Single-landmark update demo", default_steps=80, default_seed=1).parse_args()
    rng = np.random.default_rng(args.seed)
    landmark = np.array([6.0, 2.0], dtype=float)

    slam = EKFSLAM(
        motion_model=DifferentialDriveMotionModel(),
        measurement_model=RangeBearingMeasurementModel(),
        config=EKFSLAMConfig(),
    )

    true_pose = np.array([0.0, 0.0, 0.0], dtype=float)
    model = RangeBearingMeasurementModel()
    errors = []

    for _ in range(args.steps):
        u = ControlInput(v=0.6, w=0.02)
        true_pose = DifferentialDriveMotionModel().predict_state(true_pose, u, 0.1)
        z = model.predict(true_pose, landmark) + rng.normal([0.0, 0.0], [0.15, np.deg2rad(2.0)])
        slam.step(u=u, dt=0.1, measurements=[Measurement(landmark_id=0, range_m=float(z[0]), bearing_rad=float(z[1]))])
        errors.append(np.linalg.norm(slam.robot_pose()[:2] - true_pose[:2]))

    plt.figure(figsize=(6, 4))
    plt.plot(errors)
    plt.xlabel("step")
    plt.ylabel("position error [m]")
    plt.grid(True)
    finalize_plot(args.no_viz, title="07_ekf_update_single_landmark")


if __name__ == "__main__":
    main()
