"""Teach: EKF prediction-only behavior and covariance growth over time."""

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
from kiss_slam.types import ControlInput, EKFSLAMConfig


def main() -> None:
    args = make_parser("EKF predict-only demo", default_steps=120, default_seed=0).parse_args()
    slam = EKFSLAM(
        motion_model=DifferentialDriveMotionModel(),
        measurement_model=RangeBearingMeasurementModel(),
        config=EKFSLAMConfig(),
    )

    det_history = []
    for k in range(args.steps):
        u = ControlInput(v=1.0, w=0.2 * np.sin(0.1 * k))
        slam.predict(u=u, dt=0.1)
        det_history.append(float(np.linalg.det(slam.get_covariance()[:2, :2])))

    plt.figure(figsize=(6, 4))
    plt.plot(det_history)
    plt.yscale("log")
    plt.xlabel("step")
    plt.ylabel("det(P_xy)")
    plt.grid(True)
    finalize_plot(args.no_viz, title="06_ekf_predict_only")


if __name__ == "__main__":
    main()
