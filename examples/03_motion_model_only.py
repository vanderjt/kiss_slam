"""Teach: propagate robot pose with the motion model only (no SLAM update)."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np

from examples._example_utils import finalize_plot, make_parser
from kiss_slam.models.motion import DifferentialDriveMotionModel
from kiss_slam.types import ControlInput


def main() -> None:
    args = make_parser("Motion model only demo", default_steps=80, default_seed=0).parse_args()
    model = DifferentialDriveMotionModel()

    pose = np.array([0.0, 0.0, 0.0], dtype=float)
    trace = [pose.copy()]
    dt = 0.1
    for k in range(args.steps):
        control = ControlInput(v=1.0, w=0.35 * np.sin(0.1 * k))
        pose = model.predict_state(pose, control, dt)
        trace.append(pose.copy())

    trace_arr = np.vstack(trace)
    plt.figure(figsize=(5, 4))
    plt.plot(trace_arr[:, 0], trace_arr[:, 1], "-b", label="pose trace")
    plt.scatter(trace_arr[0, 0], trace_arr[0, 1], c="g", label="start")
    plt.scatter(trace_arr[-1, 0], trace_arr[-1, 1], c="r", label="end")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    finalize_plot(args.no_viz, title="03_motion_model_only")


if __name__ == "__main__":
    main()
