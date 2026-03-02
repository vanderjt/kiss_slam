"""Teach: synthetic range/bearing measurements and ray visualization from one pose."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np

from examples._example_utils import finalize_plot, landmarks_to_array, make_parser
from kiss_slam.models.measurement import RangeBearingMeasurementModel
from kiss_slam.sim.world import World2D
from kiss_slam.types import Measurement


def main() -> None:
    args = make_parser("Measurement model only demo", default_steps=0, default_seed=5).parse_args()
    rng = np.random.default_rng(args.seed)
    model = RangeBearingMeasurementModel()

    robot = np.array([1.0, -1.0, np.deg2rad(25.0)], dtype=float)
    world = World2D.random(seed=args.seed, n_landmarks=6, xlim=(-6, 6), ylim=(-6, 6))

    measurements: list[Measurement] = []
    for lm in world.landmarks:
        z = model.predict(robot_state=robot, landmark_state=np.array([lm.x, lm.y], dtype=float))
        z_noisy = z + rng.normal([0.0, 0.0], [0.1, np.deg2rad(1.5)])
        measurements.append(Measurement(landmark_id=lm.landmark_id, range_m=float(z_noisy[0]), bearing_rad=float(z_noisy[1])))

    for z in measurements:
        print(f"landmark={z.landmark_id:02d} range={z.range_m:.2f}m bearing={np.rad2deg(z.bearing_rad):+.1f}deg")

    lm_xy = landmarks_to_array(world.landmarks)
    plt.figure(figsize=(6, 5))
    plt.scatter(lm_xy[:, 0], lm_xy[:, 1], c="k", marker="*", s=120, label="landmarks")
    plt.scatter(robot[0], robot[1], c="tab:blue", s=80, label="robot")

    for z in measurements:
        ang = robot[2] + z.bearing_rad
        end = robot[:2] + z.range_m * np.array([np.cos(ang), np.sin(ang)])
        plt.plot([robot[0], end[0]], [robot[1], end[1]], "r--", alpha=0.7)

    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    finalize_plot(args.no_viz, title="04_measurement_model_only")


if __name__ == "__main__":
    main()
