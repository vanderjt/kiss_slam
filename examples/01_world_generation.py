"""Teach: generate a simple random landmark world and visualize landmark positions."""

from __future__ import annotations

import matplotlib.pyplot as plt

from examples._example_utils import finalize_plot, landmarks_to_array, make_parser
from kiss_slam.sim.world import World2D


def main() -> None:
    args = make_parser("World generation demo", default_steps=0, default_seed=7).parse_args()
    world = World2D.random(seed=args.seed, n_landmarks=8, xlim=(-10.0, 10.0), ylim=(-8.0, 8.0))

    print("Landmarks:")
    for landmark in world.landmarks:
        print(f"  id={landmark.landmark_id:02d}, x={landmark.x:+.2f}, y={landmark.y:+.2f}")

    xy = landmarks_to_array(world.landmarks)
    plt.figure(figsize=(5, 4))
    plt.scatter(xy[:, 0], xy[:, 1], marker="*", s=120, label="landmarks")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.grid(True)
    plt.axis("equal")
    plt.legend()
    finalize_plot(args.no_viz, title="01_world_generation")


if __name__ == "__main__":
    main()
