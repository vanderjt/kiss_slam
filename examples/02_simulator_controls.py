"""Teach: step the simulator and inspect controls, dt, and true robot pose."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from kiss_slam.sim.simulator import SimConfig, Simulator2D
from kiss_slam.sim.world import World2D


def main() -> None:
    world = World2D.random(seed=3, n_landmarks=6, xlim=(-8, 8), ylim=(-8, 8))
    sim = Simulator2D(world=world, config=SimConfig(steps=20, dt=0.1), seed=11)

    for step_idx, (control, dt, _, true_pose) in enumerate(sim.run(10)):
        print(
            f"step={step_idx:02d} | u=[v={control.v:+.3f}, w={control.w:+.3f}] "
            f"| dt={dt:.2f} | true_pose=[{true_pose.x:+.3f}, {true_pose.y:+.3f}, {true_pose.yaw:+.3f}]"
        )


if __name__ == "__main__":
    main()
