"""Step 02: inspect simulator outputs."""

from kiss_slam.sim.simulator import SimConfig, Simulator
from kiss_slam.sim.world import World2D


if __name__ == "__main__":
    world = World2D.pattern("grid", n_landmarks=9, xlim=(-10, 10), ylim=(-10, 10), seed=1)
    sim = Simulator(world=world, config=SimConfig(trajectory_mode="figure_eight", steps=3), seed=2)
    for index, (u, dt, measurements, gt) in enumerate(sim.run()):
        print(f"step={index} dt={dt:.2f} u=({u.v:.2f},{u.w:.2f}) measurements={len(measurements)} gt=({gt.x:.2f},{gt.y:.2f})")
