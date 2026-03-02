# 02 — Simulator and World

## Navigation
- [00 — Getting Started](00_GETTING_STARTED.md)
- [01 — Repo Tour](01_REPO_TOUR.md)
- [02 — Simulator and World](02_SIMULATOR_AND_WORLD.md)
- [03 — Motion Model](03_MOTION_MODEL.md)
- [04 — Measurement Model](04_MEASUREMENT_MODEL.md)
- [05 — Data Association](05_DATA_ASSOCIATION.md)
- [06 — EKF-SLAM Core](06_EKF_SLAM_CORE.md)
- [07 — Visualization](07_VISUALIZATION.md)
- [08 — Tuning and Debugging](08_TUNING_AND_DEBUGGING.md)
- [09 — Using in a Real Robot](09_USING_IN_REAL_ROBOT.md)

## Plain-English first
Before tuning an EKF, you need reproducible data. The simulator gives that: ground truth pose, noisy odometry, and noisy range-bearing landmark measurements.

## Relevant runnable example
- `examples/02_simulator_controls.py`

## Where to look
- Landmark placement: `src/kiss_slam/sim/world.py`
- Sim configuration and step loop: `src/kiss_slam/sim/simulator.py`

## Run examples
```bash
python examples/02_simulator_controls.py
```

## Key code snippet
```python
world = World2D.pattern("grid", n_landmarks=16, xlim=(-10, 10), ylim=(-10, 10), seed=3)
sim = Simulator(world=world, config=SimConfig(trajectory_mode="figure_eight", steps=50), seed=1)
for u, dt, measurements, true_pose in sim.run(10):
    print(u, len(measurements), true_pose)
```

## Try this
1. Set `measurement_dropout_prob=0.3` and inspect how often updates disappear.
2. Reduce `sensor_range` to 5m and watch map growth slow down.

## Common bugs
> **Bug: “my filter never initializes landmarks”**  
> Diagnose: check `sensor_range`, `fov_rad`, and dropout probability.  
> Fix: relax visibility settings, then tighten gradually.
