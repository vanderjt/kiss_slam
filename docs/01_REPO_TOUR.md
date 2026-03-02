# 01 — Repo Tour

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

## Plain-English overview
Here is the mental model: simulator generates controls + observations, EKF consumes them, viewer draws everything.

## Relevant runnable example
- `examples/01_world_generation.py`

## Module responsibilities
- `src/kiss_slam/ekf_slam.py`: owns joint state vector, covariance, predict/update loop.
- `src/kiss_slam/models/motion.py`: unicycle motion equations + Jacobians.
- `src/kiss_slam/models/measurement.py`: range-bearing prediction, Jacobians, landmark init.
- `src/kiss_slam/sim/world.py`: deterministic/random landmark world generators.
- `src/kiss_slam/sim/simulator.py`: trajectory, noisy odometry, noisy sensor, dropout.
- `src/kiss_slam/viz/live_viewer.py`: live matplotlib artists for trajectories/map/covariance.
- `src/kiss_slam/data_association.py`: known-ID baseline + nearest-neighbor gate.
- `src/kiss_slam/math_utils.py`: angle wrapping, Mahalanobis distance, ellipse math.
- `src/kiss_slam/types.py`: dataclass containers for pose/control/measurements/config.
- `examples/ekf_slam_demo.py`: end-to-end runnable demo.
- `tests/*`: unit and integration checks for math, models, simulator, EKF behavior.

## Run this tour script
```bash
python examples/01_world_generation.py
```

## Try this
- Open `examples/ekf_slam_demo.py`, find the keypress handler, and add your own toggle.
- Swap world pattern from random to circle in a custom script.

## Common bugs
> **Bug: hard to find where a behavior lives**  
> Diagnose: start from demo callsite, then “jump to definition” in IDE.  
> Fix: use this module map as your first index.
