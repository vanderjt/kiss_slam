# 07 — Visualization

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
Good visualization cuts debugging time drastically. This viewer keeps artists alive for speed and updates data in-place.

## Relevant runnable example
- `examples/08_ekf_slam_small_world.py`

## Where to look
- Viewer class: `src/kiss_slam/viz/live_viewer.py`
- Demo key bindings (`g`, `c`, `i`, space): `examples/ekf_slam_demo.py`

## Run example
```bash
python examples/08_ekf_slam_small_world.py
```

## Try this
- Disable covariance display and compare frame rate.
- Toggle innovation vectors to debug measurement disagreements.

## Common bugs
> **Bug: flickering or slow plotting**  
> Diagnose: check whether new artists are created every frame.  
> Fix: reuse artists (as `LiveViewer` already does).
