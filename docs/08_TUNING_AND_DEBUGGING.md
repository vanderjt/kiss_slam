# 08 — Tuning and Debugging

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
Most EKF issues are not “bad math”; they are mismatch between assumed noise and real data.
Start with conservative noise, then tighten.

## Where to look
- Noise defaults and Joseph toggle: `src/kiss_slam/types.py`
- NIS collection and innovation vectors: `src/kiss_slam/ekf_slam.py`
- Existing guidance: `docs/CONFIG_TUNING.md`

## Run example
```bash
python examples/tutorial_08_tuning_debugging.py
```

## Try this
1. Multiply process noise by 4 and compare RMSE.
2. Multiply measurement noise by 0.25 and inspect innovation spikes.

## Common bugs
> **Bug: “map looks good but robot track is bad”**  
> Diagnose: inspect odometry bias and process noise first.  
> Fix: calibrate odometry or increase `Q`.

> **Bug: “updates over-correct and oscillate”**  
> Diagnose: measurement noise likely too small.  
> Fix: increase `R` and verify sensor model assumptions.
