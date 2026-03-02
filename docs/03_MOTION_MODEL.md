# 03 — Motion Model

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
Motion prediction is dead-reckoning: “given last pose + control, where should robot be now?”
The EKF also needs Jacobians to propagate uncertainty.

## Where to look
- Prediction equations: `src/kiss_slam/models/motion.py`
- How EKF uses `Fx`/`Fu`: `src/kiss_slam/ekf_slam.py`

## Run example
```bash
python examples/tutorial_03_motion_model.py
```

## Try this
- Replace turning rate with zero and confirm straight-line motion.
- Compare analytic Jacobian to numerical Jacobian in the script output.

## Common bugs
> **Bug: robot heading drifts beyond ±pi**  
> Diagnose: inspect yaw values after each step.  
> Fix: always apply angle wrapping (already done in model and EKF).
