# 04 — Measurement Model

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
The sensor predicts how a landmark should look from robot pose: range + bearing.
Update step compares predicted measurement against actual measurement.

## Relevant runnable example
- `examples/04_measurement_model_only.py`

## Where to look
- Expected measurement and Jacobians: `src/kiss_slam/models/measurement.py`
- Landmark triangulation from first observation: same file (`initialize_landmark_from_measurement`).

## Run example
```bash
python examples/04_measurement_model_only.py
```

## Try this
- Move landmark behind robot and verify bearing wraps correctly.
- Add small perturbation to robot yaw and watch expected bearing change.

## Common bugs
> **Bug: update spikes near ±pi bearing**  
> Diagnose: log innovation angle before/after wrapping.  
> Fix: wrap bearing innovations with `wrap_angle`.
