# 05 — Data Association

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
Data association answers: “which landmark does this measurement belong to?”
Baseline in this repo uses known landmark IDs. Extension supports nearest-neighbor with Mahalanobis gating.

## Relevant runnable example
- `examples/05_data_association_demo.py`

## Where to look
- Association strategies: `src/kiss_slam/data_association.py`
- EKF hook point (`_associate`): `src/kiss_slam/ekf_slam.py`

## Run example
```bash
python examples/05_data_association_demo.py
```

## Try this
- Tighten gate threshold from `5.99` to `2.0` and see more “new landmarks”.
- Loosen to `20.0` and observe risk of wrong matches.

## Common bugs
> **Bug: duplicate landmarks explode map size**  
> Diagnose: track how often associator returns `None` for already-known areas.  
> Fix: improve noise models and gate threshold.
