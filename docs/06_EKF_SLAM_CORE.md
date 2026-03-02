# 06 — EKF-SLAM Core

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
EKF-SLAM alternates two phases:
1. **Predict** robot motion and uncertainty.
2. **Update** map + robot with landmark observations.

The state vector grows when new landmarks are initialized.

## Relevant runnable example
- `examples/06_ekf_predict_only.py and examples/07_ekf_update_single_landmark.py`

## Where to look
- State layout, predict/update internals: `src/kiss_slam/ekf_slam.py`
- Config defaults: `src/kiss_slam/types.py` (`EKFSLAMConfig`)

## Run example
```bash
python examples/06_ekf_predict_only.py
```

## Try this
- Turn off Joseph form (`use_joseph_form=False`) and compare numerical stability.
- Print NIS statistics over a run and compare against expected χ² range.

## Common bugs
> **Bug: covariance becomes non-symmetric / unstable**  
> Diagnose: check `np.allclose(Sigma, Sigma.T)` occasionally.  
> Fix: keep Joseph form on, and tune noise less aggressively.
