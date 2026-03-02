# 09 — Using in a Real Robot

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
Integrating on a real robot means replacing simulator inputs with real odometry + landmark detections while keeping the same EKF API.

## Relevant runnable example
- `examples/09_full_ekf_slam_live.py`

## Where to look
- Real-world guidance: `docs/USAGE_IN_ROBOT.md`
- Input dataclasses: `src/kiss_slam/types.py`
- Predict/update API: `src/kiss_slam/ekf_slam.py`

## Minimal integration loop
```python
slam.predict(u=ControlInput(v=v_mps, w=w_rps), dt=dt)
slam.update(measurements=measurement_list)
pose, landmarks = slam.get_state()
```

## Run bridge-style example
```bash
python examples/09_full_ekf_slam_live.py --no-viz
```

## Try this
- Feed a logged dataset (CSV) instead of live data.
- Add timestamp sanity checks to guard against variable `dt` spikes.

## Common bugs
> **Bug: inconsistent units from sensors**  
> Diagnose: verify radians vs degrees and meters vs millimeters at ingestion boundary.  
> Fix: normalize units before creating `ControlInput` and `Measurement`.

> **Bug: association fails in clutter**  
> Diagnose: are IDs stable? if not, enable nearest-neighbor with realistic gating.  
> Fix: tune gate and reject low-confidence detections.
