# kiss_slam

`kiss_slam` is a small, readable Python package for 2D landmark EKF-SLAM.
It is designed for fast integration into non-ROS robotics projects (Python 3.10+).

## Install

```bash
pip install -e .
```

Development install (tests, lint tooling):

```bash
pip install -e .[dev]
```

## Quickstart

```python
import numpy as np

from kiss_slam import EKFSLAM, EKFSLAMConfig
from kiss_slam.models.motion import DifferentialDriveMotionModel
from kiss_slam.models.measurement import RangeBearingMeasurementModel
from kiss_slam.types import ControlInput, Measurement

slam = EKFSLAM(
    motion_model=DifferentialDriveMotionModel(),
    measurement_model=RangeBearingMeasurementModel(),
    config=EKFSLAMConfig(
        process_noise=np.diag([0.05**2, 0.03**2]),
        measurement_noise=np.diag([0.20**2, np.deg2rad(3.0) ** 2]),
    ),
)

u = ControlInput(v=1.0, w=0.1)
slam.predict(u=u, dt=0.1)
slam.update(
    measurements=[
        Measurement(landmark_id=1, range_m=4.2, bearing_rad=0.3),
        Measurement(landmark_id=2, range_m=6.5, bearing_rad=-0.2),
    ]
)

pose_est, landmarks_est = slam.get_state()
print(pose_est, len(landmarks_est))
```

Run the end-to-end simulator demo:

```bash
python examples/ekf_slam_demo.py
```

## API overview

Core classes for integration:

- `EKFSLAM`: estimator with `predict(...)`, `update(...)`, `step(...)`.
- `EKFSLAMConfig`: process/measurement noise, initial covariance, gating/Joseph options.
- `ControlInput(v, w)`: odometry input (m/s, rad/s).
- `Measurement(landmark_id, range_m, bearing_rad)`: range-bearing observation.
- `DifferentialDriveMotionModel`: unicycle-style motion model.
- `RangeBearingMeasurementModel`: expected measurement + Jacobians + landmark init.
- `KnownCorrespondenceAssociator`: baseline known-ID data association.
- `NearestNeighborAssociator`: gated Mahalanobis nearest-neighbor association.

Typical call sequence:

1. `predict(u, dt, control_cov=...)`
2. `update(measurements, measurement_cov=...)`
3. read estimate with `get_state()` / `get_covariance()`.

## Tuning tips (Q/R and gating)

- Start conservative: larger `Q` and `R` reduce overconfidence and divergence.
- `Q` (`process_noise`) should reflect real odometry drift, wheel slip, and timing jitter.
- `R` (`measurement_noise`) should reflect range and bearing noise after outlier filtering.
- If innovations are large but consistent, increase `Q` first (odometry often dominates).
- If map updates are jittery, increase `R`.
- For nearest-neighbor data association, tune `gate_threshold`:
  - Lower gate: safer but more missed associations/new landmarks.
  - Higher gate: fewer misses but more false matches.
- Keep angle units in radians everywhere (`bearing_rad`, yaw, `w`).

## Troubleshooting

### Filter divergence (state/covariance blow-up)

- Increase `Q` and/or `R`; under-modeled noise is the most common cause.
- Verify `dt` and control units match the motion model assumptions.
- Ensure outlier measurements are removed before calling `update(...)`.
- Keep Joseph form enabled (`use_joseph_form=True`) for numerical stability.

### Angle wrap issues

Symptoms: sudden heading jumps near ±π, inconsistent bearing innovations.

- Normalize all angles to `[-pi, pi]` before passing to SLAM.
- Do not subtract raw angles directly without wrapping.
- Ensure sensor code outputs bearing relative to robot heading.

### Poor landmark association

Symptoms: duplicated landmarks, map corruption, unstable corrections.

- With known IDs, ensure IDs are stable over time and unique.
- With nearest-neighbor, tighten `gate_threshold` and improve `R` realism.
- Drop low-quality observations (grazing angles, too far, partial occlusion).
- Prefer initializing landmarks only from geometrically reliable views.

## More docs

- Robot integration guide: `docs/USAGE_IN_ROBOT.md`
- Tuning guide: `docs/CONFIG_TUNING.md`
- Extension guide: `docs/EXTENDING.md`
- Architecture notes: `docs/ARCHITECTURE.md`
