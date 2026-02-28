# Using kiss_slam in a robot loop

This guide shows the minimum pattern to connect `kiss_slam` to real odometry and landmark detections.

## 1) Feed real odometry

`EKFSLAM.predict(...)` expects a `ControlInput(v, w)` and `dt`.

```python
from kiss_slam.types import ControlInput

# Example conversion from wheel odometry / fused velocity estimate.
v_mps = odom_msg.linear_velocity_mps
w_radps = odom_msg.yaw_rate_radps
u = ControlInput(v=v_mps, w=w_radps)

slam.predict(u=u, dt=dt_s, control_cov=Q)
```

Notes:
- `v` in m/s, `w` in rad/s.
- `dt` must match the odometry timestamp delta.
- `Q` is 2x2 covariance for `[v, w]`.

## 2) Feed landmark observations

`EKFSLAM.update(...)` expects `list[Measurement]`.

```python
from kiss_slam.types import Measurement

measurements = []
for det in landmark_detections:
    measurements.append(
        Measurement(
            landmark_id=det.id,          # use None if ID is unknown
            range_m=det.range_m,
            bearing_rad=det.bearing_rad, # relative to robot heading
        )
    )

slam.update(measurements=measurements, measurement_cov=R)
```

Notes:
- Bearings must be wrapped to `[-pi, pi]`.
- `R` is 2x2 covariance for `[range, bearing]`.
- In unknown-correspondence mode, set `landmark_id=None` and use a custom associator.

## 3) Run predict/update in a loop

```python
import numpy as np

from kiss_slam import EKFSLAM, EKFSLAMConfig
from kiss_slam.models.motion import DifferentialDriveMotionModel
from kiss_slam.models.measurement import RangeBearingMeasurementModel

Q = np.diag([0.05**2, 0.03**2])
R = np.diag([0.20**2, np.deg2rad(3.0) ** 2])

slam = EKFSLAM(
    motion_model=DifferentialDriveMotionModel(),
    measurement_model=RangeBearingMeasurementModel(),
    config=EKFSLAMConfig(process_noise=Q, measurement_noise=R),
)

while robot_is_running:
    dt = clock.seconds_since_last_tick()
    odom = read_odometry()          # provides v, w
    landmarks = read_landmarks()    # provides id/range/bearing

    slam.predict(u=ControlInput(v=odom.v, w=odom.w), dt=dt)
    slam.update(
        measurements=[
            Measurement(d.id, d.range_m, d.bearing_rad)
            for d in landmarks
        ]
    )

    pose_est, landmark_est = slam.get_state()
```

## 4) Log outputs for debugging and evaluation

Recommended per-step logs:
- timestamp
- estimated pose (`slam.robot_pose()`)
- full covariance diagonal (`np.diag(slam.get_covariance())`)
- number of tracked landmarks
- innovation/NIS stats (`slam.nis_values`)

```python
import csv
import numpy as np

with open("slam_log.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["t", "x", "y", "yaw", "n_landmarks", "trace_sigma"])

    # inside runtime loop
    pose = slam.robot_pose()
    sigma = slam.get_covariance()
    writer.writerow([
        t,
        float(pose[0]), float(pose[1]), float(pose[2]),
        len(slam.landmark_states()),
        float(np.trace(sigma[:3, :3])),
    ])
```

If you have ground truth, compute RMSE and compare innovation/NIS trends across runs.
