# Architecture Overview

## Core concepts and data flow

`kiss_slam` is organized around a simulation-to-estimation pipeline:

1. **World generation** (`World2D`): static landmark map.
2. **Simulation** (`Simulator2D`): generates controls, noisy odometry, and noisy landmark measurements.
3. **SLAM estimation** (`EKFSLAM`):
   - Predict robot state using a motion model.
   - Associate measurements to landmarks.
   - Update robot + landmark state with EKF equations.
4. **Visualization** (`LiveViewer`): renders truth and estimates.

Data types (`types.py`) carry state and observations between modules using explicit dataclasses.

## Major classes and responsibilities

- `Pose2D`, `Landmark2D`, `Measurement`, `ControlInput`, `SimulationStep`:
  typed containers used throughout the package.
- `DifferentialDriveMotionModel`:
  computes predicted robot state and Jacobians from control input.
- `RangeBearingMeasurementModel`:
  predicts range/bearing observations and Jacobians for a landmark.
- `KnownCorrespondenceAssociator`:
  baseline data association using known landmark IDs.
- `NearestNeighborAssociator`:
  extension hook for ID-free nearest-neighbor matching with gating.
- `EKFSLAM`:
  maintains joint Gaussian state `[x, y, yaw, l1x, l1y, ...]` and covariance.
- `World2D` and `Simulator2D`:
  synthetic environment and data generator for end-to-end examples/tests.
- `LiveViewer`:
  plotting helper for trajectories, landmarks, and covariance ellipses.

## Extending models and data association

### Motion model extension

Implement a class exposing:

- `predict_state(pose: np.ndarray, control: ControlInput, dt: float) -> np.ndarray`
- `jacobians(pose: np.ndarray, control: ControlInput, dt: float) -> tuple[np.ndarray, np.ndarray]`

Then pass it into `EKFSLAM(motion_model=...)`.

### Measurement model extension

Implement:

- `predict(robot_state: np.ndarray, landmark_state: np.ndarray) -> np.ndarray`
- `jacobians(robot_state: np.ndarray, landmark_state: np.ndarray) -> tuple[np.ndarray, np.ndarray]`
- `initialize_landmark(robot_state: np.ndarray, measurement: Measurement) -> np.ndarray`

Use it with `EKFSLAM(measurement_model=...)`.

### Data association extension

Implement an associator with:

- `associate(measurements, slam_state, measurement_model, measurement_cov)`

The baseline `KnownCorrespondenceAssociator` trusts provided IDs.
`NearestNeighborAssociator` currently offers a simple Mahalanobis-gated match and can be
expanded with JCBB or multi-hypothesis approaches.

## Notes

- EKF math stays intentionally explicit and local for readability.
- TODO markers indicate planned additions (metrics logging, advanced association, and richer diagnostics).
