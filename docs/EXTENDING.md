# Extending kiss_slam

This package is intentionally composition-based: pass custom model/association objects into `EKFSLAM`.

## Add a new motion model

A motion model must provide:
- `predict_state(pose: np.ndarray, control: ControlInput, dt: float) -> np.ndarray`
- `jacobians(pose: np.ndarray, control: ControlInput, dt: float) -> tuple[np.ndarray, np.ndarray]`

```python
import numpy as np
from kiss_slam.types import ControlInput

class MyMotionModel:
    def predict_state(self, pose: np.ndarray, control: ControlInput, dt: float) -> np.ndarray:
        # Replace with your kinematics
        x, y, yaw = pose
        return np.array([x + control.v * dt, y, yaw + control.w * dt], dtype=float)

    def jacobians(self, pose: np.ndarray, control: ControlInput, dt: float):
        Fx = np.eye(3)
        Fu = np.array([[dt, 0.0], [0.0, 0.0], [0.0, dt]], dtype=float)
        return Fx, Fu
```

Then instantiate:

```python
slam = EKFSLAM(motion_model=MyMotionModel(), measurement_model=RangeBearingMeasurementModel())
```

## Add a new sensor model

A measurement model must provide:
- `predict(robot_state, landmark_state) -> np.ndarray`
- `jacobians(robot_state, landmark_state) -> tuple[np.ndarray, np.ndarray]`
- `initialize_landmark(robot_state, measurement) -> np.ndarray`
- `initialization_jacobians(robot_state, measurement) -> tuple[np.ndarray, np.ndarray]`

For non-range-bearing sensors, keep the measurement vector and Jacobian dimensions consistent everywhere (including `R`).

```python
class MyMeasurementModel:
    def predict(self, robot_state, landmark_state):
        ...

    def jacobians(self, robot_state, landmark_state):
        # Hr shape: (m, 3), Hl shape: (m, 2)
        ...

    def initialize_landmark(self, robot_state, measurement):
        ...

    def initialization_jacobians(self, robot_state, measurement):
        ...
```

## Replace data association

Associators are plug-ins with an `associate(...)` method returning a list of:
`AssociatedMeasurement(measurement=<Measurement>, landmark_id=<int|None>)`.

Use existing examples:
- `KnownCorrespondenceAssociator`
- `NearestNeighborAssociator`

Minimal skeleton:

```python
from kiss_slam.data_association import AssociatedMeasurement

class MyAssociator:
    def associate(self, measurements, known_landmark_ids, **kwargs):
        out = []
        for m in measurements:
            # decide matched_id or None (None => initialize new landmark)
            matched_id = m.landmark_id if m.landmark_id in known_landmark_ids else None
            out.append(AssociatedMeasurement(measurement=m, landmark_id=matched_id))
        return out
```

Then pass it in:

```python
slam = EKFSLAM(
    motion_model=DifferentialDriveMotionModel(),
    measurement_model=RangeBearingMeasurementModel(),
    assoc=MyAssociator(),
)
```

## Practical extension tips

- Keep interfaces small and testable.
- Verify Jacobians with finite-difference checks before large runs.
- Add focused unit tests in `tests/` for each new model/associator.
- Start with known correspondences before attempting unknown association.

## Linting and formatting recommendations

For local code hygiene, run lint and formatting checks before committing:

```bash
ruff check src tests examples
black src tests examples
```

These checks are currently advisory, but strongly recommended to keep contributions readable and consistent.
