# kiss_slam

`kiss_slam` is a small, readable, and extensible Python package for 2D landmark-based SLAM.
It provides:

- A baseline EKF-SLAM implementation.
- Swappable motion and measurement models.
- Data-association helpers (known correspondences + nearest-neighbor stub).
- A lightweight simulator that generates trajectory, odometry, and range-bearing measurements.
- A Matplotlib live viewer for trajectories, landmarks, and covariance ellipses.

## Install

### Editable install (recommended for development)

```bash
pip install -e .
```

### Install with dev dependencies

```bash
pip install -e .[dev]
```

## Quickstart

```python
from kiss_slam.sim.world import World2D
from kiss_slam.sim.simulator import SimulatorConfig, Simulator2D
from kiss_slam.models.motion import DifferentialDriveMotionModel
from kiss_slam.models.measurement import RangeBearingMeasurementModel
from kiss_slam.ekf_slam import EKFSLAM

world = World2D.random(seed=0, n_landmarks=10, xlim=(-20, 20), ylim=(-20, 20))
sim = Simulator2D(
    world=world,
    config=SimulatorConfig(dt=0.1, steps=50, sensor_range=15.0),
)
motion = DifferentialDriveMotionModel()
measurement = RangeBearingMeasurementModel()
slam = EKFSLAM(motion_model=motion, measurement_model=measurement)

for step in sim.run():
    slam.predict(control=step.control, control_cov=sim.control_cov, dt=sim.config.dt)
    slam.update(measurements=step.measurements, measurement_cov=sim.measurement_cov)
```

## Run the demo

```bash
python examples/ekf_slam_demo.py
```

The demo creates a random world, simulates a robot run, executes EKF-SLAM,
and displays a live plot with true/estimated pose, trajectories, landmarks, and covariance ellipses.

## Testing

```bash
pytest
```

## Project layout

- `src/kiss_slam/`: library package code.
- `examples/`: runnable demos.
- `docs/`: architecture and design notes.
- `tests/`: unit tests with `pytest`.

## Design notes

The package follows the KISS principle:

- Small classes with single responsibilities.
- Clear, explicit type hints and docstrings.
- Extension points documented in `docs/ARCHITECTURE.md`.
