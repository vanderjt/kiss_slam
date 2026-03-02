# 00 — Getting Started

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

## What this step does
This page gets you from a clean machine to your first successful run.

## Plain-English overview
`kiss_slam` is a small 2D SLAM package built around one core idea: keep the pipeline understandable.
You can run it without ROS, with only NumPy/SciPy/Matplotlib and pytest.

## Relevant runnable example
- `examples/00_sanity_check_imports.py`

## Where to look in code
- Package metadata and dependencies: `pyproject.toml`
- Main demo entrypoint: `examples/ekf_slam_demo.py`
- Core API exports: `src/kiss_slam/__init__.py`

## Setup commands
```bash
conda create -n kiss_slam python=3.10 -y
conda activate kiss_slam
pip install -e .
```

Install test tooling:
```bash
pip install -e .[dev]
```

Run tests:
```bash
python -m pytest
```

Run the main demo:
```bash
python examples/ekf_slam_demo.py
```

Run the beginner quick script:
```bash
python examples/00_sanity_check_imports.py
```

## Minimal code example
```python
import numpy as np
from kiss_slam import EKFSLAM, EKFSLAMConfig
from kiss_slam.models.motion import DifferentialDriveMotionModel
from kiss_slam.models.measurement import RangeBearingMeasurementModel
from kiss_slam.types import ControlInput

slam = EKFSLAM(
    motion_model=DifferentialDriveMotionModel(),
    measurement_model=RangeBearingMeasurementModel(),
    config=EKFSLAMConfig(
        process_noise=np.diag([0.05**2, 0.03**2]),
        measurement_noise=np.diag([0.20**2, np.deg2rad(3.0) ** 2]),
    ),
)
slam.predict(ControlInput(v=1.0, w=0.1), dt=0.1)
print(slam.robot_pose())
```

## Try this
1. Change `dt` from `0.1` to `0.2` and compare trajectory smoothness.
2. Increase process noise and observe how covariance ellipses grow in the viewer.

## Common bugs
> **Bug: `ModuleNotFoundError: kiss_slam`**  
> Diagnose: did you run `pip install -e .` inside repo root?  
> Fix: activate env, then reinstall editable package.

> **Bug: no plot window appears**  
> Diagnose: backend may be non-interactive (common over SSH/headless).  
> Fix: set `MPLBACKEND=TkAgg` or use a desktop session.
