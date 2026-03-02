# kiss_slam

`kiss_slam` is a small, readable Python package for 2D landmark EKF-SLAM.
It is designed for non-ROS robotics projects (Python 3.10+), with simulator + visualization included.

## Install

```bash
pip install -e .
```

Development install:

```bash
pip install -e .[dev]
```

## Quickstart

Run the full visual demo:

```bash
python examples/ekf_slam_demo.py
```

Run the beginner step-0 script:

```bash
python examples/tutorial_00_minimal_slam.py
```

Minimal API usage:

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
slam.predict(u=ControlInput(v=1.0, w=0.1), dt=0.1)
print(slam.robot_pose())
```

## Beginner docs sequence

1. [docs/00_GETTING_STARTED.md](docs/00_GETTING_STARTED.md)
2. [docs/01_REPO_TOUR.md](docs/01_REPO_TOUR.md)
3. [docs/02_SIMULATOR_AND_WORLD.md](docs/02_SIMULATOR_AND_WORLD.md)
4. [docs/03_MOTION_MODEL.md](docs/03_MOTION_MODEL.md)
5. [docs/04_MEASUREMENT_MODEL.md](docs/04_MEASUREMENT_MODEL.md)
6. [docs/05_DATA_ASSOCIATION.md](docs/05_DATA_ASSOCIATION.md)
7. [docs/06_EKF_SLAM_CORE.md](docs/06_EKF_SLAM_CORE.md)
8. [docs/07_VISUALIZATION.md](docs/07_VISUALIZATION.md)
9. [docs/08_TUNING_AND_DEBUGGING.md](docs/08_TUNING_AND_DEBUGGING.md)
10. [docs/09_USING_IN_REAL_ROBOT.md](docs/09_USING_IN_REAL_ROBOT.md)

## Tutorial examples

```bash
python examples/tutorial_01_repo_tour.py
python examples/tutorial_02_simulator_world.py
python examples/tutorial_03_motion_model.py
python examples/tutorial_04_measurement_model.py
python examples/tutorial_05_data_association.py
python examples/tutorial_06_ekf_core.py
python examples/tutorial_07_visualization.py
python examples/tutorial_08_tuning_debugging.py
python examples/tutorial_09_robot_integration_stub.py
```

## Notebook

Launch Jupyter and open the walkthrough notebook:

```bash
jupyter notebook examples/ekf_slam_walkthrough.ipynb
```

If `jupyter` is missing, install it in your conda environment:

```bash
conda install jupyter -y
```

## Testing

```bash
python -m pytest
```

## Troubleshooting

### Matplotlib backend issues
- Symptom: no interactive window appears.
- Try:
  ```bash
  MPLBACKEND=TkAgg python examples/ekf_slam_demo.py
  ```
- On some Linux installs you may need:
  ```bash
  conda install tk -y
  ```

### Conda environment confusion
- Ensure the active interpreter matches your env:
  ```bash
  which python
  python --version
  pip -V
  ```
- Reinstall package in active env:
  ```bash
  pip install -e .
  ```

### Import errors
- Run commands from repo root (`kiss_slam/`).
- Confirm editable install completed without errors.

## Additional docs

- `docs/USAGE_IN_ROBOT.md`
- `docs/CONFIG_TUNING.md`
- `docs/EXTENDING.md`
- `docs/ARCHITECTURE.md`
