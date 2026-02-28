# Repository Tree

```text
kiss_slam/
├── README.md
├── docs/
│   ├── ARCHITECTURE.md
│   └── FILE_TREE.md
├── examples/
│   └── ekf_slam_demo.py
├── pyproject.toml
├── src/
│   └── kiss_slam/
│       ├── __init__.py
│       ├── data_association.py
│       ├── ekf_slam.py
│       ├── math_utils.py
│       ├── models/
│       │   ├── __init__.py
│       │   ├── measurement.py
│       │   └── motion.py
│       ├── sim/
│       │   ├── __init__.py
│       │   ├── simulator.py
│       │   └── world.py
│       ├── types.py
│       └── viz/
│           ├── __init__.py
│           └── live_viewer.py
└── tests/
    ├── test_data_association.py
    ├── test_ekf_slam.py
    ├── test_math_utils.py
    ├── test_models.py
    └── test_simulator.py
```
