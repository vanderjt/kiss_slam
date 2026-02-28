from __future__ import annotations

import numpy as np
import pytest

from kiss_slam.sim.simulator import SimConfig, Simulator
from kiss_slam.sim.world import World2D


@pytest.fixture
def deterministic_simulator() -> Simulator:
    """Small seeded simulator fixture used for stable EKF regression tests."""
    world = World2D.pattern(
        pattern="circle",
        n_landmarks=16,
        xlim=(-10.0, 10.0),
        ylim=(-10.0, 10.0),
        seed=123,
    )
    config = SimConfig(
        dt=0.1,
        steps=80,
        trajectory_mode="figure_eight",
        sensor_range=20.0,
        fov_rad=2.5,
        nominal_v=1.0,
        nominal_w=0.7,
        control_noise_std=(0.02, 0.01),
        measurement_noise_std=(0.08, np.deg2rad(1.5)),
        measurement_dropout_prob=0.0,
    )
    return Simulator(world=world, config=config, seed=7)
