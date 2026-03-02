"""Step 01: quick module smoke check for newcomers."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from kiss_slam import EKFSLAM
from kiss_slam.data_association import KnownCorrespondenceAssociator, NearestNeighborAssociator
from kiss_slam.models.measurement import RangeBearingMeasurementModel
from kiss_slam.models.motion import DifferentialDriveMotionModel
from kiss_slam.sim.simulator import SimConfig, Simulator
from kiss_slam.sim.world import World2D


if __name__ == "__main__":
    world = World2D.random(seed=0, n_landmarks=5, xlim=(-2, 2), ylim=(-2, 2))
    sim = Simulator(world=world, config=SimConfig(steps=1), seed=0)
    print("EKF:", EKFSLAM.__name__)
    print("Motion model:", DifferentialDriveMotionModel.__name__)
    print("Measurement model:", RangeBearingMeasurementModel.__name__)
    print("Associators:", KnownCorrespondenceAssociator.__name__, NearestNeighborAssociator.__name__)
    print("Simulator landmarks:", len(sim.world.landmarks))
