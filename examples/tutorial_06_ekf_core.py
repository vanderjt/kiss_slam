"""Step 06: compact EKF-SLAM loop without plotting."""

import numpy as np

from kiss_slam import EKFSLAM, EKFSLAMConfig
from kiss_slam.models.measurement import RangeBearingMeasurementModel
from kiss_slam.models.motion import DifferentialDriveMotionModel
from kiss_slam.sim.simulator import SimConfig, Simulator
from kiss_slam.sim.world import World2D


if __name__ == "__main__":
    world = World2D.random(seed=5, n_landmarks=12, xlim=(-10, 10), ylim=(-10, 10))
    sim = Simulator(world=world, config=SimConfig(steps=30), seed=5)
    slam = EKFSLAM(
        motion_model=DifferentialDriveMotionModel(),
        measurement_model=RangeBearingMeasurementModel(),
        config=EKFSLAMConfig(process_noise=sim.control_cov, measurement_noise=sim.measurement_cov),
    )
    for control, dt, measurements, _ in sim.run():
        slam.step(control, dt, measurements)
    pose, landmarks = slam.get_state()
    print("pose:", pose)
    print("landmarks in map:", len(landmarks))
    if slam.nis_values:
        print("mean NIS:", float(np.mean(slam.nis_values)))
