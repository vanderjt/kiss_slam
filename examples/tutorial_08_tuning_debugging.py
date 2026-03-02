"""Step 08: compare simple RMSE under two noise settings."""

import numpy as np

from kiss_slam import EKFSLAM, EKFSLAMConfig
from kiss_slam.models.measurement import RangeBearingMeasurementModel
from kiss_slam.models.motion import DifferentialDriveMotionModel
from kiss_slam.sim.simulator import SimConfig, Simulator
from kiss_slam.sim.world import World2D


def run_with_scale(scale_q: float, scale_r: float) -> float:
    world = World2D.random(seed=9, n_landmarks=20, xlim=(-12, 12), ylim=(-12, 12))
    sim = Simulator(world=world, config=SimConfig(steps=80), seed=9)
    slam = EKFSLAM(
        motion_model=DifferentialDriveMotionModel(),
        measurement_model=RangeBearingMeasurementModel(),
        config=EKFSLAMConfig(process_noise=sim.control_cov * scale_q, measurement_noise=sim.measurement_cov * scale_r),
    )
    errors = []
    for control, dt, measurements, gt in sim.run():
        slam.step(control, dt, measurements)
        est = slam.robot_pose()
        errors.append((est[0] - gt.x) ** 2 + (est[1] - gt.y) ** 2)
    return float(np.sqrt(np.mean(errors)))


if __name__ == "__main__":
    rmse_default = run_with_scale(1.0, 1.0)
    rmse_conservative = run_with_scale(4.0, 2.0)
    print(f"RMSE default: {rmse_default:.3f} m")
    print(f"RMSE conservative: {rmse_conservative:.3f} m")
