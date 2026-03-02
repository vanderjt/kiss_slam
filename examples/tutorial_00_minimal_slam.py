"""Step 00: minimal EKF-SLAM usage."""

from __future__ import annotations

import numpy as np

from kiss_slam import EKFSLAM, EKFSLAMConfig
from kiss_slam.models.measurement import RangeBearingMeasurementModel
from kiss_slam.models.motion import DifferentialDriveMotionModel
from kiss_slam.types import ControlInput


if __name__ == "__main__":
    slam = EKFSLAM(
        motion_model=DifferentialDriveMotionModel(),
        measurement_model=RangeBearingMeasurementModel(),
        config=EKFSLAMConfig(
            process_noise=np.diag([0.05**2, 0.03**2]),
            measurement_noise=np.diag([0.2**2, np.deg2rad(3.0) ** 2]),
        ),
    )
    slam.predict(u=ControlInput(v=1.0, w=0.1), dt=0.1)
    print("Estimated pose:", slam.robot_pose())
