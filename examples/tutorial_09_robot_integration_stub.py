"""Step 09: skeleton for integrating real robot logs."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

import numpy as np

from kiss_slam import EKFSLAM, EKFSLAMConfig
from kiss_slam.models.measurement import RangeBearingMeasurementModel
from kiss_slam.models.motion import DifferentialDriveMotionModel
from kiss_slam.types import ControlInput, Measurement


if __name__ == "__main__":
    slam = EKFSLAM(
        motion_model=DifferentialDriveMotionModel(),
        measurement_model=RangeBearingMeasurementModel(),
        config=EKFSLAMConfig(
            process_noise=np.diag([0.08**2, 0.05**2]),
            measurement_noise=np.diag([0.25**2, np.deg2rad(4.0) ** 2]),
        ),
    )

    # Replace this with real sensor pipeline or recorded logs.
    fake_stream = [
        (0.1, ControlInput(0.8, 0.02), [Measurement(landmark_id=1, range_m=4.0, bearing_rad=0.1)]),
        (0.1, ControlInput(0.8, 0.00), [Measurement(landmark_id=1, range_m=3.9, bearing_rad=0.09)]),
    ]

    for dt, control, measurements in fake_stream:
        slam.predict(control, dt)
        slam.update(measurements)

    pose, landmarks = slam.get_state()
    print("final pose:", pose)
    print("mapped landmarks:", landmarks)
