"""Step 04: measurement prediction and initialization."""

import numpy as np

from kiss_slam.models.measurement import RangeBearingMeasurementModel
from kiss_slam.types import Measurement


if __name__ == "__main__":
    model = RangeBearingMeasurementModel()
    robot = np.array([0.0, 0.0, 0.1], dtype=float)
    landmark = np.array([5.0, 2.0], dtype=float)
    z_hat = model.predict(robot, landmark)
    print("expected [range,bearing]:", z_hat)
    measurement = Measurement(landmark_id=1, range_m=float(z_hat[0]), bearing_rad=float(z_hat[1]))
    print("initialized landmark:", model.initialize_landmark(robot, measurement))
