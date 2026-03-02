"""Step 03: motion model equations and Jacobians."""

import numpy as np

from kiss_slam.models.motion import DifferentialDriveMotionModel
from kiss_slam.types import ControlInput


if __name__ == "__main__":
    model = DifferentialDriveMotionModel()
    pose = np.array([0.0, 0.0, 0.2], dtype=float)
    control = ControlInput(v=1.0, w=0.3)
    dt = 0.1
    print("predicted pose:", model.predict_state(pose, control, dt))
    fx, fu = model.jacobians(pose, control, dt)
    print("Fx:\n", fx)
    print("Fu:\n", fu)
