import numpy as np

from kiss_slam.models.measurement import RangeBearingMeasurementModel
from kiss_slam.models.motion import DifferentialDriveMotionModel
from kiss_slam.types import ControlInput, Measurement


def test_motion_model_predict_forward() -> None:
    model = DifferentialDriveMotionModel()
    pose = np.array([0.0, 0.0, 0.0])
    predicted = model.predict_state(pose, ControlInput(v=1.0, w=0.0), dt=1.0)
    assert np.allclose(predicted, np.array([1.0, 0.0, 0.0]))


def test_measurement_model_predict() -> None:
    model = RangeBearingMeasurementModel()
    robot = np.array([0.0, 0.0, 0.0])
    landmark = np.array([1.0, 1.0])
    measurement = model.predict(robot, landmark)
    assert np.isclose(measurement[0], np.sqrt(2.0))
    assert np.isclose(measurement[1], np.pi / 4.0)


def test_measurement_landmark_initialization() -> None:
    model = RangeBearingMeasurementModel()
    robot = np.array([1.0, 2.0, 0.0])
    measurement = Measurement(landmark_id=0, range_m=2.0, bearing_rad=0.0)
    landmark = model.initialize_landmark(robot, measurement)
    assert np.allclose(landmark, np.array([3.0, 2.0]))
