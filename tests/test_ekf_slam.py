import numpy as np

from kiss_slam.ekf_slam import EKFSLAM
from kiss_slam.models.measurement import RangeBearingMeasurementModel
from kiss_slam.models.motion import DifferentialDriveMotionModel
from kiss_slam.types import ControlInput, Measurement


def test_ekf_adds_new_landmark() -> None:
    slam = EKFSLAM(
        motion_model=DifferentialDriveMotionModel(),
        measurement_model=RangeBearingMeasurementModel(),
    )
    slam.predict(u=ControlInput(v=0.0, w=0.0), dt=0.1, control_cov=np.diag([1e-3, 1e-3]))
    slam.update(
        measurements=[Measurement(landmark_id=7, range_m=5.0, bearing_rad=0.0)],
        measurement_cov=np.diag([0.1, 0.1]),
    )
    _, landmarks = slam.get_state()
    assert 7 in landmarks
    assert slam.state.shape[0] == 5
    assert slam.get_covariance().shape == (5, 5)


def test_ekf_update_records_nis_for_known_landmark() -> None:
    slam = EKFSLAM(
        motion_model=DifferentialDriveMotionModel(),
        measurement_model=RangeBearingMeasurementModel(),
    )
    cov_u = np.diag([1e-3, 1e-3])
    cov_z = np.diag([0.1, 0.1])

    slam.predict(u=ControlInput(v=0.0, w=0.0), dt=0.1, control_cov=cov_u)
    slam.update([Measurement(landmark_id=1, range_m=2.0, bearing_rad=0.0)], measurement_cov=cov_z)
    slam.update([Measurement(landmark_id=1, range_m=2.1, bearing_rad=0.05)], measurement_cov=cov_z)

    assert len(slam.nis_values) == 1
    assert slam.nis_values[0] >= 0.0
