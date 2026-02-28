from __future__ import annotations

import numpy as np

from kiss_slam.data_association import NearestNeighborAssociator
from kiss_slam.ekf_slam import EKFSLAM
from kiss_slam.math_utils import numerical_jacobian, wrap_angle
from kiss_slam.models.measurement import (
    RangeBearingMeasurementModel,
    init_landmark_jacobians,
    initialize_landmark_from_measurement,
    range_bearing_jacobians,
)
from kiss_slam.models.motion import DifferentialDriveMotionModel, unicycle_jacobians, unicycle_predict_state
from kiss_slam.types import ControlInput, EKFSLAMConfig, Measurement, Pose2D


def test_wrap_angle_known_boundaries() -> None:
    test_angles = np.array([-4.0 * np.pi, -np.pi, -0.1, 0.1, np.pi, 3.0 * np.pi])
    wrapped = np.array([wrap_angle(angle) for angle in test_angles])
    assert np.all(wrapped >= -np.pi)
    assert np.all(wrapped < np.pi)
    assert np.isclose(wrapped[1], -np.pi)
    assert np.isclose(wrapped[4], -np.pi)


def test_motion_jacobian_matches_finite_difference_state() -> None:
    pose = np.array([1.2, -0.7, 0.4])
    control = ControlInput(v=1.3, w=-0.5)
    dt = 0.2

    fx_analytic, _ = unicycle_jacobians(pose=pose, control=control, dt=dt)

    fx_numeric = numerical_jacobian(
        lambda state: unicycle_predict_state(state, control=control, dt=dt),
        pose,
    )
    assert np.allclose(fx_analytic, fx_numeric, atol=1e-6)


def test_motion_jacobian_matches_finite_difference_control() -> None:
    pose = np.array([-0.3, 2.1, -0.6])
    control = ControlInput(v=0.8, w=0.2)
    dt = 0.15

    _, fu_analytic = unicycle_jacobians(pose=pose, control=control, dt=dt)

    def propagate_u(u_vec: np.ndarray) -> np.ndarray:
        control_local = ControlInput(v=float(u_vec[0]), w=float(u_vec[1]))
        return unicycle_predict_state(pose=pose, control=control_local, dt=dt)

    fu_numeric = numerical_jacobian(propagate_u, np.array([control.v, control.w]))
    assert np.allclose(fu_analytic, fu_numeric, atol=1e-6)


def test_measurement_jacobians_match_finite_difference() -> None:
    model = RangeBearingMeasurementModel()
    robot = np.array([0.8, -0.4, 0.2])
    landmark = np.array([4.1, 1.2])

    hr_analytic, hl_analytic = range_bearing_jacobians(robot, landmark)
    hr_numeric = numerical_jacobian(lambda x: model.predict(robot_state=x, landmark_state=landmark), robot)
    hl_numeric = numerical_jacobian(lambda l: model.predict(robot_state=robot, landmark_state=l), landmark)

    assert np.allclose(hr_analytic, hr_numeric, atol=1e-6)
    assert np.allclose(hl_analytic, hl_numeric, atol=1e-6)


def test_landmark_initialization_and_jacobians() -> None:
    robot = np.array([2.0, -1.0, np.deg2rad(30.0)])
    measurement = Measurement(landmark_id=3, range_m=4.0, bearing_rad=np.deg2rad(-15.0))

    landmark = initialize_landmark_from_measurement(robot_state=robot, measurement=measurement)
    expected_theta = robot[2] + measurement.bearing_rad
    expected = np.array(
        [robot[0] + measurement.range_m * np.cos(expected_theta), robot[1] + measurement.range_m * np.sin(expected_theta)]
    )
    assert np.allclose(landmark, expected)

    gx_analytic, gz_analytic = init_landmark_jacobians(robot, measurement)
    gx_numeric = numerical_jacobian(lambda x: initialize_landmark_from_measurement(robot_state=x, measurement=measurement), robot)
    gz_numeric = numerical_jacobian(
        lambda z: initialize_landmark_from_measurement(
            robot_state=robot,
            measurement=Measurement(landmark_id=3, range_m=float(z[0]), bearing_rad=float(z[1])),
        ),
        np.array([measurement.range_m, measurement.bearing_rad]),
    )

    assert np.allclose(gx_analytic, gx_numeric, atol=1e-6)
    assert np.allclose(gz_analytic, gz_numeric, atol=1e-6)


def test_predict_update_invariants_and_no_nans(deterministic_simulator) -> None:
    slam = EKFSLAM(
        motion_model=DifferentialDriveMotionModel(),
        measurement_model=RangeBearingMeasurementModel(),
    )

    for _ in range(20):
        u, dt, measurements, _ = deterministic_simulator.step()
        slam.predict(u=u, dt=dt, control_cov=deterministic_simulator.control_cov)
        slam.update(measurements=measurements, measurement_cov=deterministic_simulator.measurement_cov)

        n = slam.state.size
        assert slam.covariance.shape == (n, n)
        assert np.all(np.isfinite(slam.state))
        assert np.all(np.isfinite(slam.covariance))


def test_covariance_is_symmetric_and_psd_with_tolerance(deterministic_simulator) -> None:
    slam = EKFSLAM(
        motion_model=DifferentialDriveMotionModel(),
        measurement_model=RangeBearingMeasurementModel(),
    )

    for _ in range(30):
        u, dt, measurements, _ = deterministic_simulator.step()
        slam.predict(u=u, dt=dt, control_cov=deterministic_simulator.control_cov)
        slam.update(measurements=measurements, measurement_cov=deterministic_simulator.measurement_cov)

    covariance = slam.get_covariance()
    assert np.allclose(covariance, covariance.T, atol=1e-8)
    eigvals = np.linalg.eigvalsh(covariance)
    assert np.min(eigvals) > -1e-8


def test_nearest_neighbor_gating_rejects_outlier() -> None:
    associator = NearestNeighborAssociator(gate_threshold=3.0)
    measurement_model = RangeBearingMeasurementModel()
    robot = np.array([0.0, 0.0, 0.0])
    landmarks = {0: np.array([2.0, 0.0]), 1: np.array([2.0, 2.0])}
    outlier = Measurement(landmark_id=None, range_m=8.0, bearing_rad=np.deg2rad(140.0))

    associated = associator.associate(
        measurements=[outlier],
        robot_state=robot,
        landmark_states=landmarks,
        innovation_cov_fn=lambda _lid: np.diag([0.05**2, np.deg2rad(2.0) ** 2]),
        measurement_model=measurement_model,
    )
    assert associated[0].landmark_id is None


def test_nearest_neighbor_gating_accepts_nearby_measurement() -> None:
    associator = NearestNeighborAssociator(gate_threshold=20.0)
    measurement_model = RangeBearingMeasurementModel()
    robot = np.array([0.0, 0.0, 0.0])
    landmarks = {5: np.array([3.0, 0.0]), 9: np.array([3.0, 2.0])}
    measurement = Measurement(landmark_id=None, range_m=3.05, bearing_rad=np.deg2rad(1.0))

    associated = associator.associate(
        measurements=[measurement],
        robot_state=robot,
        landmark_states=landmarks,
        innovation_cov_fn=lambda _lid: np.diag([0.1**2, np.deg2rad(3.0) ** 2]),
        measurement_model=measurement_model,
    )
    assert associated[0].landmark_id == 5


def test_rmse_decreases_in_simple_seeded_scenario(deterministic_simulator) -> None:
    slam = EKFSLAM(
        motion_model=DifferentialDriveMotionModel(),
        measurement_model=RangeBearingMeasurementModel(),
        config=EKFSLAMConfig(
            initial_pose=Pose2D(1.5, -1.0, 0.35),
            initial_pose_cov=np.diag([0.4, 0.4, 0.2]),
            process_noise=np.diag([0.02**2, 0.02**2]),
            measurement_noise=deterministic_simulator.measurement_cov,
        ),
    )

    errors = []
    for _ in range(60):
        u, dt, measurements, true_pose = deterministic_simulator.step()
        slam.predict(u=u, dt=dt, control_cov=deterministic_simulator.control_cov)
        slam.update(measurements=measurements, measurement_cov=deterministic_simulator.measurement_cov)

        est = slam.robot_pose()
        err = np.array(
            [
                est[0] - true_pose.x,
                est[1] - true_pose.y,
                wrap_angle(est[2] - true_pose.yaw),
            ]
        )
        errors.append(np.linalg.norm(err))

    early_rmse = float(np.sqrt(np.mean(np.square(errors[:20]))))
    late_rmse = float(np.sqrt(np.mean(np.square(errors[-20:]))))
    assert late_rmse < early_rmse


def test_state_size_growth_matches_number_of_initialized_landmarks() -> None:
    slam = EKFSLAM(
        motion_model=DifferentialDriveMotionModel(),
        measurement_model=RangeBearingMeasurementModel(),
    )

    slam.update(
        measurements=[
            Measurement(landmark_id=0, range_m=2.0, bearing_rad=0.1),
            Measurement(landmark_id=1, range_m=3.0, bearing_rad=-0.2),
            Measurement(landmark_id=2, range_m=4.0, bearing_rad=0.3),
        ],
        measurement_cov=np.diag([0.1, 0.05]),
    )

    assert slam.state.size == 3 + 2 * 3
    assert len(slam.landmark_states()) == 3
