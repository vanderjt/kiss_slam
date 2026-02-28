"""Data association strategies for mapping measurements to landmarks."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from kiss_slam.math_utils import wrap_angle
from kiss_slam.types import Measurement


@dataclass(slots=True)
class AssociatedMeasurement:
    """Result of associating one measurement."""

    measurement: Measurement
    landmark_id: int | None


@dataclass(slots=True)
class KnownCorrespondenceAssociator:
    """Baseline associator that trusts `measurement.landmark_id` if provided."""

    def associate(
        self,
        measurements: list[Measurement],
        known_landmark_ids: set[int],
        **_: object,
    ) -> list[AssociatedMeasurement]:
        """Return measurements with IDs unchanged.

        Unknown IDs are marked as `None` to trigger landmark initialization.
        """
        output: list[AssociatedMeasurement] = []
        for measurement in measurements:
            landmark_id = measurement.landmark_id
            if landmark_id is not None and landmark_id not in known_landmark_ids:
                landmark_id = None
            output.append(AssociatedMeasurement(measurement=measurement, landmark_id=landmark_id))
        return output


@dataclass(slots=True)
class NearestNeighborAssociator:
    """Simple Mahalanobis-gated nearest-neighbor associator.

    TODO: Improve with JCBB or multi-hypothesis tracking for robust clutter handling.
    """

    gate_threshold: float = 5.99  # 95% chi-square for 2 DoF

    def associate(
        self,
        measurements: list[Measurement],
        robot_state: np.ndarray,
        landmark_states: dict[int, np.ndarray],
        landmark_covariances: dict[int, np.ndarray],
        measurement_model,
        measurement_cov: np.ndarray,
    ) -> list[AssociatedMeasurement]:
        """Associate each measurement to nearest landmark under gating."""
        associations: list[AssociatedMeasurement] = []
        for measurement in measurements:
            best_id: int | None = None
            best_score = np.inf
            z = measurement.as_array()

            for landmark_id, landmark_state in landmark_states.items():
                z_pred = measurement_model.predict(robot_state, landmark_state)
                innovation = z - z_pred
                innovation[1] = wrap_angle(innovation[1])
                s = landmark_covariances[landmark_id] + measurement_cov
                score = float(innovation.T @ np.linalg.inv(s) @ innovation)
                if score < best_score:
                    best_score = score
                    best_id = landmark_id

            if best_score > self.gate_threshold:
                best_id = None
            associations.append(AssociatedMeasurement(measurement=measurement, landmark_id=best_id))
        return associations
