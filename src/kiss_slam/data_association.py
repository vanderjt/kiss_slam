"""Data association strategies for mapping measurements to landmarks."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from kiss_slam.math_utils import mahalanobis_distance, wrap_angle
from kiss_slam.types import Measurement


@dataclass(slots=True)
class AssociatedMeasurement:
    """Association output for one input measurement."""

    measurement: Measurement
    landmark_id: int | None


@dataclass(slots=True)
class KnownCorrespondenceAssociator:
    """Baseline associator using simulator-provided landmark IDs."""

    def associate(
        self,
        measurements: list[Measurement],
        known_landmark_ids: set[int],
        **_: object,
    ) -> list[AssociatedMeasurement]:
        associations: list[AssociatedMeasurement] = []
        for measurement in measurements:
            associations.append(AssociatedMeasurement(measurement=measurement, landmark_id=measurement.landmark_id))
        return associations


@dataclass(slots=True)
class NearestNeighborAssociator:
    """Mahalanobis-gated nearest-neighbor association.

    If a measurement is outside the configured gate for all known landmarks, the
    associator returns ``None`` so the caller can initialize a new landmark.
    """

    gate_threshold: float = 5.99

    def associate(
        self,
        measurements: list[Measurement],
        robot_state: np.ndarray,
        landmark_states: dict[int, np.ndarray],
        innovation_cov_fn,
        measurement_model,
        **_: object,
    ) -> list[AssociatedMeasurement]:
        associations: list[AssociatedMeasurement] = []
        for measurement in measurements:
            if not landmark_states:
                associations.append(AssociatedMeasurement(measurement=measurement, landmark_id=None))
                continue

            z = measurement.as_array()
            best_id: int | None = None
            best_distance = np.inf

            for landmark_id, landmark_state in landmark_states.items():
                z_pred = measurement_model.predict(robot_state, landmark_state)
                innovation = z - z_pred
                innovation[1] = wrap_angle(innovation[1])
                s = innovation_cov_fn(landmark_id)
                distance = mahalanobis_distance(innovation, s)
                if distance < best_distance:
                    best_distance = distance
                    best_id = landmark_id

            if best_distance > self.gate_threshold:
                best_id = None
            associations.append(AssociatedMeasurement(measurement=measurement, landmark_id=best_id))

        return associations
