"""Step 05: known-id vs nearest-neighbor association."""

import numpy as np

from kiss_slam.data_association import KnownCorrespondenceAssociator, NearestNeighborAssociator
from kiss_slam.models.measurement import RangeBearingMeasurementModel
from kiss_slam.types import Measurement


if __name__ == "__main__":
    measurements = [Measurement(landmark_id=10, range_m=5.0, bearing_rad=0.1)]
    known = KnownCorrespondenceAssociator().associate(measurements=measurements, known_landmark_ids={10})
    print("known correspondence ->", known[0].landmark_id)

    nn = NearestNeighborAssociator(gate_threshold=5.99)
    robot = np.array([0.0, 0.0, 0.0], dtype=float)
    landmark_states = {10: np.array([5.0, 0.0], dtype=float)}
    result = nn.associate(
        measurements=measurements,
        robot_state=robot,
        landmark_states=landmark_states,
        innovation_cov_fn=lambda landmark_id: np.diag([0.3, 0.2]),
        measurement_model=RangeBearingMeasurementModel(),
    )
    print("nearest neighbor ->", result[0].landmark_id)
