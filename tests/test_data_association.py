import numpy as np

from kiss_slam.data_association import KnownCorrespondenceAssociator, NearestNeighborAssociator
from kiss_slam.types import Measurement


def test_known_correspondence_preserves_landmark_id() -> None:
    associator = KnownCorrespondenceAssociator()
    measurements = [Measurement(landmark_id=5, range_m=1.0, bearing_rad=0.1)]
    associated = associator.associate(measurements=measurements, known_landmark_ids={1, 2})
    assert associated[0].landmark_id == 5


def test_nearest_neighbor_rejects_outside_gate() -> None:
    associator = NearestNeighborAssociator(gate_threshold=0.5)
    measurement = Measurement(landmark_id=None, range_m=10.0, bearing_rad=0.0)
    associations = associator.associate(
        measurements=[measurement],
        robot_state=np.array([0.0, 0.0, 0.0]),
        landmark_states={1: np.array([1.0, 0.0])},
        innovation_cov_fn=lambda _lid: np.eye(2),
        measurement_model=type('M', (), {'predict': lambda self, *_: np.array([1.0, 0.0])})(),
    )
    assert associations[0].landmark_id is None
