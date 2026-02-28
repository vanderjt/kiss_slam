from kiss_slam.data_association import KnownCorrespondenceAssociator
from kiss_slam.types import Measurement


def test_known_correspondence_unknown_id_becomes_none() -> None:
    associator = KnownCorrespondenceAssociator()
    measurements = [Measurement(landmark_id=5, range_m=1.0, bearing_rad=0.1)]
    associated = associator.associate(measurements=measurements, known_landmark_ids={1, 2})
    assert associated[0].landmark_id is None
