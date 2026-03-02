"""Teach: nearest-neighbor gating and failure cases in data association."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

import numpy as np

from kiss_slam.data_association import NearestNeighborAssociator
from kiss_slam.models.measurement import RangeBearingMeasurementModel
from kiss_slam.types import Measurement


def main() -> None:
    assoc = NearestNeighborAssociator(gate_threshold=5.99)
    model = RangeBearingMeasurementModel()

    robot = np.array([0.0, 0.0, 0.0], dtype=float)
    landmark_states = {0: np.array([4.0, 0.5]), 1: np.array([2.0, 3.5]), 2: np.array([-3.0, 2.0])}

    measurements = [
        Measurement(landmark_id=None, range_m=4.1, bearing_rad=0.10),
        Measurement(landmark_id=None, range_m=3.8, bearing_rad=1.05),
        Measurement(landmark_id=None, range_m=10.0, bearing_rad=-2.8),  # expected to fail gate
    ]

    innovation_cov = np.diag([0.25**2, np.deg2rad(3.0) ** 2])
    results = assoc.associate(
        measurements=measurements,
        robot_state=robot,
        landmark_states=landmark_states,
        innovation_cov_fn=lambda _landmark_id: innovation_cov,
        measurement_model=model,
    )

    for i, out in enumerate(results):
        status = "NEW (gated out)" if out.landmark_id is None else f"matched landmark {out.landmark_id}"
        print(f"measurement[{i}] -> {status}")


if __name__ == "__main__":
    main()
