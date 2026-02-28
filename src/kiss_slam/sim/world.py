"""World generation utilities for 2D landmark simulation."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from kiss_slam.types import Landmark2D


@dataclass(slots=True)
class World2D:
    """Static 2D landmark world."""

    landmarks: list[Landmark2D]

    @classmethod
    def random(
        cls,
        seed: int,
        n_landmarks: int,
        xlim: tuple[float, float],
        ylim: tuple[float, float],
    ) -> "World2D":
        """Create random landmark world with deterministic seed."""
        rng = np.random.default_rng(seed)
        xs = rng.uniform(xlim[0], xlim[1], size=n_landmarks)
        ys = rng.uniform(ylim[0], ylim[1], size=n_landmarks)
        landmarks = [
            Landmark2D(landmark_id=i, x=float(x_value), y=float(y_value))
            for i, (x_value, y_value) in enumerate(zip(xs, ys))
        ]
        return cls(landmarks=landmarks)
