"""World generation utilities for 2D landmark simulation.

The world is intentionally lightweight so it can be reused by different
simulators and sensor models.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

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

    @classmethod
    def pattern(
        cls,
        pattern: Literal["grid", "circle", "random"],
        n_landmarks: int,
        xlim: tuple[float, float],
        ylim: tuple[float, float],
        seed: int = 0,
    ) -> "World2D":
        """Create landmarks from a simple named pattern.

        Parameters
        ----------
        pattern:
            One of ``"grid"``, ``"circle"``, or ``"random"``.
        n_landmarks:
            Number of landmarks to generate.
        xlim, ylim:
            World bounds used to place landmarks.
        seed:
            Random seed used for ``"random"`` and to jitter dense ``"grid"``
            patterns.
        """
        if pattern == "random":
            return cls.random(seed=seed, n_landmarks=n_landmarks, xlim=xlim, ylim=ylim)

        if pattern == "circle":
            cx = 0.5 * (xlim[0] + xlim[1])
            cy = 0.5 * (ylim[0] + ylim[1])
            radius = 0.45 * min(xlim[1] - xlim[0], ylim[1] - ylim[0])
            angles = np.linspace(0.0, 2.0 * np.pi, num=n_landmarks, endpoint=False)
            landmarks = [
                Landmark2D(
                    landmark_id=i,
                    x=float(cx + radius * np.cos(angle)),
                    y=float(cy + radius * np.sin(angle)),
                )
                for i, angle in enumerate(angles)
            ]
            return cls(landmarks=landmarks)

        if pattern == "grid":
            cols = int(np.ceil(np.sqrt(n_landmarks)))
            rows = int(np.ceil(n_landmarks / cols))
            xs = np.linspace(xlim[0], xlim[1], num=cols)
            ys = np.linspace(ylim[0], ylim[1], num=rows)

            rng = np.random.default_rng(seed)
            jitter_x = 0.02 * (xlim[1] - xlim[0])
            jitter_y = 0.02 * (ylim[1] - ylim[0])
            landmarks: list[Landmark2D] = []
            landmark_id = 0
            for y in ys:
                for x in xs:
                    if landmark_id >= n_landmarks:
                        break
                    landmarks.append(
                        Landmark2D(
                            landmark_id=landmark_id,
                            x=float(x + rng.uniform(-jitter_x, jitter_x)),
                            y=float(y + rng.uniform(-jitter_y, jitter_y)),
                        )
                    )
                    landmark_id += 1
            return cls(landmarks=landmarks)

        raise ValueError(f"Unsupported landmark pattern: {pattern}")
