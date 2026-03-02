"""Small shared helpers for runnable tutorial examples."""

from __future__ import annotations

import argparse
from typing import Iterable

import matplotlib.pyplot as plt
import numpy as np


def make_parser(description: str, *, default_steps: int = 100, default_seed: int = 0) -> argparse.ArgumentParser:
    """Create a minimal parser with common `--steps`, `--seed`, and `--no-viz` flags."""
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--steps", type=int, default=default_steps, help="Number of simulation/filter steps.")
    parser.add_argument("--seed", type=int, default=default_seed, help="Random seed.")
    parser.add_argument("--no-viz", action="store_true", help="Disable plotting windows.")
    return parser


def finalize_plot(no_viz: bool, title: str | None = None) -> None:
    """Show plot briefly in interactive mode; no-op when visualization is disabled."""
    if no_viz:
        plt.close("all")
        return
    if title:
        plt.gcf().suptitle(title)
    plt.tight_layout()
    plt.show(block=False)
    plt.pause(1.0)
    plt.close("all")


def landmarks_to_array(landmarks: Iterable[object]) -> np.ndarray:
    """Convert a sequence of landmark objects with `x`/`y` fields into an `(N,2)` array."""
    return np.array([[float(lm.x), float(lm.y)] for lm in landmarks], dtype=float)
