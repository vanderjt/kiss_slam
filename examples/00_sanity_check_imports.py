"""Teach: verify imports and package versions before touching SLAM logic."""

from __future__ import annotations

import platform

import matplotlib
import numpy as np
import scipy

import kiss_slam


def main() -> None:
    print("kiss_slam sanity check")
    print(f"Python: {platform.python_version()}")
    print(f"NumPy: {np.__version__}")
    print(f"SciPy: {scipy.__version__}")
    print(f"Matplotlib: {matplotlib.__version__}")
    print(f"kiss_slam: {getattr(kiss_slam, '__version__', 'dev')}")
    print("Environment looks ready ✅")


if __name__ == "__main__":
    main()
