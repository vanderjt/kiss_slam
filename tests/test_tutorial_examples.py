"""Smoke tests for beginner tutorial scripts."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


def run_script(path: str, *extra: str) -> None:
    env = os.environ.copy()
    env["PYTHONPATH"] = str(REPO_ROOT / "src") + os.pathsep + env.get("PYTHONPATH", "")
    subprocess.run([sys.executable, path, *extra], cwd=REPO_ROOT, check=True, env=env)


def test_progressive_examples_smoke_non_visual() -> None:
    scripts = [
        ("examples/00_sanity_check_imports.py", []),
        ("examples/01_world_generation.py", ["--no-viz"]),
        ("examples/02_simulator_controls.py", []),
        ("examples/03_motion_model_only.py", ["--no-viz", "--steps", "20"]),
        ("examples/04_measurement_model_only.py", ["--no-viz"]),
        ("examples/05_data_association_demo.py", []),
        ("examples/06_ekf_predict_only.py", ["--no-viz", "--steps", "20"]),
        ("examples/07_ekf_update_single_landmark.py", ["--no-viz", "--steps", "20"]),
        ("examples/08_ekf_slam_small_world.py", ["--no-viz", "--steps", "20"]),
        ("examples/09_full_ekf_slam_live.py", ["--no-viz", "--steps", "20"]),
    ]
    for script, args in scripts:
        run_script(script, *args)


def test_beginner_docs_exist() -> None:
    docs = [
        "docs/00_GETTING_STARTED.md",
        "docs/01_REPO_TOUR.md",
        "docs/02_SIMULATOR_AND_WORLD.md",
        "docs/03_MOTION_MODEL.md",
        "docs/04_MEASUREMENT_MODEL.md",
        "docs/05_DATA_ASSOCIATION.md",
        "docs/06_EKF_SLAM_CORE.md",
        "docs/07_VISUALIZATION.md",
        "docs/08_TUNING_AND_DEBUGGING.md",
        "docs/09_USING_IN_REAL_ROBOT.md",
    ]
    for doc in docs:
        assert (REPO_ROOT / doc).exists(), f"Missing {doc}"
