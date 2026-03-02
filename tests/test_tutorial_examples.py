"""Smoke tests for beginner tutorial scripts."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


def run_script(path: str) -> None:
    env = os.environ.copy()
    env["PYTHONPATH"] = str(REPO_ROOT / "src") + os.pathsep + env.get("PYTHONPATH", "")
    subprocess.run([sys.executable, path], cwd=REPO_ROOT, check=True, env=env)


def test_tutorial_cli_smoke_non_visual() -> None:
    scripts = [
        "examples/tutorial_00_minimal_slam.py",
        "examples/tutorial_01_repo_tour.py",
        "examples/tutorial_02_simulator_world.py",
        "examples/tutorial_03_motion_model.py",
        "examples/tutorial_04_measurement_model.py",
        "examples/tutorial_05_data_association.py",
        "examples/tutorial_06_ekf_core.py",
        "examples/tutorial_08_tuning_debugging.py",
        "examples/tutorial_09_robot_integration_stub.py",
    ]
    for script in scripts:
        run_script(script)


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
