"""Step 07: run main visual demo."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from ekf_slam_demo import main


if __name__ == "__main__":
    main()
