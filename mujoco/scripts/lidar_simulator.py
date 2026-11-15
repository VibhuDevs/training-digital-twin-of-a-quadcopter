"""Compatibility import for earlier scripts; implementation lives in simulation."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simulation.lidar_simulator import LidarSimulator  # noqa: E402

__all__ = ['LidarSimulator']
