"""Portable tests use explicit ROS doubles; live DDS checks run separately.

See tests/live_pipeline.py, which imports genuine rclpy and never this file.
"""
from pathlib import Path
import sys

ROOT=Path(__file__).parents[1]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/'tests'))
for package in (ROOT/'ros_ws/src').iterdir():
    if package.is_dir():sys.path.insert(0,str(package))
import ros_sim
ros_sim.install()
