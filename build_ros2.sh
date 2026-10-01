#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
cd "$(dirname "$0")/ros_ws"
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
