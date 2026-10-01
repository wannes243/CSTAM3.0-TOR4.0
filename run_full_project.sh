#!/usr/bin/env bash
set -Ee pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORLD_FILE="${FABTINO_WORLD:-$PROJECT_ROOT/webots/worlds/world_simulation.wbt}"
VIEWER_URL="http://127.0.0.1:8000/map_viewer2_modular.html"
LOG_DIR="$PROJECT_ROOT/logs/full_project_$(date +%Y%m%d_%H%M%S)"

mkdir -p "$LOG_DIR"

# ROS 2 commands are normally added to PATH by the setup files, so source them
# before checking for required executables.
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "ERROR: ROS 2 Humble setup file not found: /opt/ros/humble/setup.bash" >&2
    exit 1
fi

source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/ros_ws/install/setup.bash"
set -u

for command_name in ros2 webots python3; do
    if ! command -v "$command_name" >/dev/null 2>&1; then
        echo "ERROR: required command not found: $command_name" >&2
        exit 1
    fi
done

if [[ ! -f "$WORLD_FILE" ]]; then
    echo "ERROR: Webots world not found: $WORLD_FILE" >&2
    exit 1
fi

ros_pid=""
webots_pid=""
viewer_pid=""

cleanup() {
    trap - EXIT INT TERM
    echo
    echo "Stopping Fabtino project..."

    if [[ -n "$viewer_pid" ]] && kill -0 "$viewer_pid" 2>/dev/null; then
        kill -- "-$viewer_pid" 2>/dev/null || true
    fi
    if [[ -n "$webots_pid" ]] && kill -0 "$webots_pid" 2>/dev/null; then
        kill -- "-$webots_pid" 2>/dev/null || true
    fi
    if [[ -n "$ros_pid" ]] && kill -0 "$ros_pid" 2>/dev/null; then
        kill -- "-$ros_pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

echo "Starting ROS 2 nodes..."
setsid ros2 launch fabtino_bringup fabtino_sim.launch.py \
    >"$LOG_DIR/ros2.log" 2>&1 &
ros_pid=$!

# Give the bridge time to open TCP port 8766 before Webots starts its controller.
sleep 3

if ! kill -0 "$ros_pid" 2>/dev/null; then
    echo "ERROR: ROS 2 launch exited. See $LOG_DIR/ros2.log" >&2
    exit 1
fi

echo "Starting Webots..."
setsid webots --mode=realtime "$WORLD_FILE" \
    >"$LOG_DIR/webots.log" 2>&1 &
webots_pid=$!

echo "Starting viewer server..."
setsid python3 -m http.server 8000 --directory "$PROJECT_ROOT" \
    >"$LOG_DIR/viewer_server.log" 2>&1 &
viewer_pid=$!

sleep 2

echo
echo "Fabtino project is running."
echo "Viewer: $VIEWER_URL"
echo "Logs:   $LOG_DIR"
echo "Press Ctrl+C to stop ROS 2, Webots, and the viewer."

if command -v xdg-open >/dev/null 2>&1; then
    xdg-open "$VIEWER_URL" >/dev/null 2>&1 || true
fi

wait "$ros_pid"
