#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORLD_FILE="$PROJECT_ROOT/webots/worlds/world_fixed.wbt"

echo "Stopping Fabtino project processes..."

kill_matching() {
    local pattern="$1"
    local pids
    pids="$(pgrep -f "$pattern" || true)"
    if [[ -z "$pids" ]]; then
        return
    fi

    while read -r pid; do
        [[ -z "$pid" || "$pid" == "$$" ]] && continue
        kill "$pid" 2>/dev/null || true
    done <<< "$pids"
}

# Stop the project launcher first so its process-group cleanup can run.
kill_matching "$PROJECT_ROOT/run_full_project.sh"

# Stop only the ROS launch started for this project.
kill_matching "ros2 launch fabtino_bringup fabtino_sim.launch.py"

# Stop the project Webots world and its controller.
kill_matching "$WORLD_FILE"
kill_matching "fabtino_webots_bridge_controller.py"

# Stop only the local viewer server serving this project directory.
kill_matching "python3 -m http.server 8000 --directory $PROJECT_ROOT"

# Give processes a moment to exit cleanly, then remove only remaining
# project-matching processes.
sleep 1
kill_matching "$PROJECT_ROOT/run_full_project.sh"
kill_matching "ros2 launch fabtino_bringup fabtino_sim.launch.py"
kill_matching "$WORLD_FILE"
kill_matching "fabtino_webots_bridge_controller.py"
kill_matching "python3 -m http.server 8000 --directory $PROJECT_ROOT"

echo "Fabtino project backend jobs stopped."
