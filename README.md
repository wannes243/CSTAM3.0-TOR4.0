# Fabtino Robot Navigation System

Fabtino combines a browser dashboard, ROS 2 Humble packages, and a Webots robot
simulation. Operators can drive manually, build or import a map, define denied
zones, and run delivery tasks with arrival delays and a return to base.

## High-level Software Architecture Diagram

```mermaid
flowchart TB
    subgraph CLIENT["Browser and map files"]
        UI["Web dashboard<br/>Manual controls, 2D/3D viewer<br/>Zone placement and delivery queue"]
        FILES["PNG / JSON map files<br/>Occupancy map, denied zones and base"]
        UI <-->|"Import / export"| FILES
    end

    subgraph ROS["ROS 2 Humble application"]
        GW["fabtino_viewer<br/>WebSocket gateway<br/>Commands, telemetry and confirmations"]
        LOC["fabtino_localization<br/>Wheel / IMU fusion, EKF<br/>Scan matching and map localization"]
        MAP["fabtino_mapping<br/>Timestamped scan projection<br/>Static evidence filter and live obstacle layer"]
        NAV["fabtino_navigation<br/>Mission manager and delivery scheduler<br/>Global A* planner and local goal follower"]
        SAFE["fabtino_safety<br/>Manual / autonomous command arbitration<br/>Obstacle, denied-zone and freshness checks"]
        BRIDGE["fabtino_webots_bridge<br/>ROS topics to TCP JSON<br/>Sensor timestamps, reconnect and command timeout"]
    end

    subgraph SIM["Webots simulation"]
        CTRL["Webots robot controller<br/>Sensor acquisition and wheel commands<br/>LiDAR masking and command watchdog"]
        ROBOT["Robot and simulated world<br/>Wheel motors and encoders<br/>LiDAR, IMU and physics"]
        CTRL <-->|"Webots device API"| ROBOT
    end

    STATE["Persistent navigation configuration<br/>Named circles and delivery base"]

    UI <-->|"WebSocket JSON :8765"| GW
    GW -->|"Goals, zone configuration and delivery requests"| NAV
    GW -->|"Manual velocity commands"| SAFE
    GW <-->|"Reset / import operations and estimated pose"| LOC
    GW <-->|"Map operations, static map and live points"| MAP
    NAV -->|"Mission, delivery and configuration status"| GW
    SAFE -->|"Safety status"| GW

    BRIDGE -->|"Wheel encoders, IMU and LiDAR"| LOC
    BRIDGE -->|"Captured LiDAR scans"| MAP
    BRIDGE -->|"Current LiDAR scans"| SAFE
    LOC -->|"Pose at scan capture time"| MAP
    LOC -->|"Estimated pose and measured velocity"| NAV
    LOC -->|"Fresh estimated pose"| SAFE
    MAP -->|"Static map + temporary obstacles"| NAV
    NAV -->|"Autonomous velocity, goals and denied zones"| SAFE
    SAFE -->|"Guarded motor velocity"| BRIDGE
    NAV <-->|"Load / atomic save"| STATE
    BRIDGE <-->|"TCP JSON :8766<br/>Sensor packets / control commands"| CTRL

    classDef browser fill:#10232d,stroke:#39d9ff,color:#e7eef5;
    classDef processing fill:#101720,stroke:#8493a3,color:#e7eef5;
    classDef safety fill:#291820,stroke:#ff5266,color:#e7eef5;
    classDef simulator fill:#102319,stroke:#38f28a,color:#e7eef5;
    classDef storage fill:#292515,stroke:#f4d35e,color:#e7eef5;
    class UI,GW browser;
    class LOC,MAP,NAV,BRIDGE processing;
    class SAFE safety;
    class CTRL,ROBOT simulator;
    class FILES,STATE storage;
```

Connections inside the ROS boundary represent ROS topics and operation
confirmations. The browser connects to the gateway rather than to the robot
controller. All manual and autonomous motor commands pass through the safety
supervisor before reaching Webots. Ground-truth pose is available for diagnostics;
navigation uses the estimated pose.

## Component responsibilities

| Component | Responsibility | Source |
| --- | --- | --- |
| Dashboard | Manual driving, map display, import/export, circle placement, base selection and delivery controls | [HTML](map_viewer2_modular.html), [JavaScript](js/) |
| Viewer gateway | WebSocket commands and telemetry; coordinated map/reset confirmations | [fabtino_viewer](ros_ws/src/fabtino_viewer/) |
| Localization | Timestamped wheel/IMU fusion, EKF, LiDAR scan matching and imported-map localization | [fabtino_localization](ros_ws/src/fabtino_localization/) |
| Mapping | World-frame scan projection, stable-point confirmation, static occupancy and temporary navigation obstacles | [fabtino_mapping](ros_ws/src/fabtino_mapping/) |
| Mission manager | Goals, exploration, zone/base configuration, ordered delivery stops, delays and base return | [mission_manager_node.py](ros_ws/src/fabtino_navigation/fabtino_navigation/mission_manager_node.py) |
| Global planner | A* routes around occupied cells and circular denied zones, including robot clearance | [global_planner_node.py](ros_ws/src/fabtino_navigation/fabtino_navigation/global_planner_node.py) |
| Local planner | Route following, final position/orientation adjustment and settled arrival detection | [local_planner_node.py](ros_ws/src/fabtino_navigation/fabtino_navigation/local_planner_node.py) |
| Safety supervisor | Command arbitration, sensor/command freshness, proximity stops and swept-motion zone checks | [fabtino_safety](ros_ws/src/fabtino_safety/) |
| ROS/Webots bridge | Timestamped sensor transport, motor commands, scan/reset actions and reconnection | [fabtino_webots_bridge](ros_ws/src/fabtino_webots_bridge/) |
| Webots controller | Reads simulated devices, applies wheel commands and enforces its command watchdog | [Controller](webots/controllers/fabtino_webots_bridge_controller/) |
| Shared core | Reusable geometry, localization algorithms, occupancy/A*, static-point filtering, zone geometry and delivery transitions | [fabtino_core](ros_ws/src/fabtino_core/) |
| Bringup | Starts and configures the ROS nodes | [fabtino_sim.launch.py](ros_ws/src/fabtino_bringup/launch/fabtino_sim.launch.py) |

## Main workflows

1. **Mapping:** Webots sensors → TCP bridge → timestamped localization → scan
   projection. Returns enter the static map only after more than 3 seconds of
   stable world-frame observations. Current returns immediately populate a
   temporary obstacle layer for navigation.
2. **Navigation:** Dashboard goal → mission manager → A* route → local goal
   follower → safety supervisor → bridge → wheel controller. Estimated pose
   and current obstacles continuously feed the control loop.
3. **Denied zones and deliveries:** The mission manager owns named circular
   zones and the base. Planners and safety enforce the exclusions with robot
   clearance. Each delivery stop selects the closest reachable safe approach,
   waits after a settled arrival, then advances to the next task. Return to base
   is enabled by default.
4. **Map import and reset:** The gateway coordinates request IDs and completion
   messages across localization, mapping and the mission manager. Imported
   annotations are committed after localization succeeds. Odometry reset clears
   zones and base because the coordinate frame changes.

PNG metadata and JSON bundles preserve the map scale, zones and base. The launch
also persists navigation configuration at
`~/.local/share/fabtino/navigation.json`. Delivery queues do not automatically
resume after restart.

## Run and validate

Target runtime: Ubuntu 22.04 with ROS 2 Humble and Webots. The dashboard is served
over HTTP on port 8000; the gateway uses WebSocket port 8765, and the Webots
controller listens for the ROS bridge on TCP port 8766.

```bash
bash build_ros2.sh
bash run_full_project.sh
```

Open `http://127.0.0.1:8000/map_viewer2_modular.html`.

```bash
# Isolated package suites and portable realtime TCP/WebSocket pipeline.
python3 tests/run_validation.py --package all

# Also run the dashboard workflows in Chrome.
python3 tests/run_validation.py --package all --browser

# Genuine DDS/Webots checks against an already-running simulation.
python3 tests/live_pipeline.py --delivery-json tests/fixtures/delivery.json
```

Portable tests explicitly substitute ROS topics and robot devices while using
real network transports. The live test requires genuine ROS/Webots; adjust its
delivery fixture to the active world and map frame. See
[setup, behavior and test instructions](README_ROS2_HUMBLE.md) for dependencies
and details, and [validation results](tests/results/package_validation.json)
for the latest completed suites.
