# Fabtino ROS 2 mapping forensic audit

**Scope:** read-only inspection of the active ROS 2 Humble source tree and browser viewer. No source file was modified. The audit covers the requested files plus the active Webots controller, world, shared geometry, and legacy implementation where they affect the runtime contract.

## Executive conclusion

The primary defect is in `ros_ws/src/fabtino_mapping/fabtino_mapping/mapping_node.py`:

1. `scan_cb()` stores only one `LaserScan` (`line 22`).
2. `update()` runs independently (`line 19`) and reuses that same scan every timer tick (`lines 24-30`).
3. The scan is projected with the newest pose in `self.pose`, not a pose associated with the scan timestamp (`lines 25-28`).

Therefore, while the robot moves or rotates, one old scan is repeatedly inserted at changing poses. This directly explains both wall thickening and walls that follow/rotate with the robot. It is a ROS-side temporal/pose association error, not a browser compensation problem.

A second, independent ROS-side defect is that the active nodes do not use the shared LiDAR transform convention. `frames.py` and YAML specify `angle_sign=-1` (raw Webots positive angle interpreted toward robot right, converted to robot +Y-left), while both localization and mapping use `y = range * sin(angle)` and thus treat increasing scan angle as robot +Y. The localization import of `lidar_polar_to_robot` is unused.

The browser does not currently rotate accumulated map points a second time. In fact, `/fabtino/pointcloud` is subscribed to by the gateway but never serialized, `new_points` is always empty, and `app.js` never calls `ThreeDRenderer.addPoints()`. The 3D point viewer is consequently a deployment/data-path issue, separate from the 2D map-following defect.

## 1. Root causes by symptom

### Wall thickening — real root cause

**File:** `ros_ws/src/fabtino_mapping/fabtino_mapping/mapping_node.py`

- `line 22`: `scan_cb()` overwrites `self.scan`; no queue, sequence, or timestamped scan record is retained.
- `line 19`: a periodic `update()` timer is created.
- `lines 24-30`: every timer call reprocesses `self.scan`, increments the grid, and publishes points. The same scan can be integrated multiple times before the next scan callback.
- `lines 25-28`: each repetition uses the current `self.pose` and recomputes every endpoint.

The occupancy grid's ray integration (`line 29`) therefore receives repeated, slightly displaced occupied endpoints and free-space rays. The configured evidence values in `config/robot_parameters.yaml:127-133` can reduce persistence/noise, but they do not correct repeated integration of an old scan.

### Wall rotating/following the robot — real root cause

**File:** `ros_ws/src/fabtino_mapping/fabtino_mapping/mapping_node.py:25-29`.

The endpoint is calculated as:

```text
world = current_pose ⊕ current_scan_return
```

but `current_scan_return` is often an old scan. During a pure turn, the old return is reprojected with each new yaw, so the map receives a rotating sequence of copies. This exactly matches the reported “wall remains behind the robot” behavior.

**Secondary contributing issue:** the mapper has no scan timestamp/pose history and no synchronization policy. Even if it integrated once per scan, using the latest pose can create spatial smear whenever ROS callback timing differs from sensor timing.

### 3D viewer deployment — unrelated to the ROS map-following cause

- `ros_ws/src/fabtino_mapping/fabtino_mapping/mapping_node.py:40-43` publishes a real `PointCloud2` with `x`, `y`, and `z` fields, but explicitly packs `z=0.0` for every point (`line 43`). Thus the current ROS cloud is structurally 3D but has a flat Z plane; LiDAR height is discarded.
- `ros_ws/src/fabtino_viewer/fabtino_viewer/viewer_gateway.py:13` subscribes to `/fabtino/pointcloud` and stores it as `latest['cloud']`, but `packet()` never reads that value.
- `viewer_gateway.py:26` initializes `new_points` to `[]` on every packet.
- `js/app.js:116-130` reads `message.new_points` only for the 2D render call. There is no `ThreeDRenderer.addPoints(points)` call.
- `js/three-renderer.js:86-134` has a functioning `addPoints()` implementation, but nothing in the active app invokes it.
- `js/map-processor.js` is not imported or used by `app.js`; its comment describes an older/alternate controller path.

Result: the 3D canvas can display the robot and grid helper, but no ROS point cloud reaches it. This is a viewer gateway/application integration gap, not the source of the wall rotation in `/fabtino/map`.

### Localization/frame issues — real secondary issue

- `ros_ws/src/fabtino_webots_bridge/fabtino_webots_bridge/webots_bridge_node.py:82-89` forwards raw IMU orientation and raw LiDAR ranges. It sets `LaserScan.header.frame_id='lidar_link'`, but does not transform or mirror the scan.
- `webots_bridge_node.py:88` sets `angle_min=-fov/2`, `angle_max=+fov/2`, and increasing `angle_increment`; no explicit validation of the Webots range-image ordering is present.
- `ros_ws/src/fabtino_localization/fabtino_localization/localization_node.py:46` independently converts scans using `d*cos(a), d*sin(a)`, ignoring the imported helper at `line 16` and ignoring configured `lidar_angle_sign`.
- `ros_ws/src/fabtino_mapping/fabtino_mapping/mapping_node.py:28` repeats the same independent `+sin` conversion.
- `localization_node.py:38-40` extracts IMU yaw and fuses it directly as an absolute yaw observation. The EKF is initialized at yaw zero at `line 24`; no startup IMU yaw reference is captured or subtracted. The active estimator is therefore hybrid: startup-zero EKF state plus raw Webots/IMU yaw observations.
- `localization_node.py:52-56` publishes the result as `map -> base_link`, but that map frame is only coherent if the yaw convention and startup reference are coherent.

The intended configuration says local-start map coordinates (`config/robot_parameters.yaml:1-3`, `robot.initial_world_pose`, and `simulation.lidar_angle_sign: -1` at lines 5-24), but the active implementation does not fully enforce that contract. The Webots world also contains a nonzero robot rotation in `webots/worlds/world_fixed.wbt:98-100`, so raw IMU yaw versus startup-relative yaw matters in practice.

## 2. Exact coordinate conventions found

### LiDAR angular convention

There are conflicting conventions:

| Location | Implemented convention |
|---|---|
| `fabtino_core/localization/frames.py:12-19` | Robot `+x` forward, `+y` left, `+yaw` CCW; default `angle_sign=-1`; positive Webots scan angle is documented as robot right, so `robot_y=-r*sin(a)`. |
| `config/robot_parameters.yaml:21-24` | `lidar_angle_sign: -1`. |
| `localization_node.py:44-46` | `robot_x=r*cos(a)`, `robot_y=+r*sin(a)`; helper is imported but unused. |
| `mapping_node.py:26-29` | Same `robot_y=+r*sin(a)` conversion. |
| `tests/test_transforms.py:17-22` | Duplicate test helper uses `left=-r*sin(a)`, matching `frames.py`, not the active ROS nodes. |
| Webots bridge/controller | Raw ranges only; no mirroring. The bridge assigns the ROS angular interval but does not document/verify range-array direction. |

Independent polar-to-robot implementations are therefore:

1. Shared helper: `ros_ws/src/fabtino_core/fabtino_core/localization/frames.py:7-28`.
2. Localization runtime: `ros_ws/src/fabtino_localization/fabtino_localization/localization_node.py:44-46`.
3. Mapping runtime: `ros_ws/src/fabtino_mapping/fabtino_mapping/mapping_node.py:26-29`.
4. Test-only duplicate: `tests/test_transforms.py:17-22`.
5. Legacy controller has another explicit conversion path around `legacy/webots_controller_legacy.py:588-601`, plus shared helper use elsewhere.

The active runtime is not mirrored anywhere after the bridge; it uses `+sin`. The configured/shared convention expects `-sin`. This can mirror geometry, but by itself does not explain a wall that progressively follows the robot; the repeated latest-scan integration does.

### Robot/map convention

- ROS planar convention in the shared helper: `+X` forward, `+Y` left, `+Z` upward, positive yaw counter-clockwise (`frames.py:14-19`).
- `robot_to_world_point()` uses the standard CCW rotation matrix (`frames.py:31-42`).
- Mapping publishes `OccupancyGrid.header.frame_id='map'` and origin `(-radius,-radius)` (`mapping_node.py:34-39`), so the grid is intended as a fixed metric map centered at the startup/local origin.
- Localization publishes `map -> base_link` (`localization_node.py:52-56`).
- Browser 2D rendering uses direct map coordinates: `grid-renderer.js:38-46`, `66-82`, and robot yaw only for the robot glyph at `108-116`.
- The intended map is local-start coordinates, not necessarily Webots global coordinates. However, raw Webots ground truth is published with `frame_id='map'` by `webots_bridge_node.py:90-92`, which is semantically misleading if the initial Webots world pose is not the local map origin.

### Localization yaw classification

The active localization yaw is **not consistently startup-relative**. `localization_node.py:24` initializes yaw to zero, but `imu_cb()` at `35-40` fuses the raw bridge IMU yaw directly. The bridge obtains that yaw from Webots IMU RPY at `webots/controllers/.../fabtino_webots_bridge_controller.py:194-204`. No reference subtraction exists in the active node. It is therefore best classified as a startup-zero EKF that is pulled toward raw Webots/IMU world yaw.

The legacy controller does capture an IMU reference and exposes both raw/local yaw (`legacy/webots_controller_legacy.py:621-630`, `888-896`), but that is not the launched ROS 2 path.

## 3. Real runtime data-flow diagram

```text
Webots mapping_lidar
  └─ getRangeImage() + fov/resolution
     webots/controllers/fabtino_webots_bridge_controller/...py:197-208
       TCP JSON sensors packet
         └─ fabtino_webots_bridge_node.process_rx()
            ├─ /fabtino/scan  LaserScan(frame_id=lidar_link)
            ├─ /fabtino/imu/data_raw  Imu(raw Webots yaw)
            └─ /fabtino/joint_states

/fabtino/scan ───────────────┐
/fabtino/imu + joint_states ─┼─ localization_node
                              └─ EKF + scan matcher
                                 └─ /fabtino/odometry/filtered
                                    frame_id=map, child=base_link

/fabtino/scan + /odometry/filtered
  └─ mapping_node
     ├─ latest scan retained without timestamped pose association
     ├─ timer repeatedly projects latest scan with newest pose
     ├─ CoreGrid.update_ray() ── /fabtino/map (OccupancyGrid, map frame)
     └─ PointCloud2(map frame, z forced to 0) ── /fabtino/pointcloud

/fabtino/map ───────────────┐
/fabtino/odometry/filtered ─┤
/fabtino/scan ──────────────┤ viewer_gateway latest cache
/fabtino/pointcloud ────────┘ (cloud cached but never packetized)
                                └─ WebSocket packet
                                   points_frame="map"
                                   new_points=[] always
                                      └─ js/websocket-manager.js
                                         └─ js/app.js
                                            ├─ MapModel.loadGrid()
                                            ├─ GridRenderer.render(map, robot, [])
                                            └─ ThreeDRenderer.updateRobot()
                                               (addPoints() never called)
```

## 4. Transformations at each stage

1. Webots controller: produces raw range-image distances and raw IMU RPY; no LiDAR Cartesian conversion.
2. Bridge: wraps ranges in `LaserScan`, assigns `angle_min=-fov/2`, increasing `angle_increment`; no spatial transform.
3. Localization scan matcher: converts each valid range to local 2D points with `(+cos,+sin)` at `localization_node.py:44-46`; scan matcher operates in consecutive robot frames and refines EKF pose.
4. Localization pose output: EKF pose becomes `map -> base_link`; no point cloud is emitted here.
5. Mapping: converts range to robot point with `(+cos,+sin)` and applies newest pose with `wx=x+c*rx-s*ry`, `wy=y+s*rx+c*ry` (`mapping_node.py:25-29`). It updates the fixed grid and publishes map-frame endpoints.
6. Occupancy grid: `CoreGrid.update_ray()` integrates free cells and endpoint occupancy in map metric coordinates; published origin is fixed at `(-radius,-radius)`.
7. Point cloud: mapping stores endpoint `(wx,wy,0)` in `PointCloud2`, frame `map` (`mapping_node.py:40-43`).
8. Gateway: extracts robot pose and map but does not decode/forward PointCloud2; it declares `points_frame='map'` and sends an empty point list (`viewer_gateway.py:23-36`).
9. Browser: decodes occupancy cells and draws them directly in map coordinates. `GridRenderer` only applies screen pan/zoom and flips canvas Y; it does not apply robot pose to map points. `ThreeDRenderer` transforms map `(x,y,z)` to Three.js `(x,z,-y)` only inside its unused `addPoints()` path.

## 5. Duplicate/conflicting transformation implementations

- Shared helper versus active runtime: `frames.py:7-28` uses configurable/default `-sin`; localization and mapping use `+sin` inline.
- Test duplicate: `tests/test_transforms.py:17-22` independently implements `-sin` instead of calling the shared helper.
- Legacy controller: explicit range-to-robot/world projection around `legacy/webots_controller_legacy.py:588-601`; it is a separate architecture from the launched ROS 2 mapper.
- Browser alternate path: `js/map-processor.js:5-35` assumes points are already world/map coordinates but still ray-traces using the current robot pose. It would be a second use of robot pose if called; it is currently unused by `app.js`.
- Active mapping has no scan timestamp transform. `LaserScan.header.stamp` is generated by the bridge, but mapping ignores it and publishes with current time (`mapping_node.py:31`, `43`).

## 6. Occupancy grid and browser frame assessment

The ROS occupancy grid is structurally fixed in the `map` frame: fixed width/height, fixed resolution, fixed origin, and no browser-side map rotation. The problem is that incorrect/repeated ROS projections write moving geometry into that fixed grid. The browser is not the component rotating the accumulated grid.

The browser does not perform a second transform on the active `new_points` path because `new_points` is always empty. `GridRenderer` draws coordinates as supplied. The unused `MapProcessor` is a latent conflicting implementation and should not be used for map-frame endpoints.

## 7. Minimal fix architecture (proposal only; not implemented)

1. Make the ROS frame contract authoritative and explicit: `map` is fixed local-start XY; `base_link` is robot +X forward/+Y left; `lidar_link` has a documented static extrinsic; one shared polar conversion is used everywhere.
2. In the bridge, publish correct scan timing and a documented scan ordering. Keep raw `LaserScan` data raw; do not compensate in JavaScript.
3. In localization, use the shared polar helper (or a ROS TF-consistent equivalent) for scan matching. Capture startup IMU yaw reference once and fuse startup-relative yaw, or explicitly initialize the EKF from the raw yaw and document that map frame as Webots world; do not mix the two.
4. In mapping, process each scan exactly once. Store a bounded queue of scans, pair each scan with an odometry pose interpolated/extrapolated to that scan timestamp, and integrate only that pair. Do not reuse a scan on a timer. A timer may publish the already-integrated grid, but must not reintegrate sensor data.
5. Publish map-frame `PointCloud2` endpoints with timestamp/frame consistent with the integrated scan. Preserve actual Z only if a valid sensor/world Z convention is defined; otherwise explicitly declare the product 2.5D/flat and keep the field at zero.
6. If 3D display is required, decode `/fabtino/pointcloud` in `viewer_gateway`, serialize `new_points` as map-frame points, and call `ThreeDRenderer.addPoints()` from `app.js`. Do not apply robot yaw/translation to those points in the browser.
7. Keep browser rendering limited to visualization coordinates: map-to-canvas and map-to-Three.js axis conversion only. Robot pose may transform the robot glyph, never accumulated map geometry.

## 8. Proposed test plan

### Unit tests

- Test one canonical polar-to-robot helper for forward, left/right, and angle sign cases.
- Test robot-to-map transform for yaw 0, +90°, -90°, translation, and inverse consistency.
- Test startup-relative yaw reference with nonzero Webots initial orientation.
- Test scan ordering/angle increment against a synthetic wall at known left/right locations.

### Mapping integration tests

- Feed one scan and two timer ticks without a new scan; assert one integration only and no revision/point duplication.
- Feed the same physical wall from poses before/after a 90° in-place turn; assert map-frame wall coordinates remain fixed.
- Feed scans with timestamps between odometry samples; assert the pose used is timestamp-associated, not latest-arrival pose.
- Assert `/fabtino/map.header.frame_id == 'map'`, fixed origin, and unchanged occupied cells when only robot yaw changes and no new scan is integrated.
- Assert `PointCloud2` field layout, frame, Z contract, and one cloud per processed scan.

### ROS/browser path tests

- Gateway test: inject a `PointCloud2`, assert WebSocket `new_points` contains decoded map-frame points.
- Browser test: assert `app.js` forwards those points to `ThreeDRenderer.addPoints()` and does not call a robot-pose transform.
- Regression test: rotate only `robot.theta`; map pixels and stored point coordinates must not rotate.

### Existing test coverage assessment

- `tests/test_transforms.py` validates the shared helper and a duplicated test helper, not the active ROS `LocalizationNode` or `MappingNode` callbacks.
- `tests/test_scan_matcher.py` validates the pure scan matcher motion math, not LiDAR angle convention, ROS timestamps, or mapping integration.
- `tests/test_ekf.py` validates pure EKF updates, but does not test raw-versus-startup-relative IMU yaw in `localization_node.py`.
- `tests/test_motion_model.py` validates wheel odometry math only.
- No existing test exercises `/fabtino/scan -> /fabtino/odometry/filtered -> mapping_node.update()`, repeated timer behavior, timestamp association, `/fabtino/pointcloud` gateway serialization, or `ThreeDRenderer.addPoints()` deployment.
- The test runner could not be executed in this environment because `pytest` is not installed (`pytest: command not found`). No code was changed to address that environment issue.

## 9. Files that should not be changed for the primary fix

- `js/grid-renderer.js`: it correctly renders fixed map coordinates and only rotates the robot glyph.
- `js/robot-state.js`: it only stores pose state; it is not transforming map geometry.
- `js/websocket-manager.js`: transport parsing/reconnect is unrelated to wall accumulation.
- `js/map-model.js`: its grid/ray utilities are not the active ROS mapper and are not the cause of ROS `/fabtino/map` drift.
- `js/three-renderer.js`: it needs a data-path hookup for 3D deployment, but must not be changed to compensate for ROS map errors.
- `tests/test_ekf.py` and `tests/test_motion_model.py`: pure-model tests are not the runtime defect location.
- `ros_ws/src/fabtino_webots_bridge/.../webots_bridge_node.py`: do not add world/map projection here as a workaround; the bridge should remain a sensor transport layer except for correcting message metadata/timing.
- `legacy/webots_controller_legacy.py`: it is not part of `fabtino_sim.launch.py` and should not be modified as part of the active ROS 2 fix.

## 10. Root-cause classification summary

| Finding | Classification | Why |
|---|---|---|
| Latest scan repeatedly integrated on timer with newest pose | **Real root cause** | Directly creates wall thickening and robot-following/rotation. |
| No scan timestamp to pose association | **Secondary contributing issue** | Adds motion distortion even after duplicate integration is removed. |
| Active `+sin` conversion conflicts with shared/configured `-sin` | **Real frame issue / secondary to reported temporal symptom** | Can mirror LiDAR geometry and corrupt map orientation; must be unified. |
| Raw IMU yaw fused without startup reference | **Real localization/frame issue** | Makes local map yaw inconsistent when Webots starts at nonzero heading. |
| Browser second-transform concern | **Not the current cause** | Active browser does not transform `new_points`; it receives none. |
| `/fabtino/pointcloud` not forwarded and `addPoints()` never called | **Unrelated viewer/UI deployment issue** | Explains absent 3D points, not the 2D wall moving in `/fabtino/map`. |
| Z forced to zero in PointCloud2 | **Secondary viewer/data-product issue** | Cloud is flat/2.5D; independent of XY wall drift. |

**Stop point:** this audit intentionally makes no implementation changes. The next step should be an explicitly approved ROS-side fix following the architecture above, followed by runtime ROS integration tests and only then the optional 3D gateway hookup.
