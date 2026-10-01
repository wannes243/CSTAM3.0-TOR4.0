"""Fabtino sensor-fusion mapping controller.

Webots is only the simulation/hardware adapter. Localization and sensor models
live in hardware-independent modules under sensors/ and localization/.

Current estimator:
    four mecanum wheel encoders -> equivalent planar odometry -> EKF prediction/update
    gyro           -> EKF yaw-rate/bias update
    LiDAR          -> occupancy mapping using the EKF pose

The controller uses Fabtino's four wheel encoders and fixed horizontal LiDAR while
preserving the hardware-independent estimator/navigation interfaces.
"""
from __future__ import annotations

import asyncio
import json
import math
import queue
from pathlib import Path
import sys
import threading
import time

import numpy as np

from controller import Keyboard, Supervisor

try:
    import websockets
except ImportError as exc:  # pragma: no cover - runtime dependency
    raise RuntimeError(
        "Missing Python package 'websockets'. Install with: python -m pip install websockets"
    ) from exc

# Allow the Webots controller to import the project-level robotics modules.
PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from config import load_config
from localization import (EKFConfig, KnownMapLocalizer, KnownMapLocalizerConfig,
                          LocalizationEKF, lidar_return_to_world)
from localization.scan_matcher import LidarScanMatcher, ScanMatcherConfig
from navigation import OccupancyGrid, NavigationController
from sensors import IMUAdapter, WheelGeometry, WheelMeasurement, WheelOdometryEstimator


CONFIG = load_config()
ROBOT_CFG = CONFIG["robot"]
SIM_CFG = CONFIG["simulation"]
EST_CFG = CONFIG["estimator"]
SAFETY_CFG = CONFIG["safety"]
VIEW_CFG = CONFIG.get("viewer", {})

# Fabtino R2025a geometry. The official model is a four-wheeled mecanum robot.
# The controller currently exposes only (v, omega), so lateral velocity is held at zero.
# This reduces the mecanum wheel model to an equivalent differential-drive pair for
# the existing hardware-independent WheelOdometryEstimator/EKF interfaces.
FABTINO_WHEEL_RADIUS_M = float(ROBOT_CFG.get("fabtino_wheel_radius_m", 0.10))
FABTINO_HALF_LENGTH_M = float(ROBOT_CFG.get("fabtino_half_length_m", 0.284979))
FABTINO_HALF_WIDTH_M = float(ROBOT_CFG.get("fabtino_half_width_m", 0.236938))
FABTINO_KINEMATIC_LEVER_M = FABTINO_HALF_LENGTH_M + FABTINO_HALF_WIDTH_M
FABTINO_VIRTUAL_TRACK_WIDTH_M = float(
    ROBOT_CFG.get("fabtino_virtual_track_width_m", 2.0 * FABTINO_KINEMATIC_LEVER_M)
)
WHEEL_RADIUS = FABTINO_WHEEL_RADIUS_M
TRACK_WIDTH = FABTINO_VIRTUAL_TRACK_WIDTH_M
MAX_LINEAR_SPEED = float(ROBOT_CFG["max_linear_speed_mps"])
MAX_ANGULAR_SPEED = float(ROBOT_CFG["max_angular_speed_rps"])
INITIAL_POSE = ROBOT_CFG.get("initial_world_pose", {})
INITIAL_X = float(INITIAL_POSE.get("x_m", 0.0))
INITIAL_Y = float(INITIAL_POSE.get("y_m", 0.0))
INITIAL_YAW = float(INITIAL_POSE.get("yaw_rad", 0.0))
COMMAND_TIMEOUT = float(SAFETY_CFG["command_timeout_s"])
SENSOR_TIMEOUT = float(SAFETY_CFG["sensor_timeout_s"])
MAX_STATE_COV_XY = float(SAFETY_CFG["max_state_covariance_xy_m2"])
MAX_STATE_COV_YAW = float(SAFETY_CFG["max_state_covariance_yaw_rad2"])
IMU_YAW_STD = float(EST_CFG.get("imu_yaw_std_rad", 0.01))
USE_IMU_YAW_REFERENCE = bool(EST_CFG.get("use_imu_yaw_reference", True))
DIAGNOSTIC_LOGGING = bool(EST_CFG.get("diagnostic_logging", True))
DIAGNOSTIC_LOG_PERIOD = max(0.1, float(EST_CFG.get("diagnostic_log_period_s", 0.5)))
GROUND_TRUTH_DIAGNOSTICS = bool(EST_CFG.get("ground_truth_diagnostics", True))
NAV_CFG = CONFIG.get("navigation", {})
MAP_CFG = CONFIG.get("mapping", {})
NAV_RESOLUTION = float(NAV_CFG.get("map_resolution_m", 0.05))
NAV_RADIUS = float(NAV_CFG.get("map_radius_m", 20.0))
NAV_MARGIN = float(NAV_CFG.get("safety_margin_m", 0.25))
NAV_MAX_V = float(NAV_CFG.get("max_linear_speed_mps", 0.25))
NAV_MAX_W = float(NAV_CFG.get("max_angular_speed_rps", 1.0))
NAV_DEBUG = bool(NAV_CFG.get("debug_logging", True))
SCAN_MATCH_CFG = EST_CFG.get("lidar_scan_matcher", {})
KNOWN_MAP_CFG = EST_CFG.get("known_map_localization", {})
ACCELEROMETER_ENABLED = bool(EST_CFG.get("accelerometer_enabled", False))
ACCELEROMETER_STD = float(EST_CFG.get("accelerometer_std_mps2", 0.08))
EXPECTED_TIMESTEP_MS = int(SIM_CFG.get("basic_time_step_ms", 0))
LOCALIZATION_SCAN_PERIOD_S = float(SIM_CFG.get("localization_scan_period_s", 0.05))
MAPPING_UPDATE_PERIOD_S = float(SIM_CFG.get("mapping_update_period_s", 0.10))
VIEWER_UPDATE_PERIOD_S = float(SIM_CFG.get("viewer_update_period_s", 0.10))

WS_HOST = "0.0.0.0"
WS_PORT = 8765
SEND_EVERY_N = 3
LIDAR_SUBSAMPLE = max(1, int(SIM_CFG["lidar_subsample"]))
EXPECTED_LIDAR_RESOLUTION = int(SIM_CFG.get("lidar_horizontal_resolution", 0))
EXPECTED_LIDAR_FOV = float(SIM_CFG.get("lidar_fov_rad", 0.0))
LIDAR_ANGLE_SIGN = float(SIM_CFG.get("lidar_angle_sign", -1.0))
MAX_LIDAR_RANGE = float(SIM_CFG["lidar_max_range_m"])
MIN_LIDAR_RANGE = float(SIM_CFG.get("lidar_min_range_m", 0.2))
MIN_LIDAR_VALID_FRACTION = float(SAFETY_CFG.get("min_lidar_valid_fraction", 0.05))
FRONT_STOP_DISTANCE = float(SAFETY_CFG.get("front_stop_distance_m", 0.45))
FRONT_HALF_WIDTH = float(SAFETY_CFG.get("front_half_width_m", 0.25))
FABTINO_BODY_SLOT_Z_M = 0.145
FABTINO_LIDAR_MOUNT_POSE_Z_M = 0.025


def clean(value: float, default: float = 0.0) -> float:
    try:
        value = float(value)
        return value if math.isfinite(value) else default
    except (TypeError, ValueError):
        return default


class FabtinoController:
    def __init__(self) -> None:
        # Supervisor is used only so diagnostics can read the simulated
        # ground-truth pose with getSelf(). Ground truth is never fed into the EKF.
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())
        # Webots ground truth is read for diagnostics only; it never enters
        # the EKF or the navigation controller.
        self.self_node = self.robot.getSelf()

        # Official Fabtino R2025a wheel-joint motor names.
        motor_names = {
            "fl": "front_left_wheel_joint",
            "fr": "front_right_wheel_joint",
            "rl": "back_left_wheel_joint",
            "rr": "back_right_wheel_joint",
        }
        self.motors = {}
        for key, name in motor_names.items():
            motor = self.robot.getDevice(name)
            if motor is None:
                raise RuntimeError(f'Fabtino motor device "{name}" was not found')
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)
            self.motors[key] = motor

        # Use all four encoders. The existing estimator accepts a left/right pair,
        # so _wheel_measurement() feeds it the average angle of each mecanum side.
        self.encoders = {
            "fl": self.robot.getDevice("front_left_wheel_joint_sensor"),
            "fr": self.robot.getDevice("front_right_wheel_joint_sensor"),
            "rl": self.robot.getDevice("back_left_wheel_joint_sensor"),
            "rr": self.robot.getDevice("back_right_wheel_joint_sensor"),
        }
        for side, sensor in self.encoders.items():
            if sensor is None:
                raise RuntimeError(f'Missing Fabtino encoder "{side}"')
            sensor.enable(self.timestep)

        self.lidar = self.robot.getDevice("mapping_lidar")
        if self.lidar is None:
            raise RuntimeError('Fabtino LiDAR device "mapping_lidar" was not found')
        self.lidar.enable(self.timestep)

        # Custom Fabtino sensor names from the adapted world file.
        self.imu = self.robot.getDevice("inertial unit")
        self.gyro = self.robot.getDevice("mapping_gyro")
        self.accelerometer = self.robot.getDevice("mapping_accelerometer")
        for device in (self.imu, self.gyro, self.accelerometer):
            if device:
                device.enable(self.timestep)

        self.keyboard = Keyboard()
        self.keyboard.enable(self.timestep)

        # The map frame is local: the initial robot position is the map origin
        # and the initial IMU yaw is used as the local heading reference. This
        # prevents a non-zero simulator startup heading from rotating the map.
        self.ekf = LocalizationEKF(
            EKFConfig(
                initial_position_std_m=float(EST_CFG["initial_position_std_m"]),
                initial_yaw_std_rad=float(EST_CFG["initial_yaw_std_rad"]),
                initial_velocity_std_mps=float(EST_CFG["initial_velocity_std_mps"]),
                initial_angular_velocity_std_rps=float(EST_CFG["initial_angular_velocity_std_rps"]),
                initial_gyro_bias_std_rps=float(EST_CFG["initial_gyro_bias_std_rps"]),
                accel_drive_std_mps2=float(EST_CFG["accel_drive_std_mps2"]),
                yaw_accel_std_rps2=float(EST_CFG["yaw_accel_std_rps2"]),
                gyro_bias_rw_std_rps=float(EST_CFG["gyro_bias_rw_std_rps"]),
                encoder_distance_std_m=float(EST_CFG["encoder_distance_std_m"]),
                encoder_slip_std_m=float(EST_CFG["encoder_slip_std_m"]),
                gyro_std_rps=float(EST_CFG["gyro_std_rps"]),
                track_width_m=TRACK_WIDTH,
            )
        )
        self.odom = WheelOdometryEstimator(
            WheelGeometry(radius_m=WHEEL_RADIUS, track_width_m=TRACK_WIDTH)
        )
        self.imu_adapter = IMUAdapter()

        self.web_v = 0.0
        self.web_w = 0.0
        self.last_web_command = time.monotonic()
        self.scanning = True
        self.frame = 0
        self.total_points = 0
        self.last_sensor_timestamp = 0.0
        self.last_gyro_timestamp = 0.0
        self.last_lidar_timestamp = 0.0
        self.last_v = 0.0
        self.last_omega = 0.0
        self.last_gyro_z = 0.0
        self.last_imu_yaw_raw = 0.0
        self.last_imu_yaw_local = 0.0
        self.last_yaw_error = 0.0
        self.last_yaw_rate_error = 0.0
        self.last_lidar_valid_fraction = 0.0
        self.front_obstacle_distance_m: float | None = None
        self.last_scan_match_status = "waiting_for_first_scan"
        self.last_scan_match_error_m = None
        self.scan_match_corrections = 0
        self.known_map_active = False
        self.known_map_localizer: KnownMapLocalizer | None = None
        self.known_map_request_id: str | None = None
        self.dynamic_points: list[list[float]] = []
        self.last_diagnostic_log = -1.0
        self.safety_stop_reason = ""
        self.last_safety_log_reason = None
        self.last_command_requested = (0.0, 0.0)
        self.last_command_applied = (0.0, 0.0)
        # Webots device APIs are owned by the simulation thread. Network
        # callbacks only enqueue requests and never touch devices directly.
        self.pending_actions: queue.SimpleQueue[str] = queue.SimpleQueue()
        self.nav_grid = OccupancyGrid(NAV_RESOLUTION, NAV_RADIUS)
        self.navigator = NavigationController(
            self.nav_grid, NAV_MARGIN, NAV_MAX_V, NAV_MAX_W
        )
        self.scan_matcher = LidarScanMatcher(
            ScanMatcherConfig(
                enabled=bool(SCAN_MATCH_CFG.get("enabled", True)),
                min_points=int(SCAN_MATCH_CFG.get("min_points", 40)),
                max_points=int(SCAN_MATCH_CFG.get("max_points", 96)),
                max_match_error_m=float(SCAN_MATCH_CFG.get("max_match_error_m", 0.18)),
                sigma_xy_m=float(SCAN_MATCH_CFG.get("sigma_xy_m", 0.04)),
                sigma_yaw_rad=float(SCAN_MATCH_CFG.get("sigma_yaw_rad", 0.035)),
                stationary_linear_velocity_mps=float(SCAN_MATCH_CFG.get("stationary_linear_velocity_mps", 0.01)),
                stationary_angular_velocity_rps=float(SCAN_MATCH_CFG.get("stationary_angular_velocity_rps", 0.02)),
                min_translation_m=float(SCAN_MATCH_CFG.get("min_translation_m", 0.01)),
                min_rotation_rad=float(SCAN_MATCH_CFG.get("min_rotation_rad", 0.02)),
                max_odom_translation_disagreement_m=float(SCAN_MATCH_CFG.get("max_odom_translation_disagreement_m", 0.05)),
                max_odom_rotation_disagreement_rad=float(SCAN_MATCH_CFG.get("max_odom_rotation_disagreement_rad", 0.10)),
                min_score_improvement_m=float(SCAN_MATCH_CFG.get("min_score_improvement_m", 0.002)),
            )
        )
        self.last_nav_debug_log = -1.0
        self.last_nav_debug_signature = None
        self.truth_error_count = 0
        self.truth_sum_sq_position = 0.0
        self.truth_sum_sq_yaw = 0.0
        self.truth_max_position = 0.0

        self.loop = None
        self.clients = set()

        now = self.robot.getTime()
        initial_imu_yaw = self.get_imu_yaw()
        # Webots sensors are not guaranteed to contain their first valid
        # sample before the first robot.step(). Do not capture a zero/default
        # IMU value as the map heading reference here.
        self.imu_yaw_reference: float | None = None if USE_IMU_YAW_REFERENCE else 0.0
        initial_gyro = self.get_gyro_z()
        self.ekf.initialize(now, INITIAL_X, INITIAL_Y, INITIAL_YAW, initial_gyro)
        self.truth_reference = self.get_simulation_pose_raw()
        self.last_imu_yaw_raw = initial_imu_yaw
        self.last_imu_yaw_local = 0.0
        self._reset_encoder_reference(now)

        print("=" * 72)
        print(" Fabtino — SENSOR-FUSION MAPPING CONTROLLER")
        print("=" * 72)
        print(" Robot model : official Webots Fabtino (mecanum)")
        print(f" Wheel radius: {WHEEL_RADIUS:.6f} m")
        print(f" Virtual track: {TRACK_WIDTH:.6f} m")
        print(f" Basic step  : {self.timestep} ms")
        if EXPECTED_TIMESTEP_MS and self.timestep != EXPECTED_TIMESTEP_MS:
            print(f"[CONFIG] WARNING: YAML timestep {EXPECTED_TIMESTEP_MS} differs from Webots {self.timestep}")
        print(f" LiDAR       : {self.lidar.getHorizontalResolution()} samples")
        if EXPECTED_LIDAR_RESOLUTION and self.lidar.getHorizontalResolution() != EXPECTED_LIDAR_RESOLUTION:
            print(f"[CONFIG] WARNING: YAML LiDAR resolution {EXPECTED_LIDAR_RESOLUTION} differs from device")
        if EXPECTED_LIDAR_FOV and abs(float(self.lidar.getFov()) - EXPECTED_LIDAR_FOV) > 1e-3:
            print(f"[CONFIG] WARNING: YAML LiDAR FOV {EXPECTED_LIDAR_FOV:.4f} differs from device {self.lidar.getFov():.4f}")
        print(f" IMU         : {'available' if self.imu else 'not available'}")
        print(f" Gyro        : {'available' if self.gyro else 'not available'}")
        print(f" Accel       : {'available' if self.accelerometer else 'not available'}")
        print(f" World pose  : x={INITIAL_X:.3f}, y={INITIAL_Y:.3f}, yaw={INITIAL_YAW:.3f} rad")
        print(" WebSocket   : ws://0.0.0.0:8765")
        print("=" * 72)

    def get_imu_yaw(self) -> float:
        if self.imu:
            rpy = self.imu.getRollPitchYaw()
            if rpy and len(rpy) >= 3:
                return clean(rpy[2], 0.0)
        return float(self.ekf.x[2]) if self.ekf.initialized else 0.0

    def local_imu_yaw(self, raw_yaw: float) -> float:
        """Convert Webots IMU yaw into the local map heading."""
        from localization.motion_model import wrap_angle
        if self.imu_yaw_reference is None:
            return 0.0
        return wrap_angle(float(raw_yaw) - float(self.imu_yaw_reference))

    def get_imu_rpy(self) -> tuple[float, float, float]:
        if self.imu:
            rpy = self.imu.getRollPitchYaw()
            if rpy and len(rpy) >= 3:
                return clean(rpy[0]), clean(rpy[1]), clean(rpy[2])
        return 0.0, 0.0, self.get_imu_yaw()

    def get_gyro(self) -> tuple[float, float, float]:
        if self.gyro:
            values = self.gyro.getValues()
            if values and len(values) >= 3:
                return clean(values[0]), clean(values[1]), clean(values[2])
        return 0.0, 0.0, 0.0

    def get_gyro_z(self) -> float:
        # Webots world/map is horizontal X/Y with +Z up; yaw is rotation about Z.
        return self.get_gyro()[2]

    def get_simulation_pose_raw(self) -> tuple[float, float, float] | None:
        """Read Webots ground truth for estimator diagnostics only."""
        if not GROUND_TRUTH_DIAGNOSTICS or self.self_node is None:
            return None
        try:
            translation = self.self_node.getField("translation").getSFVec3f()
            rotation = self.self_node.getField("rotation").getSFRotation()
            if len(translation) < 2 or len(rotation) < 4:
                return None
            # Fabtino remains planar in this world, so the z component of the
            # axis-angle rotation gives the world yaw sign and magnitude.
            yaw = float(rotation[3]) * (1.0 if float(rotation[2]) >= 0.0 else -1.0)
            return float(translation[0]), float(translation[1]), yaw
        except Exception:
            return None

    def get_simulation_pose_local(self) -> tuple[float, float, float] | None:
        raw = self.get_simulation_pose_raw()
        ref = self.truth_reference
        if raw is None or ref is None:
            return None
        dx, dy = raw[0] - ref[0], raw[1] - ref[1]
        c, s = math.cos(ref[2]), math.sin(ref[2])
        return (
            c * dx + s * dy,
            -s * dx + c * dy,
            math.atan2(math.sin(raw[2] - ref[2]), math.cos(raw[2] - ref[2])),
        )

    def estimator_error(self) -> dict | None:
        truth = self.get_simulation_pose_local()
        if truth is None:
            return None
        estimate = self.ekf.pose
        dx, dy = estimate[0] - truth[0], estimate[1] - truth[1]
        dyaw = math.atan2(math.sin(estimate[2] - truth[2]), math.cos(estimate[2] - truth[2]))
        return {
            "x_m": dx,
            "y_m": dy,
            "position_m": math.hypot(dx, dy),
            "yaw_rad": dyaw,
            "truth": {"x": truth[0], "y": truth[1], "yaw": truth[2]},
        }

    def update_truth_metrics(self) -> dict | None:
        error = self.estimator_error()
        if error is None:
            return None
        self.truth_error_count += 1
        self.truth_sum_sq_position += error["position_m"] ** 2
        self.truth_sum_sq_yaw += error["yaw_rad"] ** 2
        self.truth_max_position = max(self.truth_max_position, error["position_m"])
        return {
            "samples": self.truth_error_count,
            "position_rmse_m": math.sqrt(self.truth_sum_sq_position / self.truth_error_count),
            "yaw_rmse_rad": math.sqrt(self.truth_sum_sq_yaw / self.truth_error_count),
            "max_position_error_m": self.truth_max_position,
        }

    def get_acceleration(self) -> tuple[float, float, float]:
        if not ACCELEROMETER_ENABLED:
            return 0.0, 0.0, 0.0
        if self.accelerometer:
            values = self.accelerometer.getValues()
            if values and len(values) >= 3:
                return clean(values[0]), clean(values[1]), clean(values[2])
        return 0.0, 0.0, 0.0

    def _wheel_measurement(self, timestamp: float) -> WheelMeasurement:
        # For Fabtino with vy=0, the mean left/right wheel angles give the
        # same forward displacement and yaw increment as a differential pair
        # with a virtual track width of 2*(half_length + half_width).
        fl = clean(self.encoders["fl"].getValue())
        fr = clean(self.encoders["fr"].getValue())
        rl = clean(self.encoders["rl"].getValue())
        rr = clean(self.encoders["rr"].getValue())
        left_angle = 0.5 * (fl + rl)
        right_angle = 0.5 * (fr + rr)
        return WheelMeasurement(
            timestamp=timestamp,
            left_angle_rad=left_angle,
            right_angle_rad=right_angle,
        )

    def _reset_encoder_reference(self, timestamp: float) -> None:
        self.odom.reset(self._wheel_measurement(timestamp))

    def update_localization(self, timestamp: float) -> None:
        measurement = self._wheel_measurement(timestamp)
        odom = self.odom.update(measurement)

        raw_imu_yaw = self.get_imu_yaw()
        if USE_IMU_YAW_REFERENCE and self.imu_yaw_reference is None:
            self.imu_yaw_reference = raw_imu_yaw
            print(f"[POSE] initialized IMU yaw reference={raw_imu_yaw:+.3f} rad")
        local_imu_yaw = self.local_imu_yaw(raw_imu_yaw)
        gyro_z = self.get_gyro_z()

        self.last_imu_yaw_raw = raw_imu_yaw
        self.last_imu_yaw_local = local_imu_yaw
        self.last_gyro_z = gyro_z
        self.last_gyro_timestamp = timestamp

        if odom is not None:
            if not self.ekf.initialized:
                self.ekf.initialize(timestamp, INITIAL_X, INITIAL_Y, INITIAL_YAW, gyro_z)

            # Directly propagate pose from this wheel increment. This removes
            # the previous one-cycle yaw lag and makes in-place rotation
            # deterministic for LiDAR world-frame projection.
            self.ekf.predict_wheel_odometry(odom)
            self.ekf.update_wheel_odometry(odom)
            self.last_v = odom.linear_velocity_mps
            self.last_omega = odom.angular_velocity_rps

        if self.ekf.initialized:
            # Gyro supplies the instantaneous yaw-rate/bias constraint.
            self.ekf.update_gyro(gyro_z)
            # In Webots, the InertialUnit attitude is a reliable absolute
            # simulation measurement. Referencing it to the startup heading
            # keeps the map local while preventing scan rotation on turns.
            self.ekf.update_yaw(local_imu_yaw, IMU_YAW_STD)

            self.last_yaw_error = self.local_imu_yaw(raw_imu_yaw) - self.ekf.pose[2]
            from localization.motion_model import wrap_angle
            self.last_yaw_error = wrap_angle(self.last_yaw_error)
            self.last_yaw_rate_error = gyro_z - self.ekf.x[4].item()

        self.last_sensor_timestamp = timestamp
        self.update_truth_metrics()

        if DIAGNOSTIC_LOGGING and (timestamp - self.last_diagnostic_log) >= DIAGNOSTIC_LOG_PERIOD:
            x, y, yaw = self.ekf.pose
            truth_error = self.estimator_error()
            error_text = "truth=n/a"
            if truth_error:
                error_text = (
                    f"truth_err=({truth_error['x_m']:+.3f},{truth_error['y_m']:+.3f}) "
                    f"pos={truth_error['position_m']:.3f} yaw={truth_error['yaw_rad']:+.3f}"
                )
                if self.truth_error_count:
                    error_text += (
                        f" rmse={math.sqrt(self.truth_sum_sq_position / self.truth_error_count):.3f}"
                        f" max={self.truth_max_position:.3f}"
                    )
            print(
                f"[POSE] t={timestamp:7.3f} "
                f"xy=({x:+.3f},{y:+.3f}) "
                f"EKF_yaw={yaw:+.3f} "
                f"IMU_yaw={local_imu_yaw:+.3f} "
                f"yaw_err={self.last_yaw_error:+.3f} "
                f"gyro_z={gyro_z:+.3f} "
                f"wheel_w={self.last_omega:+.3f} "
                f"w_err={self.last_yaw_rate_error:+.3f} "
                f"{error_text}"
            )
            self.last_diagnostic_log = timestamp

    def drive(self, v: float, omega: float) -> None:
        """Drive Fabtino using its mecanum kinematics with vy fixed to zero."""
        v = max(-MAX_LINEAR_SPEED, min(MAX_LINEAR_SPEED, clean(v)))
        omega = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, clean(omega)))

        # With vy=0: k = L + W and wheel speeds are [v-k*w, v+k*w,
        # v-k*w, v+k*w] / r for [FL, FR, RL, RR].
        k = FABTINO_KINEMATIC_LEVER_M
        fl = (v - k * omega) / WHEEL_RADIUS
        fr = (v + k * omega) / WHEEL_RADIUS
        rl = (v - k * omega) / WHEEL_RADIUS
        rr = (v + k * omega) / WHEEL_RADIUS

        self.motors["fl"].setVelocity(fl)
        self.motors["fr"].setVelocity(fr)
        self.motors["rl"].setVelocity(rl)
        self.motors["rr"].setVelocity(rr)

    def scan(self, integrate_map: bool = True) -> list[list[float]]:
        if not self.scanning:
            return []

        ranges = self.lidar.getRangeImage()
        if not ranges:
            return []

        n = int(self.lidar.getHorizontalResolution())
        fov = float(self.lidar.getFov())
        pose = self.ekf.pose
        robot_points: list[list[float]] = []

        sampled = 0
        valid = 0
        lidar_min = max(MIN_LIDAR_RANGE, float(self.lidar.getMinRange()))
        lidar_max = min(MAX_LIDAR_RANGE, float(self.lidar.getMaxRange()))

        for i in range(0, n, LIDAR_SUBSAMPLE):
            sampled += 1
            d = clean(ranges[i], -1.0)
            if d < lidar_min or d > lidar_max:
                continue
            valid += 1

            ray_angle = (i / n - 0.5) * fov
            robot_points.append([
                d * math.cos(ray_angle),
                LIDAR_ANGLE_SIGN * d * math.sin(ray_angle),
            ])

        front_ranges = [
            math.hypot(robot_x, robot_y)
            for robot_x, robot_y in robot_points
            if robot_x > 0.0 and abs(robot_y) <= FRONT_HALF_WIDTH
        ]
        self.front_obstacle_distance_m = min(front_ranges) if front_ranges else None

        scan_timestamp = self.robot.getTime()
        if self.known_map_active and self.known_map_localizer is not None:
            measurement = self.known_map_localizer.match(robot_points, pose, scan_timestamp)
            self.last_scan_match_status = self.known_map_localizer.last_status
            self.last_scan_match_error_m = self.known_map_localizer.last_error_m
        else:
            measurement = self.scan_matcher.match(
                robot_points, pose, scan_timestamp,
                linear_velocity_mps=self.last_v,
                angular_velocity_rps=self.last_omega,
                gyro_z_rps=self.last_gyro_z,
            )
            self.last_scan_match_status = self.scan_matcher.last_status
            self.last_scan_match_error_m = self.scan_matcher.last_error_m
        if measurement is not None:
            if measurement.source == "known_map_global_localizer":
                self.ekf.set_pose_exact(measurement)
                # The global match changed the world reference. Never let a
                # scan captured before that reset participate in the next
                # scan-to-scan correction.
                self.scan_matcher.reset_reference()
            else:
                self.ekf.update_pose(measurement)
            self.scan_match_corrections += 1
            pose = self.ekf.pose
            # For ordinary local matching, the stored scan is the current scan
            # and its associated pose must include the correction. A global
            # reset intentionally leaves the scan matcher history empty.
            if measurement.source != "known_map_global_localizer":
                self.scan_matcher.previous_pose = pose
            if DIAGNOSTIC_LOGGING:
                print(
                    f"[SCAN_MATCH] accepted error={self.last_scan_match_error_m:.3f}m "
                    f"pose=({pose[0]:+.3f},{pose[1]:+.3f},{pose[2]:+.3f})"
                )

        points: list[list[float]] = []
        self.dynamic_points = []
        if self.known_map_active:
            self.nav_grid.restore_static()
        c, s = math.cos(pose[2]), math.sin(pose[2])
        for robot_x, robot_y in robot_points:
            wx = pose[0] + c * robot_x - s * robot_y
            wy = pose[1] + s * robot_x + c * robot_y
            # In known-image mode, preserve the uploaded map and add only
            # current-scan returns that do not correspond to static walls.
            is_dynamic = self.known_map_active and self.known_map_localizer is not None and not self.known_map_localizer.is_static_hit(wx, wy)
            if self.known_map_active and is_dynamic:
                self.dynamic_points.append([wx, wy, self.get_lidar_world_z()])
                self.nav_grid.set_dynamic_occupied(wx, wy)
            # Autonomy uses this controller-side copy, not browser state.
            if integrate_map and not self.known_map_active:
                self.nav_grid.update_ray(pose[0], pose[1], wx, wy)
            points.append([wx, wy, self.get_lidar_world_z()])

        self.last_lidar_timestamp = scan_timestamp
        self.last_lidar_valid_fraction = valid / sampled if sampled else 0.0
        self.total_points += len(points)
        return points

    def get_lidar_world_z(self) -> float:
        """Return the LiDAR sensor height in the Webots world frame."""
        try:
            if self.self_node is not None:
                translation = self.self_node.getField("translation").getSFVec3f()
                if len(translation) >= 3:
                    return clean(translation[2]) + FABTINO_BODY_SLOT_Z_M + FABTINO_LIDAR_MOUNT_POSE_Z_M
        except Exception:
            pass
        return FABTINO_BODY_SLOT_Z_M + FABTINO_LIDAR_MOUNT_POSE_Z_M

    def reset_localization(self) -> None:
        timestamp = self.robot.getTime()
        raw_imu_yaw = self.get_imu_yaw()
        self.imu_yaw_reference = raw_imu_yaw if USE_IMU_YAW_REFERENCE else 0.0
        self.ekf.initialize(timestamp, INITIAL_X, INITIAL_Y, INITIAL_YAW, self.get_gyro_z())
        self.truth_reference = self.get_simulation_pose_raw()
        self.truth_error_count = 0
        self.truth_sum_sq_position = 0.0
        self.truth_sum_sq_yaw = 0.0
        self.truth_max_position = 0.0
        self.last_imu_yaw_raw = raw_imu_yaw
        self.last_imu_yaw_local = self.local_imu_yaw(raw_imu_yaw)
        self._reset_encoder_reference(timestamp)
        self.last_v = 0.0
        self.last_omega = 0.0
        self.safety_stop_reason = ""
        self.scan_matcher = LidarScanMatcher(self.scan_matcher.config)
        self.last_scan_match_status = "waiting_for_first_scan"
        self.last_scan_match_error_m = None
        self.scan_match_corrections = 0

    def process_pending_actions(self) -> None:
        """Apply device/state actions from the WebSocket thread safely."""
        while True:
            try:
                action = self.pending_actions.get_nowait()
            except queue.Empty:
                return
            if action == "reset_odom":
                self.reset_localization()
            elif isinstance(action, dict):
                self.process_navigation_action(action)

    def process_navigation_action(self, action: dict) -> None:
        """Apply navigation messages on the simulation thread."""
        typ = action.get("type")
        try:
            if typ in ("stop", "nav_stop"):
                self.navigator.stop("stopped")
                print("[NAV] stop requested")
            elif typ == "nav_goal":
                self.navigator.set_goal(float(action["x"]), float(action["y"]))
                print(f"[NAV] goal requested x={float(action['x']):+.2f} y={float(action['y']):+.2f}")
            elif typ == "nav_explore":
                self.navigator.explore()
                print("[NAV] exploration requested")
            elif typ == "nav_map":
                self.navigator.load_map(
                    float(action["resolution"]), float(action["radius"]), action["grid"]
                )
                self.known_map_active = action.get("mode", "known_image") == "known_image"
                if self.known_map_active:
                    self.navigator.stop("known_map_localizing")
                    self.known_map_request_id = str(action.get("request_id", "")) or None
                    self.known_map_localizer = KnownMapLocalizer(
                        self.nav_grid.data.copy(), self.nav_grid.resolution, self.nav_grid.radius,
                        KnownMapLocalizerConfig(
                            enabled=bool(KNOWN_MAP_CFG.get("enabled", True)),
                            min_points=int(KNOWN_MAP_CFG.get("min_points", 20)),
                            max_global_points=int(KNOWN_MAP_CFG.get("max_global_points", 100)),
                            max_refine_points=int(KNOWN_MAP_CFG.get("max_refine_points", 300)),
                            search_xy_radius_m=float(KNOWN_MAP_CFG.get("search_xy_radius_m", 0.4)),
                            search_yaw_radius_rad=float(KNOWN_MAP_CFG.get("search_yaw_radius_rad", 0.3)),
                            coarse_xy_step_m=float(KNOWN_MAP_CFG.get("coarse_xy_step_m", 0.1)),
                            coarse_yaw_step_rad=float(KNOWN_MAP_CFG.get("coarse_yaw_step_rad", 0.1)),
                            fine_xy_step_m=float(KNOWN_MAP_CFG.get("fine_xy_step_m", 0.025)),
                            fine_yaw_step_rad=float(KNOWN_MAP_CFG.get("fine_yaw_step_rad", 0.035)),
                            occupied_tolerance_m=float(KNOWN_MAP_CFG.get("occupied_tolerance_m", 0.12)),
                            min_static_hit_fraction=float(KNOWN_MAP_CFG.get("min_static_hit_fraction", 0.2)),
                             sigma_xy_m=float(KNOWN_MAP_CFG.get("sigma_xy_m", 0.06)),
                             sigma_yaw_rad=float(KNOWN_MAP_CFG.get("sigma_yaw_rad", 0.05)),
                             global_xy_step_m=float(KNOWN_MAP_CFG.get("global_xy_step_m", 0.40)),
                             global_yaw_step_rad=float(KNOWN_MAP_CFG.get("global_yaw_step_rad", 0.30)),
                             global_refine_xy_radius_m=float(KNOWN_MAP_CFG.get("global_refine_xy_radius_m", 0.30)),
                             global_refine_yaw_radius_rad=float(KNOWN_MAP_CFG.get("global_refine_yaw_radius_rad", 0.30)),
                             global_refine_xy_step_m=float(KNOWN_MAP_CFG.get("global_refine_xy_step_m", 0.05)),
                             global_refine_yaw_step_rad=float(KNOWN_MAP_CFG.get("global_refine_yaw_step_rad", 0.05)),
                            global_min_static_hit_fraction=float(KNOWN_MAP_CFG.get("global_min_static_hit_fraction", 0.90)),
                            mean_distance_threshold_m=float(KNOWN_MAP_CFG.get("mean_distance_threshold_m", 0.06)),
                            distance_sigma_m=float(KNOWN_MAP_CFG.get("distance_sigma_m", 0.06)),
                            global_min_free_space_fraction=float(KNOWN_MAP_CFG.get("global_min_free_space_fraction", 0.85)),
                            sector_coverage_min=float(KNOWN_MAP_CFG.get("sector_coverage_min", 0.80)),
                            min_good_sectors=int(KNOWN_MAP_CFG.get("min_good_sectors", 6)),
                            best_second_difference_min=float(KNOWN_MAP_CFG.get("best_second_difference_min", 0.04)),
                            free_space_sample_step_m=float(KNOWN_MAP_CFG.get("free_space_sample_step_m", 0.10)),
                            candidate_count=int(KNOWN_MAP_CFG.get("candidate_count", 8)),
                          ),
                    )
                else:
                    self.known_map_request_id = None
                print(f"[NAV] known map loaded size={self.nav_grid.size} resolution={self.nav_grid.resolution:.3f}m")
            elif typ == "nav_clear_map":
                self.nav_grid.clear()
                self.navigator.path = []
                self.known_map_active = False
                self.known_map_localizer = None
                self.known_map_request_id = None
                self.scan_matcher = LidarScanMatcher(self.scan_matcher.config)
                self.dynamic_points = []
                print("[NAV] controller map cleared")
        except (KeyError, TypeError, ValueError) as exc:
            print(f"[NAV] invalid action: {exc}")

    def safety_state(self) -> tuple[bool, str]:
        if not self.ekf.is_valid(MAX_STATE_COV_XY, MAX_STATE_COV_YAW):
            return False, "EKF covariance/state invalid"
        now = self.robot.getTime()
        if self.last_sensor_timestamp > 0 and now - self.last_sensor_timestamp > SENSOR_TIMEOUT:
            return False, "sensor timeout"
        if self.scanning and self.last_lidar_timestamp > 0:
            if now - self.last_lidar_timestamp > SENSOR_TIMEOUT:
                return False, "LiDAR timeout"
            if MIN_LIDAR_VALID_FRACTION > 0.0 and self.last_lidar_valid_fraction < MIN_LIDAR_VALID_FRACTION:
                return False, "insufficient valid LiDAR returns"
        return True, ""

    async def ws_client(self, websocket):
        self.clients.add(websocket)
        try:
            async for raw in websocket:
                try:
                    msg = json.loads(raw)
                    typ = msg.get("type")
                    if typ == "drive":
                        self.pending_actions.put({"type": "nav_stop"})
                        self.web_v = max(-MAX_LINEAR_SPEED, min(MAX_LINEAR_SPEED, clean(msg.get("v"))))
                        self.web_w = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, clean(msg.get("omega"))))
                        self.last_web_command = time.monotonic()
                    elif typ == "stop":
                        self.pending_actions.put({"type": "nav_stop"})
                        self.web_v = 0.0
                        self.web_w = 0.0
                        self.last_web_command = time.monotonic()
                    elif typ == "set_scan":
                        self.scanning = bool(msg.get("enabled", True))
                    elif typ == "toggle_scan":
                        self.scanning = not self.scanning
                    elif typ == "reset_odom":
                        self.pending_actions.put("reset_odom")
                    elif typ in ("nav_goal", "nav_explore", "nav_stop", "nav_map", "nav_clear_map"):
                        self.pending_actions.put(msg)
                except Exception as exc:
                    print("[WS] message error:", exc)
        finally:
            self.clients.discard(websocket)
            self.web_v = 0.0
            self.web_w = 0.0

    async def ws_broadcast(self, payload: dict) -> None:
        if not self.clients:
            return
        raw = json.dumps(payload, allow_nan=False)
        dead = []
        for client in list(self.clients):
            try:
                await client.send(raw)
            except Exception:
                dead.append(client)
        for client in dead:
            self.clients.discard(client)

    def ws_thread(self) -> None:
        async def server():
            async with websockets.serve(
                self.ws_client,
                WS_HOST,
                WS_PORT,
                ping_interval=20,
                ping_timeout=20,
                max_size=8 * 1024 * 1024,
            ):
                print("[WS] Server listening on ws://0.0.0.0:8765")
                await asyncio.Future()

        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(server())

    def broadcast_from_robot_thread(self, payload: dict) -> None:
        if self.loop and self.loop.is_running():
            asyncio.run_coroutine_threadsafe(self.ws_broadcast(payload), self.loop)

    def packet(self, points: list[list[float]]) -> dict:
        roll, pitch, imu_yaw = self.get_imu_rpy()
        gx, gy, gz = self.get_gyro()
        ax, ay, az = self.get_acceleration()
        x, y, theta = self.ekf.pose
        covariance = self.ekf.P
        truth = self.get_simulation_pose_local()
        error = self.estimator_error()
        metrics = {
            "samples": self.truth_error_count,
            "position_rmse_m": math.sqrt(self.truth_sum_sq_position / self.truth_error_count) if self.truth_error_count else None,
            "yaw_rmse_rad": math.sqrt(self.truth_sum_sq_yaw / self.truth_error_count) if self.truth_error_count else None,
            "max_position_error_m": self.truth_max_position if self.truth_error_count else None,
        }

        return {
            "type": "scan",
            "scanning": self.scanning,
            "robot": {"x": x, "y": y, "theta": theta},
            "ground_truth": {"x": truth[0], "y": truth[1], "theta": truth[2]} if truth else None,
            "estimation_error": error,
            "estimator_metrics": metrics,
            "configuration": {
                "map_resolution_m": float(VIEW_CFG.get("map_resolution_m", NAV_RESOLUTION)),
                "map_radius_m": float(VIEW_CFG.get("map_radius_m", NAV_RADIUS)),
                "point_voxel_size_m": float(VIEW_CFG.get("point_voxel_size_m", NAV_RESOLUTION)),
                "mapping_evidence": {
                    "free_evidence_step": int(MAP_CFG.get("free_evidence_step", 1)),
                    "occupied_evidence_step": int(MAP_CFG.get("occupied_evidence_step", 1)),
                    "free_threshold": int(MAP_CFG.get("free_threshold", -3)),
                    "occupied_threshold": int(MAP_CFG.get("occupied_threshold", 4)),
                },
                "teleop_max_linear_speed_mps": MAX_LINEAR_SPEED,
                "teleop_max_angular_speed_rps": MAX_ANGULAR_SPEED,
                "accelerometer_enabled": ACCELEROMETER_ENABLED,
                "accelerometer_std_mps2": ACCELEROMETER_STD,
            },
            "scan_matching": {
                "enabled": self.known_map_localizer.config.enabled if self.known_map_localizer else self.scan_matcher.config.enabled,
                "status": self.last_scan_match_status,
                "match_error_m": self.last_scan_match_error_m,
                "metrics": self.known_map_localizer.last_metrics if self.known_map_localizer else {},
                "corrections": self.scan_match_corrections,
                "mode": (
                    "known_map_global"
                    if self.known_map_active and self.known_map_localizer and not self.known_map_localizer.global_localized
                    else "known_map_local"
                    if self.known_map_active
                    else "scan_to_scan"
                ),
                "known_map_global_localized": bool(
                    self.known_map_localizer and self.known_map_localizer.global_localized
                ),
                "known_map_request_id": self.known_map_request_id,
            },
            "estimator": {
                "name": "EKF",
                "valid": self.ekf.is_valid(MAX_STATE_COV_XY, MAX_STATE_COV_YAW),
                "velocity": self.ekf.x[3].item(),
                "angular_velocity": self.ekf.x[4].item(),
                "gyro_bias_z": self.ekf.x[5].item(),
                "covariance_trace": self.ekf.covariance_trace,
                "covariance_diag": [float(v) for v in np_diag(covariance)],
            },
            "imu": {
                "available": bool(self.imu),
                "roll": roll,
                "pitch": pitch,
                "yaw": imu_yaw,
                "acceleration": [ax, ay, az],
                "gyro": [gx, gy, gz],
            },
            "odometry": {
                "linear_velocity": self.last_v,
                "angular_velocity": self.last_omega,
            },
            "navigation": {
                "mode": self.navigator.mode,
                "status": self.navigator.status,
                "goal": list(self.navigator.active_target) if self.navigator.active_target else None,
                "safety_margin_m": NAV_MARGIN,
                "frontier_count": self.nav_grid.frontier_count(),
                "path_cells": len(self.navigator.path),
                "command_requested": list(self.last_command_requested),
                "command_applied": list(self.last_command_applied),
            },
            "diagnostics": {
                "imu_yaw_raw": self.last_imu_yaw_raw,
                "imu_yaw_local": self.last_imu_yaw_local,
                "ekf_yaw": theta,
                "yaw_error_rad": self.last_yaw_error,
                "gyro_z_rps": self.last_gyro_z,
                "wheel_angular_velocity_rps": self.last_omega,
                "yaw_rate_error_rps": self.last_yaw_rate_error,
                "map_frame": "local XY, startup heading = 0 rad",
                "lidar_transform": "LiDAR(robot) -> local map(world) exactly once",
            },
            "new_points": points,
            "dynamic_points": self.dynamic_points,
            "known_map_mode": self.known_map_active,
            "points_frame": "world",
            "frame_contract": {
                "world": "x/y horizontal Webots world frame",
                "robot": "+x forward, +y left, +yaw counter-clockwise",
                "transform": "world = pose ⊕ robot_point",
            },
            "total_points": self.total_points,
            "timestamp": self.robot.getTime(),
        }

    def keyboard_command(self) -> tuple[float, float]:
        v = 0.0
        w = 0.0
        key = self.keyboard.getKey()
        if key in (Keyboard.UP, ord("w"), ord("W")):
            v = MAX_LINEAR_SPEED
        elif key in (Keyboard.DOWN, ord("s"), ord("S")):
            v = -MAX_LINEAR_SPEED
        elif key in (Keyboard.LEFT, ord("a"), ord("A")):
            w = MAX_ANGULAR_SPEED
        elif key in (Keyboard.RIGHT, ord("d"), ord("D")):
            w = -MAX_ANGULAR_SPEED
        return v, w

    def log_navigation_debug(self, timestamp: float, command: tuple[float, float]) -> None:
        if not NAV_DEBUG:
            return
        frontier_count = self.nav_grid.frontier_count()
        signature = (self.navigator.mode, self.navigator.status, frontier_count,
                     len(self.navigator.path))
        periodic = timestamp - self.last_nav_debug_log >= DIAGNOSTIC_LOG_PERIOD
        if signature != self.last_nav_debug_signature or periodic:
            goal = self.navigator.active_target
            goal_text = f"({goal[0]:+.2f},{goal[1]:+.2f})" if goal else "-"
            print(
                f"[NAV] mode={self.navigator.mode} status={self.navigator.status} "
                f"frontiers={frontier_count} path_cells={len(self.navigator.path)} "
                f"goal={goal_text} requested=({command[0]:+.3f},{command[1]:+.3f}) "
                f"applied=({self.last_command_applied[0]:+.3f},{self.last_command_applied[1]:+.3f})"
                f"{(' SAFETY=' + self.safety_stop_reason) if self.safety_stop_reason else ''}"
            )
            self.last_nav_debug_signature = signature
            self.last_nav_debug_log = timestamp

    def run(self) -> None:
        threading.Thread(target=self.ws_thread, daemon=True).start()

        latest_points: list[list[float]] = []
        localization_every = max(1, int(round(LOCALIZATION_SCAN_PERIOD_S / (self.timestep / 1000.0))))
        mapping_every = max(1, int(round(MAPPING_UPDATE_PERIOD_S / (self.timestep / 1000.0))))
        viewer_every = max(1, int(round(VIEWER_UPDATE_PERIOD_S / (self.timestep / 1000.0))))

        print(
            f"[CONFIG] scan periods: localization={localization_every * self.timestep / 1000.0:.3f}s "
            f"mapping={mapping_every * self.timestep / 1000.0:.3f}s "
            f"viewer={viewer_every * self.timestep / 1000.0:.3f}s"
        )

        while self.robot.step(self.timestep) != -1:
            self.frame += 1
            self.process_pending_actions()
            kv, kw = self.keyboard_command()

            if self.navigator.mode != "idle":
                v, w = self.navigator.command(self.ekf.pose)
            elif kv != 0.0 or kw != 0.0:
                v, w = kv, kw
            elif time.monotonic() - self.last_web_command <= COMMAND_TIMEOUT:
                v, w = self.web_v, self.web_w
            else:
                v, w = 0.0, 0.0

            self.last_command_requested = (v, w)

            valid, reason = self.safety_state()
            if not valid:
                v = 0.0
                w = 0.0
                self.safety_stop_reason = reason
            else:
                self.safety_stop_reason = ""

            if (v > 0.0 and self.front_obstacle_distance_m is not None
                    and self.front_obstacle_distance_m <= FRONT_STOP_DISTANCE):
                v = 0.0
                self.safety_stop_reason = (
                    f"front obstacle at {self.front_obstacle_distance_m:.2f}m"
                )

            self.last_command_applied = (v, w)
            if self.safety_stop_reason != self.last_safety_log_reason:
                if self.safety_stop_reason:
                    print(
                        f"[SAFETY] motion blocked: {self.safety_stop_reason}; "
                        f"navigation remains {self.navigator.mode}/{self.navigator.status}"
                    )
                elif self.last_safety_log_reason:
                    print("[SAFETY] motion released")
                self.last_safety_log_reason = self.safety_stop_reason
            self.log_navigation_debug(self.robot.getTime(), self.last_command_requested)
            self.drive(v, w)
            timestamp = self.robot.getTime()
            self.update_localization(timestamp)

            if self.frame % localization_every == 0:
                latest_points = self.scan(integrate_map=(self.frame % mapping_every == 0))

            if self.frame % viewer_every == 0:
                payload = self.packet(latest_points)
                latest_points = []
                payload["safety"] = {
                    "stop_active": bool(self.safety_stop_reason),
                "reason": self.safety_stop_reason,
                    "front_obstacle_distance_m": self.front_obstacle_distance_m,
                }
                self.broadcast_from_robot_thread(payload)


def np_diag(matrix):
    # Small local helper avoids exposing numpy details through the packet API.
    try:
        return matrix.diagonal()
    except Exception:
        return [0.0] * 6


if __name__ == "__main__":
    FabtinoController().run()
