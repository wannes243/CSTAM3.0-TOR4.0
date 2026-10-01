"""Mask, strict-JSON transport and ROS callback regressions without Webots/DDS."""
import importlib.util
import json
import math
from pathlib import Path
import queue
import sys
import tempfile
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT = Path(__file__).parents[1]
CONTROLLER = ROOT/'webots/controllers/fabtino_webots_bridge_controller'
sys.path.insert(0, str(CONTROLLER))
from lidar_mask import AngularMask, load_mask
import test_rotation_mapping as ros_support
import test_safety_arbitration as safety_support


def device(fov=math.pi, n=19, layers=1, image=None):
    return NS(getFov=lambda: fov, getHorizontalResolution=lambda: n,
              getNumberOfLayers=lambda: layers,
              getRangeImage=lambda: image if image is not None else [2.]*(n*layers),
              getMinRange=lambda: .2, getMaxRange=lambda: 12.)


class LidarMaskTests(unittest.TestCase):
    def test_empty_returns_an_unmodified_copy(self):
        raw = [1., 2., math.inf, -1., float('nan')]
        out = AngularMask(math.pi, 5, 1, []).filter_scan(raw)
        self.assertIsNot(out, raw)
        self.assertEqual(out[:4], raw[:4])
        self.assertTrue(math.isnan(out[4]))

    def test_yaml_missing_key_and_empty_list_keep_all_rays(self):
        for content in ('', '{}', 'robot: {}', 'lidar: {}', 'lidar:\n  masked_angle_ranges: []'):
            with self.subTest(config=content), tempfile.TemporaryDirectory() as directory:
                config = Path(directory)/'robot.yaml'
                config.write_text(content)
                self.assertEqual(load_mask(device(), config).masked_indices, set())

    def test_yaml_loads_requested_intervals(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory)/'robot.yaml'
            config.write_text('lidar:\n  masked_angle_ranges:\n    - [10, 30]\n    - [100, 120]\n')
            mask = load_mask(device(fov=2*math.pi, n=361), config)
            self.assertEqual(mask.masked_indices, set(range(190, 211)) | set(range(280, 301)))

    def test_invalid_yaml_structure_is_rejected(self):
        for content in ('[1, 2]', 'lidar: null', 'lidar:\n  masked_angle_ranges: null', 'lidar: ['):
            with self.subTest(config=content), tempfile.TemporaryDirectory() as directory:
                import yaml
                config = Path(directory)/'robot.yaml'
                config.write_text(content)
                with self.assertRaises((ValueError, yaml.YAMLError)):
                    load_mask(device(), config)

    def test_positive_and_negative_intervals_include_boundaries(self):
        mask = AngularMask(math.pi, 19, 1, [[10, 30], [-30, -10]])
        self.assertEqual(mask.masked_indices, {6, 7, 8, 10, 11, 12})

    def test_boundary_tolerance_is_one_microdegree(self):
        mask = AngularMask(math.pi, 19, 1, [[10.+.5e-6, 20.-.5e-6]])
        self.assertEqual(mask.masked_indices, {10, 11})
        self.assertEqual(AngularMask(math.pi, 19, 1, [[10.+2e-6, 20.-2e-6]]).masked_indices, set())

    def test_degenerate_and_sub_step_intervals(self):
        self.assertEqual(AngularMask(math.pi, 19, 1, [[10, 10]]).masked_indices, {10})
        self.assertEqual(AngularMask(math.pi, 19, 1, [[10.1, 10.2]]).masked_indices, set())

    def test_full_fov_masks_every_layer_without_mutation(self):
        raw = list(range(21))
        mask = AngularMask(math.pi, 7, 3, [[-90, 90]])
        out = mask.filter_scan(raw)
        self.assertEqual(len(out), 21)
        self.assertTrue(all(value == math.inf for value in out))
        self.assertEqual(raw, list(range(21)))

    def test_partial_mask_repeats_at_same_column_on_all_layers(self):
        mask = AngularMask(math.pi, 7, 3, [[0, 30]])
        raw = [2.+i for i in range(21)]
        out = mask.filter_scan(raw)
        for k, value in enumerate(out):
            self.assertEqual(value, math.inf if k % 7 in (3, 4) else raw[k])

    def test_overlap_and_touching_bounds_warn_and_merge(self):
        with self.assertWarnsRegex(UserWarning, 'merged'):
            mask = AngularMask(math.pi, 19, 1, [[10, 30], [20, 40], [10, 30], [40, 50]])
        self.assertEqual(mask.intervals, ((10., 50.),))
        self.assertEqual(len(mask.flat_indices), len(set(mask.flat_indices)))

    def test_invalid_configuration_is_rejected(self):
        invalid = [None, {}, '10,30', [[1]], [[1, 2, 3]], ['12'],
                   [[True, 10]], [['1', 10]], [[None, 10]], [[30, 10]],
                   [[-91, 0]], [[0, 91]], [[0, float('inf')]], [[float('nan'), 10]]]
        for ranges in invalid:
            with self.subTest(ranges=ranges), self.assertRaises(ValueError):
                AngularMask(math.pi, 19, 1, ranges)
        with self.assertRaises(ValueError):
            AngularMask(2*math.pi, 361, 1, [[-181, 0]])

    def test_runtime_device_geometry_and_single_ray(self):
        self.assertEqual(AngularMask(math.pi/2, 10, 1, [[-45, -35]]).masked_indices, {0, 1})
        # N=1 follows the requested formula: theta_0 = -FOV/2, step=0.
        self.assertEqual(AngularMask(math.pi/2, 1, 1, [[-45, -45]]).filter_scan([2.]), [math.inf])
        self.assertEqual(AngularMask(math.pi/2, 1, 1, [[0, 0]]).filter_scan([2.]), [2.])

    def test_invalid_geometry_and_changed_shape_are_rejected(self):
        for fov, n, layers in ((0, 2, 1), (float('nan'), 2, 1), (7, 2, 1), (1, 0, 1), (1, 2, 0)):
            with self.subTest(geometry=(fov, n, layers)), self.assertRaises(ValueError):
                AngularMask(fov, n, layers, [])
        with self.assertRaises(ValueError):
            AngularMask(math.pi, 3, 2, []).filter_scan([1., 2., 3.])


class LidarMaskPipelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        ros_support.RotationMappingTests.setUpClass()
        safety_support.SafetyArbitrationTests.setUpClass()
        controller_stub = ModuleType('controller')
        controller_stub.Supervisor = NS
        with patch.dict(sys.modules, {'controller': controller_stub}):
            spec = importlib.util.spec_from_file_location('_masked_controller', CONTROLLER/'fabtino_webots_bridge_controller.py')
            cls.module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(cls.module)

    def sensor(self, mask, raw):
        controller = object.__new__(self.module.Controller)
        controller.robot = NS(getTime=lambda: .1)
        controller.encoders = {}
        controller.imu = controller.gyro = controller.accel = None
        controller.scanning = True
        controller.lidar_mask = mask
        controller.lidar = device(mask.fov_rad, mask.resolution, mask.layers, raw)
        controller.gt = lambda: None
        controller.seq = 0
        return controller

    def test_bad_mask_aborts_controller_startup_before_transport(self):
        motor = NS(setPosition=lambda value: None, setVelocity=lambda value: None, enable=lambda value: None)
        lidar = device()
        lidar.enable = lambda value: None
        robot = NS(getBasicTimeStep=lambda: 16, getSelf=lambda: None,
                   getDevice=lambda name: lidar if name == 'mapping_lidar' else motor)
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory)/'robot.yaml'
            config.write_text('lidar:\n  masked_angle_ranges: [[30, 10]]\n')
            with patch.object(self.module, 'Supervisor', return_value=robot), \
                 patch.object(self.module, 'DEFAULT_CONFIG_PATH', config), \
                 patch.object(self.module, 'BridgeTransport') as transport:
                with self.assertRaisesRegex(ValueError, 'start must be'):
                    self.module.Controller()
                transport.assert_not_called()

    def bridge_scan(self, packet):
        support = ros_support.RotationMappingTests
        bridge = object.__new__(support.Bridge)
        bridge.rx = queue.Queue()
        bridge.rx.put(json.loads(self.module.encode_sensor_packet(packet)))
        bridge.last_sensor_stamp_ns = bridge.last_scan_sim_time = None
        bridge.scan_period, bridge.seq = 0., 0
        bridge.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: ros_support.stamp(0)))
        for name in ('pub_joints', 'pub_imu', 'pub_scan', 'pub_gt'):
            setattr(bridge, name, ros_support.Publisher())
        bridge.process_rx()
        return bridge.pub_scan.messages[-1]

    def test_controller_filters_before_use_and_serialization_keeps_slots(self):
        raw = [2.]*19
        controller = self.sensor(AngularMask(math.pi, 19, 1, [[-10, 10]]), raw)
        # A scan tick must not reload config/recompute interval membership.
        with patch.object(self.module, 'load_mask', side_effect=AssertionError('per-tick load')):
            packet = controller.sensor_packet()
        self.assertEqual(packet['lidar']['ranges'][8:11], [math.inf]*3)
        wire = self.module.encode_sensor_packet(packet)
        self.assertNotIn(b'Infinity', wire)
        self.assertEqual(json.loads(wire)['lidar']['ranges'][8:11], [None]*3)
        self.assertEqual(raw, [2.]*19)
        scan = self.bridge_scan(packet)
        self.assertEqual(len(scan.ranges), 19)
        self.assertEqual(scan.ranges[8:11], [math.inf]*3)
        self.assertAlmostEqual(scan.angle_increment, math.pi/18)

    def test_reload_rebuilds_mask_and_invalid_reload_does_not_replace_it(self):
        controller = self.sensor(AngularMask(math.pi, 19, 1, []), [2.]*19)
        with tempfile.TemporaryDirectory() as directory:
            controller.config_path = Path(directory)/'robot.yaml'
            controller.config_path.write_text('lidar:\n  masked_angle_ranges: [[0, 0]]\n')
            controller.reload_lidar_mask()
            self.assertEqual(controller.lidar_mask.masked_indices, {9})
            controller.config_path.write_text('lidar:\n  masked_angle_ranges: [[30, 10]]\n')
            with self.assertRaises(ValueError):
                controller.reload_lidar_mask()
            self.assertEqual(controller.lidar_mask.masked_indices, {9})
            controller.config_path.write_text('lidar:\n  masked_angle_ranges: []\n')
            controller.reload_lidar_mask()
            self.assertEqual(controller.lidar_mask.masked_indices, set())
            controller.lidar = device(fov=math.pi/2, n=10, layers=2)
            controller.config_path.write_text('lidar:\n  masked_angle_ranges: [[-45, -35]]\n')
            controller.reload_lidar_mask()
            self.assertEqual(controller.lidar_mask.masked_indices, {0, 1})
            self.assertEqual(controller.lidar_mask.flat_indices, (0, 1, 10, 11))

    def test_multilayer_transport_keeps_shape_and_central_scan_keeps_mask(self):
        controller = self.sensor(AngularMask(math.pi, 5, 3, [[0, 0]]), [2.]*5+[1.]*5+[.5]*5)
        packet = controller.sensor_packet()
        self.assertEqual(len(packet['lidar']['ranges']), 15)
        scan = self.bridge_scan(packet)
        self.assertEqual(scan.ranges, [1., 1., math.inf, 1., 1.])
        self.assertAlmostEqual(scan.angle_increment, math.pi/4)

    def test_masked_ray_never_becomes_map_point_obstacle_or_localization_input(self):
        support = ros_support.RotationMappingTests
        # 41 columns; centre return would be a near obstacle without the mask.
        raw = [2.]*41
        raw[20] = .3
        packet = self.sensor(AngularMask(math.pi, 41, 1, [[0, 0]]), raw).sensor_packet()
        scan = self.bridge_scan(packet)
        with patch.dict(sys.modules, support.modules):
            mapper = support.Mapping()
            mapper.lidar_subsample = 1
            mapper.integrate_scan(1, scan, (0., 0., 0.), 'test')
            self.assertEqual(mapper.pub_points.messages[-1].width, 0)
            self.assertEqual(mapper.pub_live_points.messages[-1].width, 40)
            gx, gy = mapper.grid.cell(.3, 0.)
            self.assertNotEqual(mapper.grid.data[gy, gx], 100)
            localization = support.Localization()
            calls = []
            localization.matcher.match = lambda points, *args, **kwargs: calls.append(points)
            localization.process_scan(scan)
            self.assertEqual(len(calls[0]), 40)
            self.assertTrue(all(math.hypot(*p) > 1.9 for p in calls[0]))
            safety = safety_support.SafetyArbitrationTests.Safety()
            safety.now_ns = scan.header.stamp.sec*1_000_000_000+scan.header.stamp.nanosec
            safety.last_scan = scan
            safety.last_odom = ros_support.odometry(safety.now_ns)
            safety.nav_cb(safety_support.command(safety.now_ns*1e-9, v=.1))
            safety.loop()
            self.assertEqual(safety.pub.messages[-1].linear.x, .1)

    def test_all_masked_scan_produces_empty_cloud(self):
        support = ros_support.RotationMappingTests
        packet = self.sensor(AngularMask(math.pi, 19, 1, [[-90, 90]]), [.3]*19).sensor_packet()
        scan = self.bridge_scan(packet)
        with patch.dict(sys.modules, support.modules):
            mapper = support.Mapping()
            mapper.integrate_scan(1, scan, (0., 0., 0.), 'test')
            self.assertEqual(mapper.pub_points.messages[-1].width, 0)
            self.assertFalse((mapper.grid.data == 100).any())


if __name__ == '__main__':
    unittest.main()
