"""Acquisition-time regression tests using real fusion/mapping, with ROS I/O stubs.

Run without ROS: python -m unittest discover -s tests -p test_rotation_mapping.py -v
NumPy is required. These tests do not replace a live Webots/DDS test.
"""
import importlib.util
import math
from pathlib import Path
import queue
import struct
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

# Load the native extension before patch.dict restores the ROS import stubs.
# Unloading/reimporting NumPy via sys.modules is unsupported.
import numpy

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / 'ros_ws/src/fabtino_core'))


def stamp(ns=0):
    return NS(sec=ns//1_000_000_000, nanosec=ns % 1_000_000_000)


def vector():
    return NS(x=0., y=0., z=0.)


def header():
    return NS(stamp=stamp(), frame_id='')


def orientation(yaw=0.):
    return NS(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2))


def odometry(ns=0, yaw=0.):
    return NS(header=NS(stamp=stamp(ns), frame_id='map'), child_frame_id='base_link',
              pose=NS(pose=NS(position=vector(), orientation=orientation(yaw)), covariance=[0.]*36),
              twist=NS(twist=NS(linear=vector(), angular=vector())))


def scan(ns=0, distance=2., angle=0.):
    return NS(header=NS(stamp=stamp(ns), frame_id='lidar_link'),
              ranges=[distance], angle_min=angle, angle_increment=0., range_min=.1, range_max=12.)


def imu(ns=0, yaw=0., omega=0.):
    return NS(header=NS(stamp=stamp(ns), frame_id='imu_link'),
              orientation=orientation(yaw), angular_velocity=NS(x=0., y=0., z=omega),
              linear_acceleration=vector())


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeNode:
    def __init__(self, name):
        self.parameters = {}
        self.now_ns = 1_790_727_188_000_000_000

    def declare_parameter(self, name, value):
        self.parameters[name] = value

    def get_parameter(self, name):
        return NS(value=self.parameters[name])

    def create_publisher(self, *args):
        return Publisher()

    def create_subscription(self, *args):
        pass

    def create_timer(self, *args):
        pass

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=self.now_ns, to_msg=lambda: stamp(self.now_ns)))

    def get_logger(self):
        return NS(info=lambda m: None, warning=lambda m: None, debug=lambda m: None)


class RotationMappingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.modules = {name: ModuleType(name) for name in (
            'rclpy', 'rclpy.node', 'sensor_msgs', 'sensor_msgs.msg', 'nav_msgs', 'nav_msgs.msg',
            'std_msgs', 'std_msgs.msg', 'std_srvs', 'std_srvs.srv', 'geometry_msgs', 'geometry_msgs.msg', 'tf2_ros')}
        cls.modules['rclpy.node'].Node = FakeNode
        sensor = cls.modules['sensor_msgs.msg']
        sensor.LaserScan, sensor.Imu = scan, imu
        sensor.JointState = lambda: NS(header=header(), position=[])
        sensor.PointCloud2 = lambda: NS(header=header())
        sensor.PointField = type('PointField', (NS,), {'FLOAT32': 7})
        cls.modules['nav_msgs.msg'].Odometry = odometry
        cls.modules['nav_msgs.msg'].OccupancyGrid = lambda: NS(header=header(), info=NS(origin=NS(position=vector(), orientation=orientation())))
        for name in ('String', 'Bool', 'Empty'):
            setattr(cls.modules['std_msgs.msg'], name, NS)
        cls.modules['std_srvs.srv'].Trigger = cls.modules['std_srvs.srv'].SetBool = NS
        cls.modules['geometry_msgs.msg'].Twist = NS
        cls.modules['geometry_msgs.msg'].TransformStamped = lambda: NS(header=header(), transform=NS(translation=vector(), rotation=orientation()))
        cls.modules['tf2_ros'].TransformBroadcaster = lambda node: NS(sendTransform=lambda msg: None)
        with patch.dict(sys.modules, cls.modules):
            for key, package, filename, name in (
                ('Mapping', 'fabtino_mapping', 'mapping_node', 'MappingNode'),
                ('Localization', 'fabtino_localization', 'localization_node', 'LocalizationNode'),
                ('Bridge', 'fabtino_webots_bridge', 'webots_bridge_node', 'WebotsBridgeNode'),
            ):
                spec = importlib.util.spec_from_file_location('_rotation_' + filename,
                    ROOT / 'ros_ws/src' / package / package / (filename + '.py'))
                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)
                setattr(cls, key, getattr(module, name))

    def setUp(self):
        context = patch.dict(sys.modules, self.modules)
        context.start()
        self.addCleanup(context.stop)
        self.mapping = self.Mapping()
        self.mapping.lidar_subsample = 1
        self.mapping.publish_grid = lambda *args: None
        self.mapping.publish_local = lambda: None
        self.epoch = 1_790_727_188_000_000_000

    def fused_sample(self, node, ns, yaw, imu_first=False):
        # Encoder increments exactly match the body turn for this test.
        wheel = yaw*1.043834/(2*.100)
        wheels = NS(header=NS(stamp=stamp(ns)), position=[-wheel, wheel, -wheel, wheel])
        inertial = imu(ns, 1.2+yaw, .5 if yaw else 0.)
        callbacks = [(node.wheel_cb, wheels), (node.imu_cb, inertial)]
        if imu_first:
            callbacks.reverse()
        for callback, message in callbacks:
            callback(message)

    def test_epoch_pose_history_keeps_distinct_samples(self):
        for step in range(10):
            self.mapping.pose_cb(odometry(self.epoch+step*20_000_000, step*.02))
        self.assertEqual(len(self.mapping.pose_history), 10)

    def test_duplicate_pose_stamp_replaces_only_same_sample(self):
        self.mapping.pose_cb(odometry(self.epoch, 0.))
        self.mapping.pose_cb(odometry(self.epoch, .01))
        self.assertEqual(len(self.mapping.pose_history), 1)
        self.assertAlmostEqual(self.mapping.pose_history[0][3], .01)

    def test_old_pose_does_not_replace_current_pose(self):
        self.mapping.pose_cb(odometry(self.epoch+20_000_000, .1))
        self.mapping.pose_cb(odometry(self.epoch, 0.))
        self.assertAlmostEqual(self.mapping.pose[2], .1)

    def test_scan_waits_for_pose_instead_of_using_old_yaw(self):
        self.mapping.pose_cb(odometry(self.epoch, 0.))
        self.mapping.scan_cb(scan(self.epoch+20_000_000))
        self.mapping.update()
        self.assertEqual(len(self.mapping.scan_queue), 1)
        self.assertEqual(len(self.mapping.pub_points.messages), 0)
        self.mapping.pose_cb(odometry(self.epoch+20_000_000, .1))
        self.mapping.update()
        self.assertEqual(len(self.mapping.scan_queue), 0)
        self.assertEqual(len(self.mapping.pub_points.messages), 1)

    def test_scan_before_history_is_rejected_even_if_close(self):
        self.mapping.pose_cb(odometry(self.epoch+20_000_000, .1))
        self.mapping.scan_cb(scan(self.epoch))
        self.mapping.update()
        self.assertEqual(len(self.mapping.scan_queue), 0)
        self.assertEqual(len(self.mapping.pub_points.messages), 0)

    def test_interpolation_requires_small_gap_and_wraps_yaw(self):
        self.mapping.pose_cb(odometry(self.epoch, math.pi-.02))
        self.mapping.pose_cb(odometry(self.epoch+40_000_000, -math.pi+.02))
        pose, reason = self.mapping.pose_for_scan((self.epoch+20_000_000)*1e-9)
        self.assertEqual(reason, 'interpolated_pose')
        self.assertAlmostEqual(abs(pose[2]), math.pi, places=5)
        self.mapping.pose_cb(odometry(self.epoch+1_000_000_000, 0.))
        pose, reason = self.mapping.pose_for_scan((self.epoch+500_000_000)*1e-9)
        self.assertIsNone(pose)
        self.assertEqual(reason, 'pose_gap_too_large')

    def test_each_scan_is_integrated_once(self):
        self.mapping.pose_cb(odometry(self.epoch))
        self.mapping.scan_cb(scan(self.epoch))
        self.mapping.update()
        revision = self.mapping.grid.revision
        self.mapping.update()
        self.assertEqual(self.mapping.grid.revision, revision)
        self.assertEqual(len(self.mapping.pub_points.messages), 1)

    def test_fusion_is_independent_of_wheel_imu_callback_order(self):
        for imu_first in (False, True):
            with self.subTest(imu_first=imu_first):
                node = self.Localization()
                self.fused_sample(node, self.epoch, 0., imu_first)
                self.fused_sample(node, self.epoch+20_000_000, .1, imu_first)
                self.assertAlmostEqual(node.ekf.pose[2], .1, places=8)
                self.assertEqual(node.last_fused_ns, self.epoch+20_000_000)

    def test_unpaired_imu_does_not_publish_mixed_time_pose(self):
        node = self.Localization()
        node.imu_cb(imu(self.epoch, 1.2))
        node.publish()
        self.assertEqual(len(node.pub.messages), 0)

    def test_republished_odometry_keeps_capture_time(self):
        node = self.Localization()
        transforms = []
        node.tf.sendTransform = transforms.append
        self.fused_sample(node, self.epoch, 0.)
        node.now_ns += 1_000_000_000
        node.publish()
        self.assertEqual(node.stamp_ns(node.pub.messages[-1]), self.epoch)
        self.assertEqual(len(transforms), 1)

    def test_old_scan_cannot_correct_present_pose(self):
        node = self.Localization()
        self.fused_sample(node, self.epoch, 0.)
        self.fused_sample(node, self.epoch+20_000_000, .1)
        calls = []
        node.matcher.match = lambda *a, **kw: calls.append(a)
        old_scan = scan(self.epoch)
        old_scan.ranges = [2.]*50
        node.scan_cb(old_scan)
        self.assertEqual(calls, [])

    def test_wall_stays_fixed_through_first_rotation_with_delayed_mapping(self):
        node = self.Localization()
        for step in range(101):
            yaw = step*math.pi/200
            ns = self.epoch+step*20_000_000
            self.fused_sample(node, ns, yaw, imu_first=bool(step % 2))
            self.mapping.pose_cb(node.pub.messages[-1])
            # A wall return fixed at map (2, 1), observed by a clockwise-indexed LiDAR.
            rx = 2*math.cos(yaw)+math.sin(yaw)
            ry = -2*math.sin(yaw)+math.cos(yaw)
            self.mapping.scan_cb(scan(ns, math.hypot(rx, ry), -math.atan2(ry, rx)))
            if step % 5 == 0:
                self.mapping.update()
        self.mapping.update()
        self.assertEqual(len(self.mapping.pub_points.messages), 101)
        self.assertTrue(all(not cloud.data for cloud in self.mapping.pub_points.messages))
        for cloud in self.mapping.pub_live_points.messages:
            x, y, z = struct.unpack('<fff', cloud.data)
            self.assertAlmostEqual(x, 2., places=5)
            self.assertAlmostEqual(y, 1., places=5)
            self.assertEqual(cloud.header.frame_id, 'map')

    def test_bridge_preserves_capture_time_through_transport_backlog(self):
        bridge = object.__new__(self.Bridge)
        bridge.rx = queue.Queue()
        bridge.last_sensor_stamp_ns = bridge.last_scan_sim_time = None
        bridge.scan_period, bridge.seq = .05, 0
        bridge.get_clock = self.mapping.get_clock
        for name in ('pub_joints', 'pub_imu', 'pub_scan', 'pub_gt'):
            setattr(bridge, name, Publisher())
        for step in range(2):
            bridge.rx.put({'type': 'sensors', 'seq': step, 'wall_time_ns': self.epoch+step*100_000_000,
                           'sim_time': step*.1, 'lidar': {'ranges': [2.]}})
        self.mapping.now_ns += 2_000_000_000
        bridge.process_rx()
        for publisher in (bridge.pub_joints, bridge.pub_imu, bridge.pub_scan):
            self.assertEqual([self.Localization.stamp_ns(m) for m in publisher.messages],
                             [self.epoch, self.epoch+100_000_000])


if __name__ == '__main__':
    unittest.main()
