"""Motor-output regressions using production safety callbacks without DDS."""
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import sys
import unittest
from unittest.mock import patch

import test_rotation_mapping as ros_support
from test_rotation_mapping import stamp, vector


def twist():
    return NS(linear=vector(), angular=vector())


def command(seconds, v=0., w=0.):
    message = NS(header=NS(stamp=stamp(int(seconds*1e9))), twist=twist())
    message.twist.linear.x, message.twist.angular.z = v, w
    return message


class SafetyArbitrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        ros_support.RotationMappingTests.setUpClass()
        modules = ros_support.RotationMappingTests.modules.copy()
        modules['geometry_msgs.msg'].Twist = twist
        modules['geometry_msgs.msg'].TwistStamped = lambda: command(0)
        with patch.dict(sys.modules, modules):
            path = Path(__file__).parents[1]/'ros_ws/src/fabtino_safety/fabtino_safety/safety_supervisor_node.py'
            spec = importlib.util.spec_from_file_location('_safety_regression', path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            cls.Safety = module.SafetySupervisor

    def setUp(self):
        self.node = self.Safety()

    def output(self, seconds):
        self.node.now_ns = int(seconds*1e9)
        self.node.last_scan = NS(header=NS(stamp=stamp(self.node.now_ns)), ranges=[], angle_min=0., angle_increment=0.)
        self.node.last_odom = NS(header=NS(stamp=stamp(self.node.now_ns)))
        self.node.loop()
        value = self.node.pub.messages[-1]
        return value.linear.x, value.angular.z

    def test_old_manual_turn_cannot_override_navigation_stop_forever(self):
        self.node.teleop_cb(command(10., w=1.5))
        self.node.nav_cb(command(11., v=0., w=0.))
        self.assertEqual(self.output(11.), (0., 0.))

    def test_live_manual_turn_still_works(self):
        for w in (.5, -.5):
            with self.subTest(angular=w):
                self.node.teleop_cb(command(10., w=w))
                self.assertEqual(self.output(10.1), (0., w))

    def test_invalid_lidar_ranges_do_not_block_either_turn_direction(self):
        for w in (.5, -.5):
            with self.subTest(angular=w):
                self.node.teleop_cb(command(10., w=w))
                self.output(10.1)
                self.node.last_scan.range_min = .2
                self.node.last_scan.range_max = 12.
                self.node.last_scan.ranges = [.1, 0., -1., 15., float('inf'), float('nan')]
                self.node.loop()
                self.assertEqual(self.node.pub.messages[-1].angular.z, w)

    def test_valid_close_obstacle_blocks_both_manual_turns(self):
        for w in (.5, -.5):
            with self.subTest(angular=w):
                self.node.teleop_cb(command(10., w=w))
                self.output(10.1)
                self.node.last_scan.range_min = .2
                self.node.last_scan.range_max = 12.
                self.node.last_scan.ranges = [.48]
                self.node.loop()
                self.assertEqual(self.node.pub.messages[-1].angular.z, 0.)
                self.assertIn('rotation_obstacle', self.node.status.messages[-1].data)

    def test_stale_manual_and_navigation_commands_both_stop(self):
        self.node.teleop_cb(command(10., w=1.5))
        self.node.nav_cb(command(10., v=.2))
        self.assertEqual(self.output(11.), (0., 0.))

    def test_new_goal_releases_manual_override_immediately(self):
        self.node.teleop_cb(command(10., w=1.5))
        self.node.goal_cb(NS(data=json.dumps({'type':'goal','goal_id':'new'})))
        self.node.nav_cb(command(10.1, v=.1))
        self.assertEqual(self.output(10.1), (.1, 0.))

    def test_completion_blocks_delayed_navigation_commands(self):
        self.node.goal_cb(NS(data=json.dumps({'type':'goal','goal_id':'new'})))
        self.node.local_status_cb(NS(data=json.dumps({'state':'goal_reached','goal_id':'new'})))
        self.node.nav_cb(command(10., w=.5))
        self.assertEqual(self.output(10.), (0., 0.))

    def test_old_completion_cannot_stop_new_goal(self):
        self.node.goal_cb(NS(data=json.dumps({'type':'goal','goal_id':'new'})))
        self.node.local_status_cb(NS(data=json.dumps({'state':'goal_reached','goal_id':'old'})))
        self.node.nav_cb(command(10., v=.1))
        self.assertEqual(self.output(10.), (.1, 0.))

    def test_explicit_stop_clears_both_motion_sources(self):
        self.node.teleop_cb(command(10., w=1.5))
        self.node.nav_cb(command(10., v=.2))
        self.node.goal_cb(NS(data=json.dumps({'type':'stop'})))
        self.node.nav_cb(command(10., w=.5))
        self.assertEqual(self.output(10.), (0., 0.))

    def test_chair_leg_in_front_corner_stops_forward_motion(self):
        self.node.nav_cb(command(10., v=.2))
        self.output(10.)
        self.node.last_scan.ranges = [.5]
        self.node.last_scan.angle_min = .6
        self.node.loop()
        self.assertEqual(self.node.pub.messages[-1].linear.x, 0.)
        self.assertIn('front_obstacle', self.node.status.messages[-1].data)

    def test_rear_wall_stops_manual_and_autonomous_reverse(self):
        for callback in (self.node.teleop_cb,self.node.nav_cb):
            with self.subTest(source=callback.__name__):
                self.node.clear_motion()
                callback(command(10.,v=-.5))
                self.output(10.)
                self.node.last_scan.ranges=[.60]
                self.node.last_scan.angle_min=math.pi
                self.node.loop()
                self.assertEqual(self.node.pub.messages[-1].linear.x,0.)
                self.assertIn('rear_obstacle',self.node.status.messages[-1].data)

    def test_near_rear_wall_still_allows_driving_forward_to_escape(self):
        self.node.teleop_cb(command(10.,v=.2))
        self.output(10.)
        self.node.last_scan.ranges=[.45]
        self.node.last_scan.angle_min=-math.pi
        self.node.loop()
        self.assertEqual(self.node.pub.messages[-1].linear.x,.2)
        self.assertFalse(json.loads(self.node.status.messages[-1].data)['stop_active'])

    def test_rear_corner_is_protected_and_side_returns_do_not_block_reverse(self):
        self.node.teleop_cb(command(10.,v=-.2))
        self.output(10.)
        self.node.last_scan.ranges=[.60]
        self.node.last_scan.angle_min=math.pi-.5
        self.node.loop()
        self.assertEqual(self.node.pub.messages[-1].linear.x,0.)
        self.node.last_scan.angle_min=math.pi/2
        self.node.loop()
        self.assertEqual(self.node.pub.messages[-1].linear.x,-.2)

    def test_invalid_rear_ranges_do_not_block_reverse(self):
        self.node.teleop_cb(command(10.,v=-.2))
        self.output(10.)
        self.node.last_scan.range_min=.2
        self.node.last_scan.range_max=12.
        self.node.last_scan.ranges=[.1,-1.,float('nan'),float('inf'),13.]
        self.node.last_scan.angle_min=math.pi
        self.node.loop()
        self.assertEqual(self.node.pub.messages[-1].linear.x,-.2)

    def test_obstacle_beside_robot_stops_rotation(self):
        self.node.nav_cb(command(10., w=.5))
        self.output(10.)
        self.node.last_scan.ranges = [.48]
        self.node.last_scan.angle_min = 1.57
        self.node.loop()
        self.assertEqual(self.node.pub.messages[-1].angular.z, 0.)
        self.assertIn('rotation_obstacle', self.node.status.messages[-1].data)

    def test_obstacle_outside_robot_corridor_does_not_block_straight_motion(self):
        self.node.nav_cb(command(10., v=.2))
        self.output(10.)
        self.node.last_scan.ranges = [.7]
        self.node.last_scan.angle_min = 1.2
        self.node.loop()
        self.assertEqual(self.node.pub.messages[-1].linear.x, .2)


if __name__ == '__main__':
    unittest.main()
