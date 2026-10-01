"""Callback integration with real A*, substituting only the unavailable ROS transport.

Run: python -m unittest discover -s tests -p test_navigation_flow.py -v
This does not exercise DDS, ROS executors, sensors or motor dynamics.
"""
import importlib.util
import json
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT = Path(__file__).parents[1]
for package in ('fabtino_navigation', 'fabtino_core'):
    sys.path.insert(0, str(ROOT / 'ros_ws/src' / package))
from fabtino_core.navigation.planner import OccupancyGrid


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def header():
    return NS(stamp=NS(sec=0, nanosec=0), frame_id='')


def pose_stamped():
    return NS(header=header(), pose=NS(position=NS(x=0., y=0., z=0.),
                                      orientation=NS(x=0., y=0., z=0., w=0.)))


class FakeNode:
    def __init__(self, name):
        self.parameters = {}
        self.now = 0.

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
        ns = int(self.now*1e9)
        return NS(now=lambda: NS(nanoseconds=ns, to_msg=lambda: NS(sec=ns//1_000_000_000, nanosec=ns%1_000_000_000)))

    def get_logger(self):
        return NS(warning=lambda msg: None)


def load_adapter(package, name):
    spec = importlib.util.spec_from_file_location(
        '_test_' + name, ROOT / 'ros_ws/src' / package / package / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def msg(payload):
    return NS(data=json.dumps(payload))


class NavigationFlowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        modules = {}
        for name in ('rclpy', 'rclpy.node', 'nav_msgs', 'nav_msgs.msg',
                     'geometry_msgs', 'geometry_msgs.msg', 'std_msgs', 'std_msgs.msg',
                     'sensor_msgs', 'sensor_msgs.msg', 'websockets'):
            modules[name] = ModuleType(name)
        modules['rclpy.node'].Node = FakeNode
        modules['nav_msgs.msg'].OccupancyGrid = NS
        modules['nav_msgs.msg'].Odometry = NS
        modules['nav_msgs.msg'].Path = lambda: NS(header=header(), poses=[])
        modules['geometry_msgs.msg'].PoseStamped = pose_stamped
        modules['geometry_msgs.msg'].TwistStamped = lambda: NS(
            header=header(), twist=NS(linear=NS(x=0., y=0., z=0.), angular=NS(x=0., y=0., z=0.)))
        modules['std_msgs.msg'].String = lambda: NS(data='')
        modules['sensor_msgs.msg'].LaserScan = NS
        modules['sensor_msgs.msg'].PointCloud2 = NS
        # Restore sys.modules after loading; do not mask ROS for other test suites.
        with patch.dict(sys.modules, modules):
            cls.Mission = load_adapter('fabtino_navigation', 'mission_manager_node').MissionManagerNode
            cls.Global = load_adapter('fabtino_navigation', 'global_planner_node').GlobalPlannerNode
            cls.Local = load_adapter('fabtino_navigation', 'local_planner_node').LocalPlannerNode
            cls.Gateway = load_adapter('fabtino_viewer', 'viewer_gateway').ViewerGateway

    def setUp(self):
        self.mission, self.planner, self.local = self.Mission(), self.Global(), self.Local()
        self.planner.grid = OccupancyGrid(.1, 20)
        self.planner.grid.data.fill(0)
        self.planner.pose = (1., 0.)
        self.local.pose = (1., 0., 0.)
        self.local.pose_stamp = 0.0

    def tick_local(self):
        self.local.pose_stamp = self.local.now
        self.local.control()

    def start(self, **request):
        self.mission.req_cb(msg(dict(type='goal', x=10, y=0, yaw_deg=90) | request))
        goal = self.mission.pub.messages[-1]
        self.planner.goal_cb(goal)
        self.local.goal_cb(goal)
        self.planner.plan()
        self.local.path_cb(self.planner.route_pub.messages[-1])

    def test_goal_keeps_exact_endpoint_and_orientation_through_all_callbacks(self):
        self.start(x=10.03, y=.02, yaw_deg=80)
        endpoint = self.planner.pub.messages[-1].poses[-1].pose
        self.assertEqual((endpoint.position.x, endpoint.position.y), (10.03, .02))
        self.assertAlmostEqual(endpoint.orientation.z, math.sin(math.radians(40)))
        self.assertAlmostEqual(self.local.controller.goal.yaw_rad, math.radians(80))
        self.tick_local()
        self.assertGreater(self.local.pub.messages[-1].twist.linear.x, 0)

    def test_stop_cancels_both_planners_and_rejects_delayed_path(self):
        self.start()
        old_route = self.planner.route_pub.messages[-1]
        self.mission.req_cb(msg({'type': 'stop'}))
        stop = self.mission.pub.messages[-1]
        self.planner.goal_cb(stop)
        self.local.goal_cb(stop)
        self.local.path_cb(old_route)
        self.tick_local()
        self.assertIsNone(self.planner.goal)
        self.assertEqual(self.planner.pub.messages[-1].poses, [])
        self.assertEqual(self.local.pub.messages[-1].twist.linear.x, 0)
        self.assertEqual(self.local.pub.messages[-1].twist.angular.z, 0)
        self.assertEqual(self.local.controller.state, 'stopped')

    def test_completion_clears_mission_and_stops_global_replanning(self):
        self.start()
        self.local.pose = (10., 0., math.pi/2)
        self.tick_local()
        self.local.now = .4
        self.tick_local()
        reached = self.local.status.messages[-1]
        self.mission.local_cb(reached)
        self.planner.local_cb(reached)
        self.local.path_cb(self.planner.route_pub.messages[-1])
        count = len(self.planner.pub.messages)
        self.planner.plan()
        self.assertEqual(len(self.planner.pub.messages), count)
        self.assertIsNone(self.mission.goal)
        self.assertIsNone(self.planner.goal)
        self.assertEqual(json.loads(self.mission.status.messages[-1].data)['state'], 'goal_reached')
        self.assertEqual(self.local.controller.state, 'goal_reached')

    def test_old_completion_does_not_cancel_replacement_goal(self):
        self.start()
        old_id = self.mission.goal.goal_id
        self.start(yaw_deg=180)
        old_status = msg({'state': 'goal_reached', 'goal_id': old_id})
        self.mission.local_cb(old_status)
        self.planner.local_cb(old_status)
        self.assertIsNotNone(self.mission.goal)
        self.assertIsNotNone(self.planner.goal)
        self.assertAlmostEqual(self.local.controller.goal.yaw_rad, math.pi)

    def test_failed_final_alignment_stops_all_navigation_and_reports_reason(self):
        self.start()
        self.local.pose = (10., 0., 0.)
        self.tick_local()
        self.local.now = 6.
        self.tick_local()
        failure = self.local.status.messages[-1]
        details = json.loads(failure.data)
        self.assertEqual(details['state'], 'goal_failed')
        self.assertEqual(details['controller_version'], 'arrival-v4')
        self.assertEqual(details['command_angular_rps'], 0.)
        self.assertAlmostEqual(details['yaw_error_deg'], 90.)
        self.mission.local_cb(failure)
        self.planner.local_cb(failure)
        self.assertIsNone(self.mission.goal)
        self.assertIsNone(self.planner.goal)
        self.assertFalse(self.mission.explore_mode)
        self.assertEqual(json.loads(self.mission.status.messages[-1].data)['reason'], 'final_heading_no_progress')

    def test_diagnostics_refresh_even_when_control_state_is_unchanged(self):
        self.start()
        self.tick_local()
        before = json.loads(self.local.status.messages[-1].data)
        self.local.pose = (2., 0., 0.)
        self.local.now = .6
        self.tick_local()
        after = json.loads(self.local.status.messages[-1].data)
        self.assertEqual(before['state'], after['state'])
        self.assertEqual(after['distance_m'], before['distance_m']-1.)

    def test_stale_pose_cannot_confirm_arrival(self):
        self.start()
        self.local.pose = (10., 0., math.pi/2)
        self.tick_local()
        self.local.now = 1.
        self.local.control()  # No new odometry sample.
        self.assertEqual(self.local.controller.state, 'waiting_for_pose')
        self.assertEqual(self.local.pub.messages[-1].twist.angular.z, 0.)

    def test_measured_motion_prevents_premature_completion(self):
        self.start()
        self.local.pose = (10., 0., math.pi/2)
        self.local.velocity = (0., .2)
        self.tick_local()
        self.local.now = .6
        self.tick_local()
        self.assertEqual(self.local.controller.state, 'braking')
        self.local.velocity = (0., 0.)
        self.local.now = .7
        self.tick_local()
        self.local.now = 1.2
        self.tick_local()
        self.assertEqual(self.local.controller.state, 'goal_reached')

    def test_failed_path_clears_previous_motion(self):
        self.start()
        self.planner.grid.data.fill(100)
        self.planner.plan()
        self.local.path_cb(self.planner.route_pub.messages[-1])
        self.tick_local()
        self.assertEqual(json.loads(self.planner.status.messages[-1].data)['state'], 'pathfinding_failed')
        self.assertEqual(self.local.pub.messages[-1].twist.linear.x, 0)
        self.assertEqual(self.local.pub.messages[-1].twist.angular.z, 0)

    def test_route_keeps_robot_footprint_away_from_a_table(self):
        self.planner.grid.set_cell(self.planner.grid.cell(5., 0.), 100)
        self.start()
        self.assertTrue(self.planner.path)
        for cell in self.planner.path:
            x, y = self.planner.grid.point(cell)
            self.assertGreater(math.hypot(x-5.05, y-.05), .52)

    def test_goal_too_close_to_a_table_does_not_generate_motion(self):
        self.planner.grid.set_cell(self.planner.grid.cell(5., 0.), 100)
        self.start(x=5.2, y=0.)
        self.tick_local()
        self.assertFalse(self.planner.path)
        self.assertEqual(self.local.pub.messages[-1].twist.linear.x, 0)
        self.assertEqual(self.local.pub.messages[-1].twist.angular.z, 0)

    def test_outside_map_goal_does_not_keep_old_path(self):
        self.start(x=100)
        self.assertEqual(self.planner.pub.messages[-1].poses, [])
        self.assertEqual(json.loads(self.planner.status.messages[-1].data)['state'], 'goal_outside_map')
        self.tick_local()
        self.assertEqual(self.local.pub.messages[-1].twist.linear.x, 0)

    def test_gateway_forwards_angle_and_both_stop_controls(self):
        gateway = object.__new__(self.Gateway)
        gateway.pub_req, gateway.pub_teleop = Publisher(), Publisher()
        gateway.get_clock = self.local.get_clock
        gateway.cmd('nav_goal', {'x': 10, 'y': 0, 'yaw_deg': 180})
        request = json.loads(gateway.pub_req.messages[-1].data)
        self.assertEqual(request['yaw_deg'], 180)
        self.mission.req_cb(gateway.pub_req.messages[-1])
        self.assertAlmostEqual(self.mission.goal.yaw_rad, math.pi)
        for kind in ('nav_stop', 'stop'):
            gateway.cmd(kind, {})
            self.assertEqual(json.loads(gateway.pub_req.messages[-1].data)['type'], 'stop')

    def test_invalid_angle_does_not_replace_running_mission(self):
        self.start()
        goal = self.mission.goal
        self.mission.req_cb(msg({'type': 'goal', 'x': 2, 'y': 3, 'yaw_deg': 'bad'}))
        self.assertEqual(self.mission.goal, goal)
        self.assertEqual(json.loads(self.mission.status.messages[-1].data)['state'], 'invalid_request')

    def test_exploration_ignores_duplicate_completion_and_visited_frontier(self):
        grid = NS(info=NS(width=4, height=4, resolution=1., origin=NS(position=NS(x=0., y=0.))), data=[-1]*16)
        grid.data[5] = grid.data[10] = 0
        self.mission.map, self.mission.pose = grid, (1.5, 1.5)
        self.mission.req_cb(msg({'type': 'explore'}))
        first = self.mission.goal
        reached = msg({'state': 'goal_reached', 'goal_id': first.goal_id})
        self.mission.local_cb(reached)
        second = self.mission.goal
        self.assertNotEqual((first.x, first.y), (second.x, second.y))
        self.mission.local_cb(reached)
        self.assertEqual(self.mission.goal, second)
        self.mission.local_cb(msg({'state': 'goal_reached', 'goal_id': second.goal_id}))
        self.assertIsNone(self.mission.goal)
        self.assertFalse(self.mission.explore_mode)


if __name__ == '__main__':
    unittest.main()
