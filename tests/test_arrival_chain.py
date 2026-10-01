"""Production mission/A*/local/safety callbacks with a synthetic motor plant.

The transport is in-process; this is not a ROS DDS or Webots execution.
"""
from collections import deque
import json
import math
import random
import unittest
from types import SimpleNamespace as NS

import test_navigation_flow as navigation_support
import test_safety_arbitration as safety_support
from test_rotation_mapping import odometry, scan


class ArrivalChainTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        navigation_support.NavigationFlowTests.setUpClass()
        safety_support.SafetyArbitrationTests.setUpClass()

    def test_arrival_through_actual_planner_and_motor_arbitration_callbacks(self):
        for angle, distance in ((None, 1.), (80, 1.), (90, 1.), (180, 1.), (-90, 9.)):
            with self.subTest(angle=angle, distance=distance):
                support = navigation_support.NavigationFlowTests
                mission, planner, local = support.Mission(), support.Global(), support.Local()
                safety = safety_support.SafetyArbitrationTests.Safety()
                planner.grid = navigation_support.OccupancyGrid(.1, 20.)
                planner.grid.data.fill(0)
                safety.teleop_cb(safety_support.command(0., w=1.5))
                mission.req_cb(navigation_support.msg(dict(type='goal', x=distance, y=0., yaw_deg=angle)))
                goal = mission.pub.messages[-1]
                for callback in (planner.goal_cb, local.goal_cb, safety.goal_cb):
                    callback(goal)
                pose, velocity = [0., 0., 0.], [0., 0.]
                feedback = deque([(pose.copy(), velocity.copy())]*6, maxlen=6)
                commands = deque([(0., 0.)]*6, maxlen=6)
                status_index, reached_at = 0, None
                rng = random.Random(42)
                for step in range(6000):
                    now = step*.02
                    local.now = now
                    safety.now_ns = int(now*1e9)
                    delayed_pose, delayed_velocity = feedback[0]
                    odom = odometry(safety.now_ns, delayed_pose[2])
                    odom.pose.pose.position.x = delayed_pose[0]+rng.uniform(-.005,.005)
                    odom.pose.pose.position.y = delayed_pose[1]+rng.uniform(-.005,.005)
                    odom.twist.twist.linear.x, odom.twist.twist.angular.z = delayed_velocity
                    local.pose_cb(odom)
                    planner.pose_cb(odom)
                    safety.odom_cb(odom)
                    safety.scan_cb(scan(safety.now_ns, distance=12.))
                    if step % 5 == 0:
                        planner.plan()
                        local.path_cb(planner.route_pub.messages[-1])
                    local.control()
                    safety.nav_cb(local.pub.messages[-1])
                    for status in local.status.messages[status_index:]:
                        mission.local_cb(status)
                        planner.local_cb(status)
                        safety.local_status_cb(status)
                    status_index = len(local.status.messages)
                    safety.loop()
                    motor = safety.pub.messages[-1]
                    if local.controller.state == 'goal_reached':
                        if reached_at is None:
                            reached_at = now
                            self.assertLessEqual(abs(velocity[0]), .01)
                            self.assertLessEqual(abs(velocity[1]), .03)
                        self.assertEqual((motor.linear.x, motor.angular.z), (0., 0.))
                    self.assertNotEqual(local.controller.state, 'goal_failed')
                    commands.append((motor.linear.x,motor.angular.z))
                    for index in (0,1):
                        velocity[index] += (commands[0][index]-velocity[index])*.02/.3
                    pose[0] += velocity[0]*math.cos(pose[2])*.02
                    pose[1] += velocity[0]*math.sin(pose[2])*.02
                    pose[2] += velocity[1]*.02
                    feedback.append((pose.copy(),velocity.copy()))
                    if reached_at is not None and now-reached_at >= 3.:
                        break
                self.assertIsNotNone(reached_at)
                self.assertLessEqual(math.hypot(pose[0]-distance,pose[1]), .05)
                if angle is not None:
                    error = math.atan2(math.sin(math.radians(angle)-pose[2]), math.cos(math.radians(angle)-pose[2]))
                    self.assertLessEqual(abs(math.degrees(error)),3.)
                self.assertIsNone(mission.goal)
                self.assertIsNone(planner.goal)
                self.assertFalse(safety.navigation_enabled)
                # Even a queued command received after success cannot turn motors.
                safety.nav_cb(safety_support.command(now,w=.5))
                safety.loop()
                self.assertEqual(safety.pub.messages[-1].angular.z,0.)


if __name__ == '__main__':
    unittest.main()
