"""Run without ROS: python -m unittest discover -s tests -p test_goal_control.py."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parents[1] / 'ros_ws/src/fabtino_navigation'))
from fabtino_navigation.goal_control import Goal, GoalFollower, wrap_angle


class GoalControlTests(unittest.TestCase):
    def controller(self, yaw=None, x=10.0, y=0.0):
        goal = Goal.from_request({'x': x, 'y': y, 'yaw_deg': yaw})
        controller = GoalFollower(goal_tolerance_m=0.10)
        controller.set_goal(goal)
        controller.set_path(goal.goal_id, [(1.0, 0.0), (x, y)])
        return controller

    def test_finite_validation_and_optional_angle(self):
        self.assertIsNone(Goal.from_request({'x': 10, 'y': 0}).yaw_rad)
        for field in ('x', 'y', 'yaw_deg'):
            for value in (float('nan'), float('inf'), -float('inf')):
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    Goal.from_request(dict(x=10, y=0, yaw_deg=90) | {field: value})

    def test_goal_round_trip_and_repeated_coordinates_have_new_ids(self):
        goal = Goal.from_request({'x': 10, 'y': 0, 'yaw_deg': 450})
        self.assertEqual(goal, Goal.from_payload(goal.payload()))
        self.assertAlmostEqual(goal.yaw_rad, math.pi/2)
        self.assertNotEqual(goal.goal_id, Goal.from_request(goal.payload()).goal_id)

    def test_position_only_does_not_turn_at_arrival(self):
        controller = self.controller()
        self.assertEqual(controller.command((10, 0, 1.2), 0), (0, 0))
        self.assertEqual(controller.command((10, 0, 1.2), .4), (0, 0))
        self.assertEqual(controller.state, 'goal_reached')

    def test_final_heading_requires_rotation_before_success(self):
        for degrees in (80, 90, 180, -90):
            with self.subTest(degrees=degrees):
                controller = self.controller(degrees)
                linear, angular = controller.command((10, 0, 0), 0)
                self.assertEqual(linear, 0)
                self.assertEqual(controller.state, 'aligning')
                self.assertGreater(angular*degrees, 0)
                self.assertLessEqual(abs(angular), .5)
                self.assertEqual(controller.command((10, 0, math.radians(degrees)), 1), (0, 0))
                self.assertEqual(controller.state, 'settling')
                controller.command((10, 0, math.radians(degrees)), 1.4)
                self.assertEqual(controller.state, 'goal_reached')

    def test_wraparound_takes_short_rotation(self):
        controller = self.controller(-170)
        linear, angular = controller.command((10, 0, math.radians(170)), 0)
        self.assertEqual(linear, 0)
        self.assertGreater(angular, 0)

    def test_completed_goal_stays_stopped_despite_pose_noise_and_replans(self):
        controller = self.controller(90)
        controller.command((10, 0, math.pi/2), 0)
        controller.command((10, 0, math.pi/2), .4)
        for step in range(100):
            controller.set_path(controller.goal.goal_id, [(9, 0), (10, 0)])
            self.assertEqual(controller.command((9.85, .1, 1.4), 1+step*.05), (0, 0))
            self.assertEqual(controller.state, 'goal_reached')

    def test_stop_rejects_delayed_routes(self):
        controller = self.controller(90)
        old_id = controller.goal.goal_id
        controller.stop()
        controller.set_path(old_id, [(1, 0), (10, 0)])
        self.assertEqual(controller.command((1, 0, 0), 1), (0, 0))
        self.assertEqual(controller.state, 'stopped')

    def test_centimetre_position_jitter_does_not_restart_rotation_at_arrival(self):
        # Regression: Noise around the braking threshold must not alternate settling/following and
        # command 1 rad/s forever, even with no final orientation requested.
        for degrees in (None, 90):
            with self.subTest(degrees=degrees):
                controller = self.controller(degrees)
                yaw = 0.0 if degrees is None else math.pi/2
                for step in range(20):
                    controller.set_path(controller.goal.goal_id, [(9, 0), (10, 0)])
                    x = 10.045 if step % 2 == 0 else 10.055
                    self.assertEqual(controller.command((x, 0, yaw), step*.05), (0, 0))
                self.assertEqual(controller.state, 'goal_reached')

    def test_yaw_noise_at_inner_tolerance_does_not_restart_a_turn(self):
        controller = self.controller(90)
        for step in range(20):
            error = math.radians(1.4 if step % 2 == 0 else 2.2)
            self.assertEqual(controller.command((10, 0, math.pi/2-error), step*.05), (0, 0))
        self.assertEqual(controller.state, 'goal_reached')

    def test_initial_yaw_outside_inner_tolerance_still_requires_alignment(self):
        controller = self.controller(90)
        linear, angular = controller.command((10, 0, math.pi/2-math.radians(4)), 0)
        self.assertEqual(controller.state, 'aligning')
        self.assertEqual(linear, 0)
        self.assertGreater(angular, 0)

    def test_final_rotation_without_progress_fails_and_stays_stopped(self):
        controller = self.controller(90)
        controller.command((10, 0, 0), 0)
        self.assertEqual(controller.command((10, 0, 0), 5.1), (0, 0))
        self.assertEqual(controller.state, 'goal_failed')
        self.assertEqual(controller.reason, 'final_heading_no_progress')
        for step in range(10):
            controller.set_path(controller.goal.goal_id, [(9, 0), (10, 0)])
            self.assertEqual(controller.command((10, 0, .2), 6+step), (0, 0))
            self.assertEqual(controller.state, 'goal_failed')

    def test_slow_progress_is_bounded_by_final_rotation_deadline(self):
        controller = self.controller(180)
        for step in range(31):
            command = controller.command((10, 0, step*.02), float(step))
        self.assertEqual(command, (0, 0))
        self.assertEqual(controller.state, 'goal_failed')
        self.assertEqual(controller.reason, 'final_heading_timeout')

    def test_new_goal_resets_failed_rotation(self):
        controller = self.controller(90)
        controller.command((10, 0, 0), 0)
        controller.command((10, 0, 0), 6)
        goal = Goal.from_request({'x': 10, 'y': 0})
        controller.set_goal(goal)
        controller.set_path(goal.goal_id, [(10, 0)])
        controller.command((10, 0, 0), 7)
        controller.command((10, 0, 0), 7.4)
        self.assertEqual(controller.state, 'goal_reached')
        self.assertEqual(controller.reason, '')

    def test_position_inside_tolerance_is_not_success_while_still_moving(self):
        controller = self.controller()
        for step in range(10):
            self.assertEqual(controller.command((10, 0, 0), step*.1, velocity=(.04, 0.)), (0., 0.))
            self.assertEqual(controller.state, 'braking')
        controller.command((10, 0, 0), 1., velocity=(0., 0.))
        self.assertEqual(controller.state, 'settling')
        controller.command((10, 0, 0), 1.41, velocity=(0., 0.))
        self.assertEqual(controller.state, 'goal_reached')

    def test_correct_angle_is_not_success_until_measured_rotation_stops(self):
        controller = self.controller(90)
        for rate in (.2, -.2):
            self.assertEqual(controller.command((10, 0, math.pi/2), 0., velocity=(0., rate)), (0., 0.))
            self.assertEqual(controller.state, 'braking')

    def test_braking_margin_never_expands_the_requested_position_tolerance(self):
        controller = self.controller()
        controller.command((10.04, 0, 0), 0.)
        controller.command((10.11, 0, 0), .5)
        self.assertFalse(controller.position_reached)
        self.assertNotEqual(controller.state, 'goal_reached')

    def test_final_approach_speed_is_limited(self):
        controller = self.controller()
        linear, _ = controller.command((9.7, 0, 0), 0.)
        self.assertEqual(controller.state, 'approaching')
        self.assertGreater(linear, 0.)
        self.assertLessEqual(linear, .08)

    def test_final_approach_does_not_cut_a_planned_detour(self):
        controller = self.controller()
        controller.set_path(controller.goal.goal_id, [(9.8, 0), (9.8, 1), (10, 1), (10, 0)])
        controller.command((9.8, 0, math.pi/2), 0.)
        self.assertEqual(controller.state, 'following')

    def test_target_behind_robot_does_not_reverse_turn_on_pose_noise(self):
        controller = self.controller(x=-2)
        controller.set_path(controller.goal.goal_id, [(0, 0), (-2, 0)])
        commands = [controller.command((0, 0, yaw), i*.1)[1]
                    for i, yaw in enumerate([.002, -.002]*10)]
        self.assertTrue(all(w > 0 for w in commands))
        self.assertTrue(all(abs(w) <= .7 for w in commands))

    def test_final_half_turn_keeps_its_direction_despite_pose_noise(self):
        controller = self.controller(yaw=180)
        commands = [controller.command((10, 0, yaw), i*.1)[1]
                    for i, yaw in enumerate([.002, -.002]*10)]
        self.assertTrue(all(w > 0 for w in commands))

    def test_turning_without_heading_progress_fails_and_stays_stopped(self):
        controller = self.controller(x=-2)
        controller.set_path(controller.goal.goal_id, [(0, 0), (-2, 0)])
        controller.command((0, 0, 0), 0)
        self.assertEqual(controller.state, 'turning_to_path')
        self.assertEqual(controller.command((0, 0, 0), 5.1), (0, 0))
        self.assertEqual(controller.reason, 'path_heading_no_progress')
        self.assertEqual(controller.command((0, 0, math.pi), 6), (0, 0))

    def test_final_angle_does_not_control_departure_heading(self):
        for degrees in (0, 80, 90, 180):
            controller = self.controller(yaw=degrees)
            v, w = controller.command((1, 0, 0), 0)
            self.assertGreater(v, 0)
            self.assertEqual(w, 0)

    def test_path_heading_brakes_measured_rotation(self):
        controller = self.controller()
        _, w = controller.command((1, 0, -.1), 0, velocity=(0, .7))
        self.assertLess(w, 0)

    def test_blocked_translation_eventually_stops_the_mission(self):
        controller = self.controller()
        controller.command((1, 0, 0), 0)
        self.assertEqual(controller.command((1, 0, 0), 15.1), (0, 0))
        self.assertEqual(controller.reason, 'translation_no_progress')

    def test_new_goal_rejects_previous_route_even_for_same_coordinates(self):
        controller = self.controller()
        old_id = controller.goal.goal_id
        goal = Goal.from_request({'x': 10, 'y': 0, 'yaw_deg': 180})
        controller.set_goal(goal)
        controller.set_path(old_id, [(1, 0), (10, 0)])
        self.assertEqual(controller.command((1, 0, 0), 0), (0, 0))
        controller.set_path(goal.goal_id, [(1, 0), (10, 0)])
        self.assertGreater(controller.command((1, 0, 0), 1)[0], 0)

    def test_missing_or_failed_path_stops_before_alignment(self):
        controller = self.controller(90)
        controller.set_path(controller.goal.goal_id, [])
        self.assertEqual(controller.command((10, 0, 0), 0), (0, 0))
        self.assertEqual(controller.state, 'waiting_for_path')

    def test_lookahead_does_not_go_back_to_old_path_start(self):
        controller = self.controller()
        self.assertAlmostEqual(controller.arc_target((8, 0, 0))[0], 8.45)
        linear, angular = controller.command((8, 0, 0), 0)
        self.assertGreater(linear, 0)
        self.assertAlmostEqual(angular, 0)

    def test_small_position_noise_during_alignment_does_not_restart_translation(self):
        controller = self.controller(90)
        controller.command((9.95, 0, 0), 0)
        linear, angular = controller.command((9.94, 0, 0), .1)
        self.assertEqual(linear, 0)
        self.assertGreater(angular, 0)
        # Larger displacement requires a new approach; never report a false arrival.
        linear, _ = controller.command((9.7, 0, 0), .2)
        self.assertGreater(linear, 0)
        self.assertEqual(controller.state, 'approaching')

    def test_settle_requires_continuous_heading_tolerance(self):
        controller = self.controller(90)
        controller.command((10, 0, math.pi/2), 0)
        controller.command((10, 0, math.pi/2-.2), .2)
        controller.command((10, 0, math.pi/2), .4)
        self.assertEqual(controller.state, 'settling')
        controller.command((10, 0, math.pi/2), .6)
        self.assertEqual(controller.state, 'settling')
        controller.command((10, 0, math.pi/2), .8)
        self.assertEqual(controller.state, 'goal_reached')

    def test_simulated_trip_from_x1_to_x10_and_final_headings(self):
        # Kinematic regression, not a substitute for Webots / odometry validation.
        for degrees in (None, 80, 90, 180):
            with self.subTest(degrees=degrees):
                controller = self.controller(degrees)
                pose = [1.0, 0.0, 0.0]
                for step in range(2000):
                    # Replan repeatedly, including after arrival.
                    controller.set_path(controller.goal.goal_id, [(pose[0], pose[1]), (10, 0)])
                    linear, angular = controller.command(pose, step*.05)
                    pose[0] += linear*math.cos(pose[2])*.05
                    pose[1] += linear*math.sin(pose[2])*.05
                    pose[2] = wrap_angle(pose[2]+angular*.05)
                self.assertEqual(controller.state, 'goal_reached')
                self.assertLessEqual(math.hypot(10-pose[0], pose[1]), .10)
                if degrees is not None:
                    self.assertLessEqual(abs(wrap_angle(math.radians(degrees)-pose[2])), math.radians(3))
                self.assertEqual((linear, angular), (0, 0))


if __name__ == '__main__':
    unittest.main()
