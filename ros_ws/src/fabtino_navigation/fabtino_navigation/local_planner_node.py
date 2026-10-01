from __future__ import annotations
import json
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from fabtino_navigation.goal_control import Goal, GoalFollower, wrap_angle
from fabtino_core.navigation.zones import segment_denied


class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__('local_planner')
        defaults = dict(lookahead_m=0.45, max_linear_mps=0.25, max_angular_rps=1.0,
                        heading_slowdown_rad=1.0, goal_tolerance_m=0.05,
                        yaw_tolerance_rad=math.radians(3), final_max_angular_rps=0.35,
                        settle_time_s=0.4, yaw_hysteresis_rad=math.radians(2),
                        alignment_no_progress_s=5.0, alignment_timeout_s=30.0,
                        final_approach_distance_m=0.4, final_linear_mps=0.08,
                        final_heading_kp=1.2, final_heading_kd=0.35,
                        stopped_linear_mps=0.01, stopped_angular_rps=0.03,
                        path_max_angular_rps=0.7, path_heading_kd=0.35,
                        path_turn_timeout_s=20.0, path_turn_no_progress_s=5.0,
                        movement_no_progress_s=15.0)
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self.controller = GoalFollower(**{name: float(self.get_parameter(name).value) for name in defaults})
        self.pose = None
        self.velocity = (0.0, 0.0)
        self.pose_stamp = None
        self.declare_parameter('pose_timeout_s',0.30)
        self.last_status = None
        self.last_status_time = -math.inf
        self.zones=[]
        self.config_revision=-1
        self.declare_parameter('navigation_clearance_m',.60)
        self.create_subscription(String,'/navigation/config',self.config_cb,10)
        self.pub = self.create_publisher(TwistStamped, '/navigation/cmd_vel', 20)
        self.status = self.create_publisher(String, '/navigation/local_status', 10)
        self.create_subscription(String, '/navigation/goal', self.goal_cb, 10)
        self.create_subscription(String, '/navigation/route', self.path_cb, 5)
        self.create_subscription(Odometry, '/fabtino/odometry/filtered', self.pose_cb, 10)
        self.create_timer(0.05, self.control)

    def goal_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            if 'navigation_config' in payload:
                config=String();config.data=json.dumps(dict(payload['navigation_config'],revision=payload.get('config_revision',0)));self.config_cb(config)
            if payload.get('type') == 'stop':
                self.controller.stop()
            else:
                self.controller.set_goal(Goal.from_payload(payload))
            self.control()
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warning(f'Invalid navigation goal: {exc}')

    def config_cb(self,msg):
        payload=json.loads(msg.data);revision=int(payload.get('revision',0))
        if revision<self.config_revision:return
        self.config_revision=revision;self.zones=payload.get('zones',[])

    def path_cb(self, msg):
        payload = json.loads(msg.data)
        self.controller.set_path(payload['goal_id'], payload['points'])

    def pose_cb(self, msg):
        from fabtino_core.geometry import euler_from_quaternion
        q = msg.pose.pose.orientation
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     euler_from_quaternion([q.x, q.y, q.z, q.w])[2])
        self.velocity = (msg.twist.twist.linear.x,msg.twist.twist.angular.z)
        self.pose_stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def control(self):
        now = self.get_clock().now()
        seconds = now.nanoseconds*1e-9
        pose_age = None if self.pose_stamp is None else seconds-self.pose_stamp
        pose_fresh = pose_age is not None and 0 <= pose_age <= float(self.get_parameter('pose_timeout_s').value)
        if not pose_fresh and self.controller.goal is not None and self.controller.state not in ('goal_reached','goal_failed'):
            linear, angular = 0.0, 0.0
            self.controller.state = 'waiting_for_pose'
            self.controller.settled_since = None
        else:
            lookahead=self.controller.lookahead
            try:
                if self.pose is not None and self.zones and self.controller.path:
                    while self.controller.lookahead>.025 and segment_denied(self.pose[:2],self.controller.arc_target(self.pose),
                            self.zones,float(self.get_parameter('navigation_clearance_m').value)):
                        self.controller.lookahead*=.5
                linear, angular = self.controller.command(self.pose, seconds, velocity=self.velocity)
            finally:self.controller.lookahead=lookahead
        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x, msg.twist.angular.z = linear, angular
        self.pub.publish(msg)
        goal_id = self.controller.goal.goal_id if self.controller.goal else None
        signature = (self.controller.state, goal_id)
        if signature != self.last_status or seconds-self.last_status_time >= 0.5:
            status = String()
            goal = self.controller.goal
            distance = None if goal is None or self.pose is None else math.hypot(goal.x-self.pose[0], goal.y-self.pose[1])
            yaw_error = None if goal is None or goal.yaw_rad is None or self.pose is None else math.degrees(wrap_angle(goal.yaw_rad-self.pose[2]))
            status.data = json.dumps({'state': signature[0], 'goal_id': goal_id,
                                     'reason': self.controller.reason,
                                     'controller_version': 'arrival-v4',
                                     'distance_m': distance, 'yaw_error_deg': yaw_error,
                                     'measured_linear_mps':self.velocity[0],
                                     'measured_angular_rps':self.velocity[1], 'pose_age_s':pose_age,
                                     'position_reached': self.controller.position_reached,
                                     'command_linear_mps': linear, 'command_angular_rps': angular})
            self.status.publish(status)
            self.last_status = signature
            self.last_status_time = seconds


def main():
    rclpy.init()
    node = LocalPlannerNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
