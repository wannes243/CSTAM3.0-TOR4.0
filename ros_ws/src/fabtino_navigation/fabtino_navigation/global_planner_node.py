from __future__ import annotations
import json
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from fabtino_core.navigation.planner import OccupancyGrid as CoreGrid, astar
from fabtino_navigation.goal_control import Goal
from fabtino_core.navigation.zones import navigation_mask,point_denied,segment_denied


class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner')
        for name, value in dict(resolution_m=0.10, radius_m=20.0, safety_margin_m=0.08,
                                robot_radius_m=0.52,
                                allow_unknown=True, unknown_cost=1.15).items():
            self.declare_parameter(name, value)
        self.grid = None
        self.pose = None
        self.goal = None
        self.path = []
        self.zones=[]
        self.config_revision=-1
        self.last_navigation_map_ns=None
        self.pub = self.create_publisher(NavPath, '/navigation/global_path', 5)
        # Keep standard Path for visualization; tagged routes prevent stale motion.
        self.route_pub = self.create_publisher(String, '/navigation/route', 5)
        self.status = self.create_publisher(String, '/navigation/global_status', 10)
        self.create_subscription(OccupancyGrid, '/fabtino/map', self.map_cb, 3)
        self.create_subscription(OccupancyGrid,'/fabtino/navigation_map',lambda msg:self.map_cb(msg,True),3)
        self.create_subscription(String,'/navigation/config',self.config_cb,10)
        self.create_subscription(Odometry, '/fabtino/odometry/filtered', self.pose_cb, 10)
        self.create_subscription(String, '/navigation/goal', self.goal_cb, 10)
        self.create_subscription(String, '/navigation/local_status', self.local_cb, 10)
        self.create_timer(0.1, self.plan)

    def map_cb(self, msg, navigation=False):
        now=self.get_clock().now().nanoseconds
        if navigation:self.last_navigation_map_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        elif self.last_navigation_map_ns is not None and now-self.last_navigation_map_ns<600_000_000:return
        resolution = float(msg.info.resolution)
        radius = -float(msg.info.origin.position.x)
        self.grid = CoreGrid(resolution, radius)
        self.grid.load(resolution, radius, msg.data)

    def config_cb(self,msg):
        payload=json.loads(msg.data);revision=int(payload.get('revision',0))
        if revision<self.config_revision:return
        self.config_revision=revision;zones=payload.get('zones',[])
        if zones==self.zones:return
        self.zones=zones
        self.path=[];self.publish_path()

    def pose_cb(self, msg):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def goal_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            if 'navigation_config' in payload:
                config=String();config.data=json.dumps(dict(payload['navigation_config'],revision=payload.get('config_revision',0)));self.config_cb(config)
            goal = None if payload.get('type') == 'stop' else Goal.from_payload(payload)
            self.goal = goal
            self.path = []
            self.publish_path()
            self.emit('stopped' if goal is None else 'planning')
        except (ValueError, KeyError, TypeError) as exc:
            self.emit('invalid_goal', str(exc))

    def local_cb(self, msg):
        payload = json.loads(msg.data)
        if (self.goal is not None and payload.get('goal_id') == self.goal.goal_id
                and payload.get('state') in ('goal_reached', 'goal_failed')):
            self.emit(payload['state'], payload.get('reason', ''))
            self.goal = None
            self.path = []
            self.publish_path()

    def emit(self, state, reason=''):
        msg = String()
        msg.data = json.dumps({'state': state, 'reason': reason,
                               'goal_id': self.goal.goal_id if self.goal else None,
                               'goal': [self.goal.x, self.goal.y] if self.goal else None,
                               'path_cells': len(self.path)})
        self.status.publish(msg)

    def fail(self, state, reason=''):
        self.path = []
        self.publish_path()
        self.emit(state, reason)

    def plan(self):
        if self.goal is None:
            return
        if self.grid is None or self.pose is None:
            self.fail('waiting_for_map_or_pose')
            return
        start = self.grid.cell(*self.pose)
        goal = self.grid.cell(self.goal.x, self.goal.y)
        if goal is None:
            self.fail('goal_outside_map')
            return
        if start is None:
            self.fail('outside_map_recovery', 'Robot is outside the navigation map')
            return
        clearance = (float(self.get_parameter('robot_radius_m').value)
                     + float(self.get_parameter('safety_margin_m').value))
        if point_denied(self.pose[0],self.pose[1],self.zones,clearance):
            self.fail('robot_in_denied_zone','Move the robot outside the denied zone before navigating');return
        if point_denied(self.goal.x,self.goal.y,self.zones,clearance):
            self.fail('goal_in_denied_zone','Choose a target outside the denied zone and robot clearance');return
        blocked = navigation_mask(self.grid,self.pose,self.zones,clearance)
        def edge_allowed(a,b):
            if not self.zones or (a!=start and b!=goal):return True
            pa=self.pose if a==start else self.grid.point(a)
            pb=(self.goal.x,self.goal.y) if b==goal else self.grid.point(b)
            return not segment_denied(pa,pb,self.zones,clearance)
        path = astar(self.grid, start, goal, blocked,
                     bool(self.get_parameter('allow_unknown').value),
                     float(self.get_parameter('unknown_cost').value),edge_allowed)
        if path:
            self.path = path
            self.publish_path()
            if self.path:self.emit('path_ready')
            else:self.emit('pathfinding_failed','The route intersects a denied zone')
        else:
            self.fail('pathfinding_failed', 'No A* path from current tile to goal')

    def publish_path(self):
        msg = NavPath()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        points = [self.grid.point(cell) for cell in self.path]
        if points and self.pose:points[0]=tuple(self.pose)
        if points and self.goal:
            # A* uses cell centres; the last target must be the requested position.
            points[-1] = (self.goal.x, self.goal.y)
        clearance=float(self.get_parameter('robot_radius_m').value)+float(self.get_parameter('safety_margin_m').value)
        if any(segment_denied(a,b,self.zones,clearance) for a,b in zip(points,points[1:])):
            self.path=[];points=[]
        for x, y in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        if msg.poses and self.goal.yaw_rad is not None:
            msg.poses[-1].pose.orientation.z = math.sin(self.goal.yaw_rad/2)
            msg.poses[-1].pose.orientation.w = math.cos(self.goal.yaw_rad/2)
        self.pub.publish(msg)
        route = String()
        route.data = json.dumps({'goal_id': self.goal.goal_id if self.goal else None,
                                 'points': points})
        self.route_pub.publish(route)


def main():
    rclpy.init()
    node = GlobalPlannerNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
