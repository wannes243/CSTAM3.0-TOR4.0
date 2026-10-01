from __future__ import annotations

from collections import deque
import json
import math
import struct
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String, Bool

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from fabtino_core.localization.frames import (
    interpolate_pose,
    lidar_polar_to_robot,
    robot_to_world_point,
)
from fabtino_core.navigation.planner import OccupancyGrid as CoreGrid
from fabtino_core.operations import imported_grid
from fabtino_core.operations import map_dimensions
from fabtino_core.static_points import StaticPointFilter


class MappingNode(Node):
    """Integrate each timestamped scan exactly once in the fixed map frame."""

    def __init__(self):
        super().__init__('mapping')
        self.declare_parameter('resolution_m', 0.10)
        self.declare_parameter('radius_m', 20.0)
        self.declare_parameter('lidar_angle_sign', -1.0)
        self.declare_parameter('lidar_subsample', 2)
        self.declare_parameter('publish_period_s', 0.10)
        self.declare_parameter('map_publish_period_s', 0.50)
        self.declare_parameter('scan_queue_size', 256)
        self.declare_parameter('pose_history_size', 256)
        self.declare_parameter('pose_match_tolerance_s', 0.05)
        self.declare_parameter('static_persistence_s',3.0)
        self.declare_parameter('static_observation_gap_s',.45)
        self.declare_parameter('dynamic_obstacle_lifetime_s',.5)

        resolution = float(self.get_parameter('resolution_m').value)
        radius = float(self.get_parameter('radius_m').value)
        self.lidar_angle_sign = float(self.get_parameter('lidar_angle_sign').value)
        self.lidar_subsample = max(1, int(self.get_parameter('lidar_subsample').value))
        if self.lidar_angle_sign not in (-1.0, 1.0):
            raise ValueError('lidar_angle_sign must be -1.0 or 1.0')
        self.pose_match_tolerance_s = float(self.get_parameter('pose_match_tolerance_s').value)
        self.map_publish_period_s = max(0.05, float(self.get_parameter('map_publish_period_s').value))
        self.last_map_publish_time = 0.0
        self.scan_queue = deque(maxlen=max(1, int(self.get_parameter('scan_queue_size').value)))
        self.pose_history = deque(maxlen=max(2, int(self.get_parameter('pose_history_size').value)))
        self.next_scan_sequence = 0
        self.pose = (0.0, 0.0, 0.0)
        self.pose_stable = True
        self.grid = CoreGrid(resolution, radius)
        self.static_filter=StaticPointFilter(resolution,float(self.get_parameter('static_persistence_s').value),
                                              float(self.get_parameter('static_observation_gap_s').value))
        self.dynamic_cells={}
        self.dynamic_lifetime_ns=round(float(self.get_parameter('dynamic_obstacle_lifetime_s').value)*1e9)
        self.revision = 0
        self.map_id='startup'
        self.mode='live'
        self.minimum_scan_stamp_ns=0
        self.pending_import=None

        self.pub_map = self.create_publisher(OccupancyGrid, '/fabtino/map', 3)
        self.pub_local = self.create_publisher(OccupancyGrid, '/fabtino/local_costmap', 3)
        self.pub_points = self.create_publisher(PointCloud2, '/fabtino/pointcloud', 3)
        self.pub_live_points=self.create_publisher(PointCloud2,'/fabtino/live_pointcloud',3)
        self.pub_navigation=self.create_publisher(OccupancyGrid,'/fabtino/navigation_map',3)
        self.pub_status = self.create_publisher(String, '/fabtino/mapping_status', 3)
        self.pub_operation=self.create_publisher(String,'/viewer/operation_status',10)
        self.create_subscription(String,'/viewer/request',self.request_cb,10)
        self.create_subscription(String,'/viewer/operation_status',self.operation_cb,10)
        self.create_subscription(LaserScan, '/fabtino/scan', self.scan_cb, 10)
        self.create_subscription(Bool, '/fabtino/pose_stable', self.pose_stable_cb, 10)
        self.create_subscription(Odometry, '/fabtino/odometry/filtered', self.pose_cb, 20)
        self.create_subscription(
            __import__('std_msgs.msg', fromlist=['Empty']).Empty,
            '/mapping/clear',
            self.clear_cb,
            2,
        )
        self.create_timer(float(self.get_parameter('publish_period_s').value), self.update)

    @staticmethod
    def stamp_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def pose_cb(self, msg):
        from fabtino_core.geometry import euler_from_quaternion

        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        timestamp = self.stamp_seconds(msg.header.stamp)
        if msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec<getattr(self,'minimum_scan_stamp_ns',0):return
        pose = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y), float(yaw))
        sample = (timestamp, pose[0], pose[1], pose[2])
        if self.pose_history and timestamp < self.pose_history[-1][0]:
            self.get_logger().warning('Ignoring out-of-order odometry sample')
            return
        self.pose = pose
        # Relative tolerance at Unix-epoch times treats samples up to ~1.8 s
        # apart as equal, collapsing the entire history during a turn.
        if self.pose_history and timestamp == self.pose_history[-1][0]:
            self.pose_history[-1] = sample
        else:
            self.pose_history.append(sample)

    def scan_cb(self, msg):
        if msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec<getattr(self,'minimum_scan_stamp_ns',0):return
        if not msg.ranges:
            return
        self.next_scan_sequence += 1
        if len(self.scan_queue) == self.scan_queue.maxlen:
            dropped_sequence, _ = self.scan_queue.popleft()
            self.get_logger().warning(
                f'Dropping scan sequence {dropped_sequence}: bounded scan queue is full'
            )
        self.scan_queue.append((self.next_scan_sequence, msg))

    def pose_stable_cb(self, msg):
        self.pose_stable = bool(msg.data)

    def pose_for_scan(self, scan_timestamp: float):
        """Use the acquisition pose, never a newer/older pose extrapolation."""
        if not self.pose_history:
            return None, 'no_pose_history'
        first = self.pose_history[0]
        last = self.pose_history[-1]
        tolerance = self.pose_match_tolerance_s
        if scan_timestamp < first[0]:
            return None, 'scan_older_than_pose_history'
        if scan_timestamp > last[0]:
            return None, 'waiting_for_newer_pose'
        for sample in self.pose_history:
            if scan_timestamp == sample[0]:
                return tuple(sample[1:]), 'exact_pose'
        samples = list(self.pose_history)
        for before, after in zip(samples, samples[1:]):
            if before[0] <= scan_timestamp <= after[0]:
                if after[0] - before[0] > tolerance:
                    return None, 'pose_gap_too_large'
                return interpolate_pose(before, after, scan_timestamp), 'interpolated_pose'
        return None, 'waiting_for_newer_pose'

    def update(self):
        # Drain each queued scan once. Publishing the map may remain periodic,
        # but sensor data is never reintegrated merely because the timer fires.
        while self.scan_queue:
            sequence, scan = self.scan_queue[0]
            scan_timestamp = self.stamp_seconds(scan.header.stamp)
            pose, pose_status = self.pose_for_scan(scan_timestamp)
            if pose is None:
                if pose_status in ('scan_older_than_pose_history', 'pose_gap_too_large'):
                    self.scan_queue.popleft()
                    self.get_logger().warning(
                        f'Rejecting scan sequence {sequence}: no pose for scan timestamp '
                        f'{scan_timestamp:.6f}'
                    )
                    continue
                break
            self.scan_queue.popleft()
            self.integrate_scan(sequence, scan, pose, pose_status)

        now = time.monotonic()
        if now - self.last_map_publish_time >= self.map_publish_period_s:
            self.publish_grid(self.pub_map, self.grid.data, self.grid.resolution, self.grid.radius, 'map')
            self.publish_local()
            self.last_map_publish_time = now
        status = String()
        status.data = json.dumps(dict(revision=self.grid.revision,robot_x=self.pose[0],robot_y=self.pose[1],
                                     queued_scans=len(self.scan_queue),map_id=getattr(self,'map_id','startup'),
                                     mode=getattr(self,'mode','live'),static_persistence_s=self.static_filter.persistence_ns*1e-9,
                                     pending_static_points=len(self.static_filter.candidates)))
        self.pub_status.publish(status)

    def integrate_scan(self, sequence, scan, pose, pose_status):
        stamp_ns=scan.header.stamp.sec*1_000_000_000+scan.header.stamp.nanosec
        if self.mode=='live' and self.static_filter.last_ns is not None and stamp_ns<=self.static_filter.last_ns:
            return
        x0, y0, yaw = pose
        points = []
        valid_returns = 0
        for index in range(0, len(scan.ranges), self.lidar_subsample):
            distance = scan.ranges[index]
            distance = float(distance)
            if not math.isfinite(distance) or distance < scan.range_min or distance > scan.range_max:
                continue
            angle = float(scan.angle_min + index * scan.angle_increment)
            robot_point = lidar_polar_to_robot(distance, angle, self.lidar_angle_sign)
            wx, wy = robot_to_world_point(pose, robot_point)
            points.append((wx, wy, 0.0))
            valid_returns += 1

        cells={}
        for wx,wy,_ in points:
            cell=self.grid.cell(wx,wy)
            if cell is not None:cells.setdefault(cell,[]).append((wx,wy))
        confirmed=set()
        if self.mode=='live':
            freed=set()
            for wx,wy,_ in points:
                freed.update(self.grid.update_ray(x0,y0,wx,wy,mark_endpoint=False,
                    clear_occupied=True,protected_cells=cells))
            self.static_filter.forget(freed)
            confirmed=self.static_filter.observe(cells,stamp_ns)
            for cell in confirmed:self.grid.set_cell(cell,100)
        else:confirmed={cell for cell in cells if self.grid.data[cell[1],cell[0]]==100}
        for cell in cells:self.dynamic_cells[cell]=stamp_ns

        self.revision += 1
        self.publish_cloud([point for point in points if self.grid.cell(point[0],point[1]) in confirmed], scan.header.stamp)
        self.publish_cloud(points,scan.header.stamp,self.pub_live_points)
        self.publish_local()
        self.get_logger().debug(
            f'SCAN INTEGRATED: seq={sequence} scan_t={self.stamp_seconds(scan.header.stamp):.6f} '
            f'pose=({x0:.3f},{y0:.3f},{yaw:.3f}) pose_source={pose_status} '
            f'valid_returns={valid_returns}'
        )

    def publish_grid(self, publisher, data, resolution, radius, frame):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame
        msg.info.resolution = float(resolution)
        msg.info.width = int(self.grid.size)
        msg.info.height = int(self.grid.size)
        msg.info.origin.position.x = -float(radius)
        msg.info.origin.position.y = -float(radius)
        msg.info.origin.orientation.w=1.0
        msg.data = [int(value) for value in np.asarray(data).reshape(-1)]
        publisher.publish(msg)

    def publish_local(self):
        now=self.get_clock().now().nanoseconds
        data=self.grid.data.copy()
        for cell,stamp in list(self.dynamic_cells.items()):
            if now-stamp>self.dynamic_lifetime_ns:
                del self.dynamic_cells[cell]
            else:data[cell[1],cell[0]]=100
        self.publish_grid(self.pub_local,data,self.grid.resolution,self.grid.radius,'map')
        self.publish_grid(self.pub_navigation,data,self.grid.resolution,self.grid.radius,'map')

    def publish_cloud(self, points, stamp, publisher=None):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        # The current sensor path is a 2D LaserScan; Z is explicitly flat,
        # rather than fabricated from an assumed LiDAR height.
        buffer = b''.join(struct.pack('<fff', float(x), float(y), float(z)) for x, y, z in points)
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.height = 1
        msg.width = len(points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(points)
        msg.is_dense = True
        msg.data = buffer
        (self.pub_points if publisher is None else publisher).publish(msg)

    def clear_cb(self, msg):
        self.grid.clear()
        self.scan_queue.clear()
        self.mode='live'
        self.pending_import=None
        self.static_filter=StaticPointFilter(self.grid.resolution,self.static_filter.persistence_ns*1e-9,self.static_filter.max_gap_ns*1e-9)
        self.dynamic_cells.clear()
        self.map_id=str(self.get_clock().now().nanoseconds)
        self.publish_grid(self.pub_map,self.grid.data,self.grid.resolution,self.grid.radius,'map')
        self.publish_cloud([],self.get_clock().now().to_msg())
        self.publish_cloud([],self.get_clock().now().to_msg(),self.pub_live_points)
        self.publish_local()
        self.last_map_publish_time=0.
        self.get_logger().info('Cleared fixed map and pending scan queue')

    def operation_status(self,request,state,reason=''):
        msg=String()
        msg.data=json.dumps(dict(node='mapping',request_id=request['request_id'],type=request['type'],
                                state=state,reason=reason,map_id=self.map_id,mode=self.mode,
                                stamp_ns=self.get_clock().now().nanoseconds))
        self.pub_operation.publish(msg)

    def request_cb(self,msg):
        request=json.loads(msg.data)
        kind=request.get('type')
        if kind=='cancel_operation':
            if self.pending_import is not None and self.pending_import[0]['request_id']==request.get('request_id'):
                self.pending_import=None
            return
        if kind not in ('clear_map','reset_odom','nav_map','nav_clear_map','configure_map'):return
        if kind=='nav_map':
            try:
                self.pending_import=(request,imported_grid(request))
                self.operation_status(request,'pending')
            except (ValueError,KeyError,TypeError) as exc:self.operation_status(request,'error',str(exc))
            return
        if kind=='configure_map':
            try:
                resolution,radius,_=map_dimensions(request)
                self.grid=CoreGrid(resolution,radius)
            except (ValueError,KeyError,TypeError) as exc:
                self.operation_status(request,'error',str(exc));return
        self.minimum_scan_stamp_ns=int(request['stamp_ns'])
        if kind=='reset_odom':self.pose_history.clear()
        self.clear_cb(msg)
        self.map_id=request['request_id']
        self.operation_status(request,'complete')

    def operation_cb(self,msg):
        status=json.loads(msg.data)
        if self.pending_import is None or status.get('node')!='localization':return
        request,grid=self.pending_import
        if status.get('request_id')!=request['request_id']:return
        if status.get('state')=='error':
            self.pending_import=None
        elif status.get('state')=='complete':
            self.grid=grid
            self.mode='known'
            self.map_id=request['request_id']
            self.scan_queue.clear()
            self.pose_history.clear()
            self.minimum_scan_stamp_ns=self.get_clock().now().nanoseconds
            self.pending_import=None
            self.static_filter.clear();self.dynamic_cells.clear()
            self.publish_grid(self.pub_map,self.grid.data,self.grid.resolution,self.grid.radius,'map')
            self.publish_cloud([],self.get_clock().now().to_msg())
            self.publish_cloud([],self.get_clock().now().to_msg(),self.pub_live_points)
            self.publish_local()
            self.last_map_publish_time=0.
            self.operation_status(request,'complete')


def main():
    rclpy.init()
    node = MappingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
