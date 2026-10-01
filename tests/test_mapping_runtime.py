import math
import sys
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1] / "ros_ws" / "src" / "fabtino_core"))
sys.path.insert(0, str(Path(__file__).parents[1] / "ros_ws" / "src" / "fabtino_mapping"))
import rclpy
from sensor_msgs.msg import LaserScan

from fabtino_mapping.mapping_node import MappingNode
from fabtino_core.navigation.planner import OccupancyGrid
from fabtino_core.static_points import StaticPointFilter


class _Logger:
    def debug(self, message):
        pass

    def warning(self, message):
        pass


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _bare_mapping_node():
    node = object.__new__(MappingNode)
    node.pose_match_tolerance_s = 0.05
    node.scan_queue = deque(maxlen=8)
    node.pose_history = deque(maxlen=8)
    node.next_scan_sequence = 0
    node.pose = (0.0, 0.0, 0.0)
    node.grid = OccupancyGrid(0.1, 5.0)
    node.revision = 0
    node.lidar_angle_sign = -1.0
    node.lidar_subsample = 1
    node.last_map_publish_time = 0.0
    node.map_publish_period_s = 0.5
    node.pub_points = _Publisher()
    node.pub_live_points=_Publisher()
    node.mode='live';node.dynamic_cells={};node.static_filter=StaticPointFilter(.1)
    node.pub_map = _Publisher()
    node.pub_local = _Publisher()
    node.pub_status = _Publisher()
    node.get_logger = lambda: _Logger()
    node.publish_grid = lambda *args: None
    node.publish_local = lambda: None
    return node


def _scan(timestamp, distance=2.0):
    msg = LaserScan()
    msg.header.stamp.sec = int(timestamp)
    msg.header.stamp.nanosec = int(round((timestamp - int(timestamp)) * 1e9))
    msg.angle_min = 0.0
    msg.angle_increment = 0.0
    msg.range_min = 0.1
    msg.range_max = 10.0
    msg.ranges = [distance]
    return msg


def test_one_scan_is_integrated_once_even_when_update_runs_twice():
    node = _bare_mapping_node()
    node.pose_history.append((1.0, 0.0, 0.0, 0.0))
    node.scan_queue.append((1, _scan(1.0)))

    node.update()
    first_grid = node.grid.data.copy()
    first_cloud_count = len(node.pub_points.messages)
    node.update()

    assert node.grid.revision == 2  # one ray update plus initial grid revision
    assert (node.grid.data == first_grid).all()
    assert len(node.pub_points.messages) == first_cloud_count == 1


def test_scan_pose_is_interpolated_at_scan_timestamp():
    node = _bare_mapping_node()
    node.pose_history.extend([
        (1.0, 0.0, 0.0, 0.0),
        (1.04, 2.0, 0.0, math.pi),
    ])
    pose, source = node.pose_for_scan(1.02)

    assert source == 'interpolated_pose'
    assert math.isclose(pose[0], 1.0, abs_tol=1e-12)
    assert math.isclose(pose[1], 0.0, abs_tol=1e-12)
    assert math.isclose(abs(pose[2]), math.pi / 2, abs_tol=1e-12)
