import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1] / "ros_ws" / "src" / "fabtino_viewer"))
import rclpy
from sensor_msgs.msg import PointCloud2, PointField

from fabtino_viewer.viewer_gateway import ViewerGateway


def test_gateway_decodes_map_frame_xyz_without_pose_transform():
    msg = PointCloud2()
    msg.header.frame_id = "map"
    msg.height = 1
    msg.width = 2
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 12
    msg.row_step = 24
    msg.data = b"".join(struct.pack("<fff", *point) for point in ((1.0, 2.0, 0.0), (-3.0, 4.0, 0.5)))

    assert ViewerGateway._cloud_points(msg) == [[1.0, 2.0, 0.0], [-3.0, 4.0, 0.5]]
