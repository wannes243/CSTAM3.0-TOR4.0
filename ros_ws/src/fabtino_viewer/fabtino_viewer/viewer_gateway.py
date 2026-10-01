from __future__ import annotations

import asyncio
import base64
import json
import math
import struct
import threading
import time
import uuid

import numpy as np
import websockets
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
from fabtino_core.operations import imported_grid,map_dimensions
from fabtino_core.navigation.zones import validate_config


class ViewerGateway(Node):
    """Expose fixed-map ROS data to the existing WebSocket viewer."""

    def __init__(self):
        super().__init__('viewer_gateway')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8765)
        self.declare_parameter('map_period_s', 0.5)
        self.declare_parameter('start_server',True)
        self.ws_clients = set()
        self.latest = {}
        self.cloud_sequence = 0
        self.last_cloud_sequence_sent = 0
        self.last_map_sent = 0.0
        self.latest_live_cloud=None
        self.operation=None
        self.operation_nodes={}
        self.minimum_map_stamp_ns=0
        self.minimum_odom_stamp_ns=0
        self.server_ready=threading.Event()
        self.pub_operation=self.create_publisher(String,'/viewer/request',10)
        self.create_subscription(String,'/viewer/operation_status',self.operation_cb,10)
        self.create_subscription(String,'/fabtino/mapping_status',self.mapping_cb,10)
        self.create_subscription(String,'/fabtino/bridge_status',self.bridge_cb,10)

        self.pub_teleop = self.create_publisher(TwistStamped, '/teleop/cmd_vel', 20)
        self.pub_scan_enable = self.create_publisher(
            __import__('std_msgs.msg', fromlist=['Bool']).Bool,
            '/fabtino/scan_enable',
            5,
        )
        self.pub_clear = self.create_publisher(
            __import__('std_msgs.msg', fromlist=['Empty']).Empty,
            '/mapping/clear',
            2,
        )
        self.pub_req = self.create_publisher(String, '/navigation/request', 10)

        self.create_subscription(
            Odometry, '/fabtino/odometry/filtered',
            lambda msg: self.latest.__setitem__('odom', msg), 20,
        )
        self.create_subscription(
            Odometry, '/fabtino/ground_truth/odom',
            lambda msg: self.latest.__setitem__('gt', msg), 10,
        )
        self.create_subscription(
            LaserScan, '/fabtino/scan',
            lambda msg: self.latest.__setitem__('scan', msg), 10,
        )
        self.create_subscription(PointCloud2, '/fabtino/pointcloud', self.cloud_cb, 3)
        self.create_subscription(PointCloud2,'/fabtino/live_pointcloud',lambda msg:setattr(self,'latest_live_cloud',msg),3)
        self.create_subscription(String,'/navigation/config',lambda msg:self.latest.__setitem__('navigation_config',json.loads(msg.data)),10)
        self.create_subscription(
            OccupancyGrid, '/fabtino/map',
            lambda msg: self.latest.__setitem__('map', msg), 3,
        )
        self.create_subscription(
            String, '/fabtino/safety_status',
            lambda msg: self.latest.__setitem__('safety', msg.data), 10,
        )
        self.create_subscription(
            String, '/navigation/mission_status',
            lambda msg: self.latest.__setitem__('nav', msg.data), 10,
        )
        self.create_timer(0.1, self.push)
        self.thread = threading.Thread(target=self.ws_thread, daemon=True)
        if self.get_parameter('start_server').value:self.thread.start()

    def mapping_cb(self,msg):
        self.latest['mapping']=json.loads(msg.data)

    def bridge_cb(self,msg):
        self.latest['bridge']=json.loads(msg.data)

    def ensure_scan(self):
        if not hasattr(self,'pub_scan_enable'):return
        if not self.latest.get('scan_enabled',False):
            message=__import__('std_msgs.msg',fromlist=['Bool']).Bool()
            message.data=True
            self.pub_scan_enable.publish(message)
            self.latest['scan_enabled']=True

    def operation_cb(self,msg):
        status=json.loads(msg.data)
        if self.operation is None or status.get('request_id')!=self.operation['request_id']:return
        self.operation_nodes[status['node']]=status
        state=status.get('state')
        if state=='error':
            self.latest['operation']=dict(self.operation,state='error',reason=status.get('reason','Operation failed'))
            self.operation=None
            return
        required={'mapping'} if self.operation['type'] in ('clear_map','configure_map') else {'mapping','localization'}
        if self.operation['type'] in ('nav_map','reset_odom'):required.add('mission')
        if all(self.operation_nodes.get(node,{}).get('state')=='complete' for node in required):
            self.latest['operation']=dict(self.operation,state='complete',reason='')
            self.latest.setdefault('mapping',{}).update({key:self.operation_nodes['mapping'][key]
                for key in ('map_id','mode') if key in self.operation_nodes['mapping']})
            self.minimum_map_stamp_ns=int(self.operation_nodes['mapping'].get('stamp_ns',self.operation['stamp_ns']))
            self.latest.pop('map',None)
            self.latest.pop('cloud',None)
            self.last_map_sent=0.
            if self.operation['type']=='nav_map':
                self.minimum_odom_stamp_ns=self.get_clock().now().nanoseconds
                self.latest['scan_matching']=dict(known_map_global_localized=True,
                    known_map_request_id=self.operation['request_id'])
            else:self.latest.pop('scan_matching',None)
            self.operation=None

    def cancel_operation(self,reason):
        if self.operation is None:return
        request=dict(self.operation,type='cancel_operation')
        message=String();message.data=json.dumps(request)
        self.pub_operation.publish(message)
        self.latest['operation']=dict(self.operation,state='error',reason=reason)
        self.operation=None

    def cloud_cb(self, msg):
        self.latest['cloud'] = msg
        self.cloud_sequence += 1

    @staticmethod
    def _cloud_points(cloud):
        """Decode the mapper's x/y/z PointCloud2 without applying a pose transform."""
        fields = {field.name: field for field in cloud.fields}
        if not all(name in fields for name in ('x', 'y', 'z')):
            return []
        formats = {
            7: ('f', 4),   # sensor_msgs/PointField.FLOAT32
            8: ('d', 8),   # sensor_msgs/PointField.FLOAT64
        }
        prefix = '>' if cloud.is_bigendian else '<'
        decoded = []
        count = int(cloud.width) * int(cloud.height)
        for index in range(count):
            base = index // max(1, int(cloud.width)) * int(cloud.row_step)
            base += (index % max(1, int(cloud.width))) * int(cloud.point_step)
            values = []
            try:
                for name in ('x', 'y', 'z'):
                    field = fields[name]
                    fmt, _ = formats[int(field.datatype)]
                    values.append(struct.unpack_from(prefix + fmt, cloud.data, base + int(field.offset))[0])
            except (KeyError, struct.error, IndexError, ValueError):
                continue
            if all(np.isfinite(value) for value in values):
                decoded.append([float(values[0]), float(values[1]), float(values[2])])
        return decoded

    def cmd(self, typ, payload):
        if typ in ('clear_map','reset_odom','nav_map','nav_clear_map','configure_map'):
            request_id=str(payload.get('request_id') or uuid.uuid4())
            if len(request_id)>128:raise ValueError('Invalid request identifier')
            request=dict(payload,type=typ,request_id=request_id,stamp_ns=self.get_clock().now().nanoseconds)
            if typ=='nav_map':
                imported_grid(request)
                if 'navigation_config' in request:request['navigation_config']=validate_config(request['navigation_config'])
            if typ=='configure_map':map_dimensions(request)
            self.cancel_operation('Operation replaced by a newer request')
            self.cmd('stop',{})
            self.ensure_scan()
            self.operation={key:request[key] for key in ('type','request_id','stamp_ns')}
            self.operation_nodes={}
            self.latest['operation']=dict(self.operation,state='pending',reason='Waiting for robot confirmation')
            if typ!='nav_map':
                self.latest.pop('map',None)
                self.minimum_map_stamp_ns=request['stamp_ns']
            if typ=='reset_odom':self.minimum_odom_stamp_ns=request['stamp_ns']
            message=String();message.data=json.dumps(request,allow_nan=False)
            self.pub_operation.publish(message)
            return
        if (getattr(self,'operation',None) is not None
                and typ in ('drive','nav_goal','nav_explore','delivery_start','delivery_return','set_navigation_config')
                and (typ!='drive' or payload.get('v') or payload.get('omega'))):
            raise ValueError('Map/reset operation in progress; wait for confirmation before moving')
        if typ in ('set_navigation_config','delivery_start','delivery_return'):
            request=dict(payload,type='configure' if typ=='set_navigation_config' else typ)
            if typ=='set_navigation_config':request.update(validate_config(payload))
            else:self.ensure_scan()
            msg=String();msg.data=json.dumps(request,allow_nan=False);self.pub_req.publish(msg);return
        if typ in ('drive', 'stop'):
            if typ == 'stop':
                request = String()
                request.data = json.dumps({'type': 'stop'})
                self.pub_req.publish(request)
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = float(payload.get('v', 0) if typ == 'drive' else 0)
            msg.twist.angular.z = float(payload.get('omega', 0) if typ == 'drive' else 0)
            if not math.isfinite(msg.twist.linear.x) or not math.isfinite(msg.twist.angular.z):
                raise ValueError('Drive velocities must be finite')
            msg.twist.linear.x=max(-.5,min(.5,msg.twist.linear.x))
            msg.twist.angular.z=max(-1.5,min(1.5,msg.twist.angular.z))
            if msg.twist.linear.x or msg.twist.angular.z:self.ensure_scan()
            self.pub_teleop.publish(msg)
            return
        if typ in ('toggle_scan', 'set_scan'):
            enabled = bool(payload.get('enabled', not self.latest.get('scan_enabled', True)))
            self.latest['scan_enabled'] = enabled
            scan_msg = __import__('std_msgs.msg', fromlist=['Bool']).Bool()
            scan_msg.data = enabled
            self.pub_scan_enable.publish(scan_msg)
            return
        if typ in ('nav_goal', 'nav_explore', 'nav_stop'):
            if typ!='nav_stop':self.ensure_scan()
            msg = String()
            msg.data = json.dumps({
                'type': 'goal' if typ == 'nav_goal' else 'explore' if typ == 'nav_explore' else 'stop',
                'x': payload.get('x'),
                'y': payload.get('y'),
                'yaw_deg': payload.get('yaw_deg'),
            })
            self.pub_req.publish(msg)

    def packet(self):
        odom = self.latest.get('odom')
        if odom is not None and odom.header.stamp.sec*1_000_000_000+odom.header.stamp.nanosec<getattr(self,'minimum_odom_stamp_ns',0):odom=None
        ground_truth = self.latest.get('gt')
        scan = self.latest.get('scan')
        occupancy_map = self.latest.get('map')
        from fabtino_core.geometry import euler_from_quaternion
        yaw=0.
        if odom is not None:
            q=odom.pose.pose.orientation
            _,_,yaw=euler_from_quaternion([q.x,q.y,q.z,q.w])
        packet = {
            'type': 'scan',
            'scanning': scan is not None and self.latest.get('scan_enabled', True),
            'points_frame': 'map',
            'new_points': [],
            'live_points':[],
            'navigation': {},
            'safety': {},
            'operation':self.latest.get('operation'),
            'mapping':self.latest.get('mapping',{}),
            'bridge':self.latest.get('bridge',{}),
            'scan_matching':self.latest.get('scan_matching',{}),
            'navigation_config':self.latest.get('navigation_config'),
        }
        if odom is not None:
            packet['robot']=dict(x=odom.pose.pose.position.x,y=odom.pose.pose.position.y,theta=yaw)

        if ground_truth is not None and odom is not None:
            _, _, truth_yaw = euler_from_quaternion([
                ground_truth.pose.pose.orientation.x,
                ground_truth.pose.pose.orientation.y,
                ground_truth.pose.pose.orientation.z,
                ground_truth.pose.pose.orientation.w,
            ])
            packet['ground_truth'] = {
                'x': ground_truth.pose.pose.position.x,
                'y': ground_truth.pose.pose.position.y,
                'theta': truth_yaw,
            }
            packet['estimation_error'] = {
                'position_m': (
                    (ground_truth.pose.pose.position.x - odom.pose.pose.position.x) ** 2
                    + (ground_truth.pose.pose.position.y - odom.pose.pose.position.y) ** 2
                ) ** 0.5,
                'yaw_rad': abs(((truth_yaw - yaw + 3.1415926535) % (2 * 3.1415926535)) - 3.1415926535),
            }

        if self.latest.get('nav'):
            try:
                packet['navigation'] = json.loads(self.latest['nav'])
            except Exception:
                pass
        if self.latest.get('safety'):
            try:
                packet['safety'] = json.loads(self.latest['safety'])
            except Exception:
                pass

        if (
            occupancy_map is not None
            and (occupancy_map.header.stamp.sec*1_000_000_000+occupancy_map.header.stamp.nanosec)>=getattr(self,'minimum_map_stamp_ns',0)
            and time.monotonic() - self.last_map_sent >= float(self.get_parameter('map_period_s').value)
        ):
            raw = np.asarray(occupancy_map.data, dtype=np.int8).tobytes()
            packet['map'] = {
                'resolution': occupancy_map.info.resolution,
                'radius': -occupancy_map.info.origin.position.x,
                'width': occupancy_map.info.width,
                'height': occupancy_map.info.height,
                'data_b64': base64.b64encode(raw).decode(),
                'map_id':self.latest.get('mapping',{}).get('map_id','startup'),
            }
            self.last_map_sent = time.monotonic()

        if self.cloud_sequence > self.last_cloud_sequence_sent:
            cloud = self.latest.get('cloud')
            if cloud is not None and cloud.header.stamp.sec*1_000_000_000+cloud.header.stamp.nanosec<getattr(self,'minimum_map_stamp_ns',0):cloud=None
            packet['new_points'] = self._cloud_points(cloud) if cloud is not None else []
            packet['points_frame'] = 'map'
            self.last_cloud_sequence_sent = self.cloud_sequence

        live=getattr(self,'latest_live_cloud',None)
        if live is not None:
            stamp=live.header.stamp.sec*1_000_000_000+live.header.stamp.nanosec
            if stamp>=getattr(self,'minimum_map_stamp_ns',0) and 0<=self.get_clock().now().nanoseconds-stamp<500_000_000:
                packet['live_points']=self._cloud_points(live)

        return packet

    def push(self):
        if self.operation is not None and self.get_clock().now().nanoseconds-self.operation['stamp_ns']>30_000_000_000:
            self.cancel_operation('No robot confirmation within 30 seconds; check sensors and retry')
        if not self.ws_clients:return
        packet = self.packet()
        if packet is None or not self.ws_clients:
            return
        asyncio.run_coroutine_threadsafe(self.broadcast(packet), self.loop)

    async def broadcast(self, packet):
        raw = json.dumps(packet, separators=(',', ':'))
        dead = []
        for client in list(self.ws_clients):
            try:
                await client.send(raw)
            except Exception:
                dead.append(client)
        for client in dead:
            self.ws_clients.discard(client)

    async def ws_handler(self, websocket):
        self.ws_clients.add(websocket)
        try:
            async for raw in websocket:
                payload=None
                try:
                    payload = json.loads(raw)
                    self.cmd(payload.get('type'), payload)
                except Exception as exc:
                    self.get_logger().warning(f'viewer message error: {exc}')
                    await websocket.send(json.dumps(dict(type='command_result',state='error',
                        request_id=payload.get('request_id') if isinstance(payload,dict) else None,
                        reason=str(exc))))
        finally:
            self.ws_clients.discard(websocket)

    def ws_thread(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        async def run():
            self.server_stop=asyncio.Event()
            async with websockets.serve(
                self.ws_handler,
                str(self.get_parameter('host').value),
                int(self.get_parameter('port').value),
                max_size=16 * 1024 * 1024,
            ) as server:
                self.server_port=server.sockets[0].getsockname()[1]
                self.server_ready.set()
                await self.server_stop.wait()

        self.loop.run_until_complete(run())
        self.loop.close()

    def close(self):
        if self.thread.is_alive() and self.server_ready.wait(timeout=2.):
            self.loop.call_soon_threadsafe(self.server_stop.set)
            self.thread.join(timeout=3.)


def main():
    rclpy.init()
    node = ViewerGateway()
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
