from __future__ import annotations
import json, math, queue, socket, threading, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu, JointState
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, SetBool
from fabtino_core.geometry import quaternion_from_euler


def planar_ranges(lidar):
    """Restore null -> +inf, then select a layer for the existing 2D ROS stack.

    The controller/TCP image remains N*L, in its original order. LaserScan has
    one range per horizontal angle: use the central layer (L//2 for even L).
    Never concatenate layers into a false wider scan. A masked column is +inf
    in every layer and stays +inf. This does not implement 3D mapping.
    """
    raw = lidar.get('ranges', [])
    if not raw:
        return []
    ranges = [math.inf if value is None else float(value) for value in raw]
    layers = int(lidar.get('layers', 1))
    resolution = int(lidar.get('resolution', len(ranges)))
    if layers < 1 or resolution < 1 or len(ranges) != layers*resolution:
        raise ValueError('LiDAR image length does not match resolution * layers')
    if layers == 1:
        return ranges
    offset = (layers//2)*resolution
    return ranges[offset:offset+resolution]

class WebotsBridgeNode(Node):
    def __init__(self):
        super().__init__('webots_bridge')
        self.declare_parameter('host','127.0.0.1'); self.declare_parameter('port',8766)
        self.declare_parameter('command_timeout_s',0.40)
        self.declare_parameter('scan_period_s',0.05)
        self.host=str(self.get_parameter('host').value); self.port=int(self.get_parameter('port').value)
        self.command_timeout=float(self.get_parameter('command_timeout_s').value)
        self.scan_period=float(self.get_parameter('scan_period_s').value)
        self.sock=None; self.connected=False; self.rx=queue.Queue(maxsize=8); self.tx=queue.Queue(maxsize=8)
        self.control_tx=queue.Queue(maxsize=32)
        self.motion_lock=threading.Lock()
        self.latest_motion=None
        self.stop_event=threading.Event()
        self.last_cmd=time.monotonic(); self.seq=0
        self.scan_enabled=True
        self.last_scan_sim_time=None
        self.last_sensor_stamp_ns=None
        self.pub_scan=self.create_publisher(LaserScan,'/fabtino/scan',10)
        self.pub_imu=self.create_publisher(Imu,'/fabtino/imu/data_raw',20)
        self.pub_joints=self.create_publisher(JointState,'/fabtino/joint_states',20)
        self.pub_gt=self.create_publisher(Odometry,'/fabtino/ground_truth/odom',10)
        self.pub_status=self.create_publisher(__import__('std_msgs.msg',fromlist=['String']).String,'/fabtino/bridge_status',10)
        self.pub_operation=self.create_publisher(__import__('std_msgs.msg',fromlist=['String']).String,'/viewer/request',10)
        self.sub_cmd=self.create_subscription(Twist,'/fabtino/cmd_vel',self.cmd_cb,20)
        self.sub_scan=self.create_subscription(__import__('std_msgs.msg',fromlist=['Bool']).Bool,'/fabtino/scan_enable',self.scan_cb,10)
        self.reset_srv=self.create_service(Trigger,'/fabtino/reset',self.reset_cb)
        self.thread=threading.Thread(target=self.io_loop,daemon=True); self.thread.start()
        self.timer=self.create_timer(0.01,self.process_rx)
        self.create_timer(0.25,self.status_timer)
        self.get_logger().info(f'Connecting to Webots bridge at {self.host}:{self.port}')
    def cmd_cb(self,msg):
        self.last_cmd=time.monotonic(); self.send({'type':'cmd_vel','v':msg.linear.x,'omega':msg.angular.z})
    def scan_cb(self,msg):
        self.scan_enabled=bool(msg.data)
        self.send({'type':'scan_enable','enabled':self.scan_enabled})
    def reset_cb(self,req,res):
        request_id=f'bridge-reset-{time.time_ns()}'
        self.send({'type':'reset','request_id':request_id})
        msg=__import__('std_msgs.msg',fromlist=['String']).String()
        msg.data=json.dumps(dict(type='reset_odom',request_id=request_id,stamp_ns=self.get_clock().now().nanoseconds))
        self.pub_operation.publish(msg)
        res.success=True; res.message='Estimator and map reset requested; waiting for fresh sensor baseline'; return res
    def send(self,msg):
        if msg.get('type')=='cmd_vel':
            with self.motion_lock:
                self.latest_motion=dict(msg,issued_time_ns=time.time_ns())
        else:
            # Reliable actions must not be discarded by high-rate motor updates.
            self.control_tx.put_nowait(msg)
            if msg.get('type') in ('reset','stop'):
                with self.motion_lock:self.latest_motion=None

    def outbound_commands(self):
        commands=[]
        while True:
            try:commands.append(self.control_tx.get_nowait())
            except queue.Empty:break
        with self.motion_lock:
            motion=self.latest_motion
            self.latest_motion=None
        if motion is not None:
            if time.monotonic()-self.last_cmd>self.command_timeout:
                motion=dict(motion,v=0.,omega=0.,issued_time_ns=time.time_ns())
            commands.append(motion)
        return commands
    def io_loop(self):
        while rclpy.ok() and not self.stop_event.is_set():
            try:
                s=socket.create_connection((self.host,self.port),timeout=1.0); s.settimeout(0.1); self.sock=s; self.connected=True
                self.last_scan_sim_time=None
                # Reapply scan state after reconnecting to a restarted controller.
                s.sendall((json.dumps({'type':'scan_enable','enabled':self.scan_enabled})+'\n').encode())
                buf=b''
                while rclpy.ok() and not self.stop_event.is_set():
                    try:
                        d=s.recv(262144)
                        if d: buf+=d
                        elif d==b'': break
                    except socket.timeout: pass
                    while b'\n' in buf:
                        line,buf=buf.split(b'\n',1)
                        try:
                            packet=json.loads(line.decode())
                            try:self.rx.put_nowait(packet)
                            except queue.Full:
                                try:self.rx.get_nowait()
                                except queue.Empty:pass
                                self.rx.put_nowait(packet)
                        except (ValueError,queue.Full):pass
                    commands=self.outbound_commands()
                    if commands:
                        s.sendall(''.join(json.dumps(command,separators=(',',':'))+'\n' for command in commands).encode())
            except Exception as exc:
                self.connected=False; self.stop_event.wait(0.5)
            finally:
                try:
                    if self.sock:self.sock.close()
                except Exception:pass
                self.sock=None; self.connected=False
    def process_rx(self):
        while True:
            try:p=self.rx.get_nowait()
            except queue.Empty:break
            if p.get('type')!='sensors':continue
            # The controller and ROS bridge run on the same Ubuntu host.
            # Keep capture time through TCP buffering instead of stamping an
            # old measurement as if it had just been acquired.
            stamp=self.get_clock().now().to_msg()
            capture_ns=int(p.get('wall_time_ns', stamp.sec*1_000_000_000+stamp.nanosec))
            if self.last_sensor_stamp_ns is not None and capture_ns <= self.last_sensor_stamp_ns:
                continue
            self.last_sensor_stamp_ns=capture_ns
            stamp.sec,stamp.nanosec=divmod(capture_ns,1_000_000_000)
            self.seq=p.get('seq',self.seq)
            js=JointState(); js.header.stamp=stamp; js.name=['front_left_wheel_joint','front_right_wheel_joint','back_left_wheel_joint','back_right_wheel_joint']
            w=p.get('wheels',{}); js.position=[float(w.get('fl',0)),float(w.get('fr',0)),float(w.get('rl',0)),float(w.get('rr',0))]; self.pub_joints.publish(js)
            imu=Imu(); imu.header.stamp=stamp; imu.header.frame_id='imu_link'
            i=p.get('imu',{}); a=i.get('acceleration',[0,0,0]); g=i.get('angular_velocity',[0,0,0]);
            imu.linear_acceleration.x=float(a[0]); imu.linear_acceleration.y=float(a[1]); imu.linear_acceleration.z=float(a[2]); imu.angular_velocity.x=float(g[0]); imu.angular_velocity.y=float(g[1]); imu.angular_velocity.z=float(g[2])
            q=quaternion_from_euler(float(i.get('roll',0)),float(i.get('pitch',0)),float(i.get('yaw',0))); imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w=map(float,q); self.pub_imu.publish(imu)
            li=p.get('lidar',{})
            try:
                ranges=planar_ranges(li)
            except (ValueError, TypeError) as exc:
                self.get_logger().error(f'Rejecting invalid LiDAR image: {exc}')
                continue
            sim_time = float(p.get('sim_time', 0.0))
            publish_scan = (
                ranges
                and (
                    self.last_scan_sim_time is None
                    or sim_time - self.last_scan_sim_time >= self.scan_period
                )
            )
            if publish_scan:
                self.last_scan_sim_time = sim_time
                scan=LaserScan(); scan.header.stamp=stamp; scan.header.frame_id='lidar_link'; scan.angle_min=-float(li.get('fov',6.283185307179586))/2; scan.angle_max=float(li.get('fov',6.283185307179586))/2
                scan.angle_increment=(scan.angle_max-scan.angle_min)/max(1,len(ranges)-1)
                scan.range_min=float(li.get('min_range',0.2)); scan.range_max=float(li.get('max_range',12.0)); scan.ranges=[float(x) for x in ranges]; self.pub_scan.publish(scan)
            gt=p.get('ground_truth')
            if gt:
                od=Odometry(); od.header.stamp=stamp; od.header.frame_id='map'; od.child_frame_id='ground_truth/base_link'; od.pose.pose.position.x=float(gt[0]); od.pose.pose.position.y=float(gt[1]); od.pose.pose.position.z=float(gt[2]); q=quaternion_from_euler(0,0,float(gt[3])); od.pose.pose.orientation.x,od.pose.pose.orientation.y,od.pose.pose.orientation.z,od.pose.pose.orientation.w=map(float,q); self.pub_gt.publish(od)
    def status_timer(self):
        msg=__import__('std_msgs.msg',fromlist=['String']).String()
        msg.data=json.dumps({'connected':self.connected,'scan_enabled':self.scan_enabled,
                             'sensor_stamp_ns':self.last_sensor_stamp_ns})
        self.pub_status.publish(msg)

    def close(self):
        self.stop_event.set()
        if self.sock is not None:
            try:self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:pass
            self.sock.close()
        if self.thread.is_alive():self.thread.join(timeout=2.)
def main():
    rclpy.init(); n=WebotsBridgeNode();
    try:rclpy.spin(n)
    finally:n.close();n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
