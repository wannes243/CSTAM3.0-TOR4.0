from __future__ import annotations
import json, math, sys
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, LaserScan
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from fabtino_core.geometry import euler_from_quaternion, quaternion_from_euler
ROOT=Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path: sys.path.insert(0,str(ROOT))
from fabtino_core.localization.ekf import EKFConfig, LocalizationEKF
from fabtino_core.localization.scan_matcher import LidarScanMatcher, ScanMatcherConfig
from fabtino_core.localization.frames import lidar_polar_to_robot, relative_yaw
from fabtino_core.localization.map_localizer import KnownMapLocalizer
from fabtino_core.operations import imported_grid
from fabtino_core.sensors.wheel_encoder import WheelGeometry, WheelMeasurement, WheelOdometryEstimator
class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization')
        # Equivalent differential track for the current four-wheel average:
        # 2 * (Webots HALF_LENGTH + HALF_WIDTH) = 1.043834 m.
        self.declare_parameter('track_width_m',1.043834); self.declare_parameter('wheel_radius_m',0.100)
        self.declare_parameter('use_lidar_scan_matcher',True); self.declare_parameter('lidar_roll_rad',0.0)
        self.declare_parameter('lidar_angle_sign',-1.0)
        self.declare_parameter('use_imu_yaw_reference',True)
        self.declare_parameter('rotation_pending_wheel_rate_threshold',0.05)
        self.declare_parameter('rotation_confirm_gyro_rate_threshold',0.03)
        self.declare_parameter('rotation_active_gyro_rate_threshold',0.02)
        self.declare_parameter('rotation_in_place_linear_threshold',0.08)
        self.declare_parameter('rotation_settle_time_s',0.40)
        # The Fabtino footprint reaches about 0.44 m along its body X axis.
        # Close LiDAR returns bound wheel translation when wheels slip at a wall.
        self.declare_parameter('translation_collision_distance_m',0.50)
        self.declare_parameter('translation_collision_half_width_m',0.35)
        self.declare_parameter('collision_scan_timeout_s',0.30)
        tw=float(self.get_parameter('track_width_m').value); wr=float(self.get_parameter('wheel_radius_m').value)
        self.lidar_angle_sign=float(self.get_parameter('lidar_angle_sign').value)
        if self.lidar_angle_sign not in (-1.0, 1.0):
            raise ValueError('lidar_angle_sign must be -1.0 or 1.0')
        self.use_imu_yaw_reference=bool(self.get_parameter('use_imu_yaw_reference').value)
        self.rotation_pending_wheel_rate_threshold=float(self.get_parameter('rotation_pending_wheel_rate_threshold').value)
        self.rotation_confirm_gyro_rate_threshold=float(self.get_parameter('rotation_confirm_gyro_rate_threshold').value)
        self.rotation_active_gyro_rate_threshold=float(self.get_parameter('rotation_active_gyro_rate_threshold').value)
        self.rotation_in_place_linear_threshold=float(self.get_parameter('rotation_in_place_linear_threshold').value)
        self.rotation_settle_time_s=float(self.get_parameter('rotation_settle_time_s').value)
        self.startup_imu_yaw=None
        self.imu_yaw_offset=0.0
        self.minimum_sensor_stamp_ns=0
        self.operation=None
        self.operation_future=None
        self.operation_executor=None
        self.ekf=LocalizationEKF(EKFConfig(track_width_m=tw)); self.ekf.initialize(0.0,0,0,0,0)
        self.odom_est=WheelOdometryEstimator(WheelGeometry(wr,tw)); self.last_imu=None; self.last_scan=None; self.last_odom=None
        self.encoder_reference_initialized=False
        self.last_wheel_omega=0.0
        self.last_gyro_z=0.0
        self.rotation_pending=False
        self.rotation_active=False
        self.pose_stable=True
        self.settle_started=None
        self.matcher=LidarScanMatcher(ScanMatcherConfig(enabled=bool(self.get_parameter('use_lidar_scan_matcher').value)))
        # Wheel and IMU topics carry the same capture stamp per sensor packet.
        # DDS callback ordering must not decide which yaw belongs to a scan.
        self.pending_wheels={}
        self.pending_imus={}
        self.pending_scans={}
        self.last_fused_ns=None
        self.last_fused_stamp=None
        self.last_matched_ns=-1
        self.pub=self.create_publisher(Odometry,'/fabtino/odometry/filtered',20)
        self.pub_operation=self.create_publisher(String,'/viewer/operation_status',10)
        self.pub_pose_stable=self.create_publisher(Bool,'/fabtino/pose_stable',10)
        self.tf=TransformBroadcaster(self)
        self.create_subscription(JointState,'/fabtino/joint_states',self.wheel_cb,20); self.create_subscription(Imu,'/fabtino/imu/data_raw',self.imu_cb,20); self.create_subscription(LaserScan,'/fabtino/scan',self.scan_cb,10)
        self.create_timer(0.02,self.publish)
        self.create_subscription(String,'/viewer/request',self.request_cb,10)
        self.create_timer(0.05,self.poll_operations)
        self.publish_pose_stable(True)

    def operation_status(self,state,reason='',**details):
        if self.operation is None:return
        msg=String()
        msg.data=json.dumps(dict(node='localization',request_id=self.operation['request_id'],
                                type=self.operation['type'],state=state,reason=reason,**details))
        self.pub_operation.publish(msg)

    def request_cb(self,msg):
        request=json.loads(msg.data)
        kind=request.get('type')
        if kind=='cancel_operation':
            if self.operation is not None and self.operation['request_id']==request.get('request_id'):
                if self.operation_future is not None:self.operation_future.cancel()
                self.operation_future=None
                self.operation=None
            return
        if kind not in ('reset_odom','nav_map','nav_clear_map'):return
        self.operation=request
        if self.operation_future is not None:self.operation_future.cancel()
        self.operation_future=None
        if kind=='reset_odom':
            self.minimum_sensor_stamp_ns=int(request['stamp_ns'])
            self.ekf=LocalizationEKF(self.ekf.config)
            self.ekf.initialize(self.minimum_sensor_stamp_ns*1e-9,0.,0.,0.)
            self.odom_est.reset()
            self.encoder_reference_initialized=False
            self.startup_imu_yaw=None
            self.imu_yaw_offset=0.
            self.last_imu=self.last_scan=self.last_odom=None
            self.last_fused_ns=self.last_fused_stamp=None
            self.last_matched_ns=-1
            self.last_wheel_omega=self.last_gyro_z=0.
            self.rotation_pending=self.rotation_active=False
            self.settle_started=None
            self.pending_wheels.clear();self.pending_imus.clear();self.pending_scans.clear()
            self.matcher.reset_reference()
            self.publish_pose_stable(True)
            self.operation_status('pending','Waiting for a fresh wheel/IMU baseline')
        elif kind=='nav_clear_map':
            self.matcher.reset_reference()
            self.operation_status('complete')
            self.operation=None
        else:
            self.operation_status('pending','Waiting for a fresh stationary LiDAR scan')

    @staticmethod
    def localize_import(request,points,timestamp):
        grid=imported_grid(request)
        matcher=KnownMapLocalizer(grid.data,grid.resolution,grid.radius)
        match=matcher.match(points,(0.,0.,0.),timestamp)
        if match is None:
            raise ValueError('Map could not be localized: '+matcher.last_status)
        return match

    def poll_operations(self):
        if self.operation is None or self.operation.get('type')!='nav_map':return
        now=self.get_clock().now().nanoseconds
        if (now-int(self.operation['stamp_ns']))*1e-9>30.:
            self.operation_status('error','Map localization timed out; check the image and scale, then retry')
            self.operation=None
            return
        if self.operation_future is None:return
        if not self.operation_future.done():return
        try:
            match=self.operation_future.result()
            if self.last_odom is not None and (abs(self.last_odom.linear_velocity_mps)>.02 or abs(self.last_gyro_z)>.03):
                raise ValueError('Robot moved during map localization; stop it and retry')
            self.ekf=LocalizationEKF(self.ekf.config)
            timestamp=(self.last_fused_ns or now)*1e-9
            self.ekf.initialize(timestamp,match.x_m,match.y_m,match.yaw_rad)
            if self.last_imu is not None:
                q=self.last_imu.orientation
                raw=euler_from_quaternion([q.x,q.y,q.z,q.w])[2]
                base=relative_yaw(raw,self.startup_imu_yaw) if self.use_imu_yaw_reference else raw
                self.imu_yaw_offset=match.yaw_rad-base
            self.matcher.reset_reference()
            self.publish()
            self.operation_status('complete',known_map_global_localized=True,
                                  known_map_request_id=self.operation['request_id'])
        except Exception as exc:
            self.operation_status('error',str(exc))
        self.operation_future=None
        self.operation=None

    def publish_pose_stable(self, stable):
        stable=bool(stable)
        if stable == self.pose_stable and hasattr(self, '_pose_stable_published'):
            return
        self.pose_stable=stable
        self._pose_stable_published=True
        msg=Bool(); msg.data=stable; self.pub_pose_stable.publish(msg)

    def update_rotation_gate(self):
        gyro_active=abs(self.last_gyro_z) >= self.rotation_active_gyro_rate_threshold
        wheel_active=(
            abs(self.last_wheel_omega) >= self.rotation_pending_wheel_rate_threshold
            and self.last_odom is not None
            and abs(self.last_odom.linear_velocity_mps) <= self.rotation_in_place_linear_threshold
        )
        if self.rotation_pending and gyro_active:
            self.rotation_pending=False
            self.get_logger().info('Rotation confirmed by IMU; continuing motion fusion')
        unstable=self.rotation_pending or gyro_active or wheel_active
        if unstable:
            self.settle_started=None
            self.rotation_active=True
            self.publish_pose_stable(False)
        elif self.rotation_active:
            if self.settle_started is None:
                self.settle_started=self.get_clock().now().nanoseconds*1e-9
            settled_for=self.get_clock().now().nanoseconds*1e-9-self.settle_started
            if settled_for >= self.rotation_settle_time_s:
                self.rotation_active=False
                self.publish_pose_stable(True)
                self.get_logger().info('Rotation settled; pose stability restored')
    @staticmethod
    def stamp_ns(msg):
        return msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec

    def queue_sensor(self, pending, msg):
        stamp=self.stamp_ns(msg)
        if stamp<self.minimum_sensor_stamp_ns:return
        if self.last_fused_ns is not None and stamp <= self.last_fused_ns:
            return
        pending[stamp]=msg
        while len(pending)>128:
            del pending[min(pending)]
        self.fuse_ready_samples()

    def wheel_cb(self,msg):
        self.queue_sensor(self.pending_wheels,msg)

    def imu_cb(self,msg):
        self.queue_sensor(self.pending_imus,msg)

    def fuse_ready_samples(self):
        for stamp in sorted(self.pending_wheels.keys() & self.pending_imus.keys()):
            wheels=self.pending_wheels.pop(stamp)
            imu=self.pending_imus.pop(stamp)
            if len(wheels.position)<4:
                continue
            self.last_gyro_z=float(imu.angular_velocity.z)
            self.process_wheels(wheels)
            self.process_imu(imu)
            self.last_fused_ns=stamp
            self.last_fused_stamp=wheels.header.stamp
            scan=self.pending_scans.pop(stamp,None)
            if scan is not None:
                self.process_scan(scan)
            self.publish()
            if self.operation is not None and self.operation.get('type')=='reset_odom':
                self.operation_status('complete')
                self.operation=None
        if self.last_fused_ns is not None:
            for pending in (self.pending_wheels,self.pending_imus,self.pending_scans):
                for stamp in list(pending):
                    if stamp <= self.last_fused_ns:
                        del pending[stamp]

    def process_wheels(self,msg):
        if len(msg.position)<4:return
        t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9; left=(msg.position[0]+msg.position[2])*0.5; right=(msg.position[1]+msg.position[3])*0.5
        measurement=WheelMeasurement(t,left,right)
        # The Webots encoders contain absolute joint angles.  The original
        # controller establishes this baseline before integrating motion;
        # otherwise startup wheel angles become a false displacement.
        if not self.encoder_reference_initialized:
            self.odom_est.reset(measurement)
            self.ekf.timestamp=t
            self.encoder_reference_initialized=True
            return
        od=self.odom_est.update(measurement)
        if od:
            od=self.constrain_wheel_translation(od,self.stamp_ns(msg))
            self.last_wheel_omega=float(od.angular_velocity_rps)
            self.last_odom=od
            wheel_turning=(
                abs(self.last_wheel_omega) >= self.rotation_pending_wheel_rate_threshold
                and abs(od.linear_velocity_mps) <= self.rotation_in_place_linear_threshold
            )
            gyro_confirmed=abs(self.last_gyro_z) >= self.rotation_confirm_gyro_rate_threshold
            if wheel_turning and not gyro_confirmed and not self.rotation_pending:
                self.rotation_pending=True
                self.rotation_active=True
                self.settle_started=None
                self.publish_pose_stable(False)
                self.get_logger().warning(
                    'Wheel rotation detected before IMU body rotation; monitoring pose fusion'
                )
            # Rotation state is diagnostic only.  Wheel odometry must continue
            # updating the pose while scans are being acquired; otherwise the
            # mapper projects rotating LiDAR returns with a stale yaw.
            self.ekf.predict_wheel_odometry(od)
            self.ekf.update_wheel_odometry(od)
            self.update_rotation_gate()

    def constrain_wheel_translation(self,odom,stamp):
        """Do not count spinning wheels as travel into a nearby observed wall.

        Use the same capture's scan when available, otherwise a recent older
        scan. The encoder estimator has already consumed this increment, so
        moving away from the wall cannot replay discarded wheel rotation.
        """
        if abs(odom.distance_m)<1e-12:
            return odom
        scan=self.pending_scans.get(stamp,self.last_scan)
        if scan is None:
            return odom
        age=(stamp-self.stamp_ns(scan))*1e-9
        if age<0 or age>float(self.get_parameter('collision_scan_timeout_s').value):
            return odom
        clearance=float(self.get_parameter('translation_collision_distance_m').value)
        half_width=float(self.get_parameter('translation_collision_half_width_m').value)
        direction=1.0 if odom.distance_m>0 else -1.0
        for index,distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or distance<=0 or distance<scan.range_min or distance>scan.range_max:
                continue
            angle=scan.angle_min+index*scan.angle_increment
            x,y=lidar_polar_to_robot(float(distance),float(angle),self.lidar_angle_sign)
            if 0<direction*x<clearance and abs(y)<half_width:
                return replace(odom,distance_m=0.0,linear_velocity_mps=0.0)
        return odom

    def process_imu(self,msg):
        self.last_imu=msg
        self.last_gyro_z=float(msg.angular_velocity.z)
        self.ekf.update_gyro(self.last_gyro_z)
        q=msg.orientation; _,_,yaw=euler_from_quaternion([q.x,q.y,q.z,q.w])
        if self.use_imu_yaw_reference and self.startup_imu_yaw is None:
            self.startup_imu_yaw=float(yaw)
            self.get_logger().info(f'IMU startup yaw reference: {self.startup_imu_yaw:.6f} rad')
        if self.use_imu_yaw_reference:
            yaw=relative_yaw(float(yaw), self.startup_imu_yaw)
        yaw+=self.imu_yaw_offset
        # IMU orientation is used only as a local-map yaw observation, not GT.
        # Keep the yaw observation live during rotation.  Suppressing it while
        # accepting scans makes the mapper use a stale pose and rotates a
        # static wall in the opposite direction until the robot settles.
        self.ekf.update_yaw(float(yaw),0.02)
        self.update_rotation_gate()
    def scan_cb(self,msg):
        stamp=self.stamp_ns(msg)
        if stamp<self.minimum_sensor_stamp_ns:return
        if self.last_fused_ns is not None and stamp < self.last_fused_ns:
            # A delayed scan may still map with historical odometry, but must
            # not correct the present EKF state using an old observation.
            return
        if stamp == self.last_fused_ns:
            self.process_scan(msg)
            self.publish()
        else:
            self.pending_scans[stamp]=msg
            while len(self.pending_scans)>128:
                del self.pending_scans[min(self.pending_scans)]

    def process_scan(self,msg):
        stamp=self.stamp_ns(msg)
        if stamp <= self.last_matched_ns:
            return
        self.last_matched_ns=stamp
        self.last_scan=msg
        pts=[]
        for idx,d in enumerate(msg.ranges):
            if not math.isfinite(d) or d<msg.range_min or d>msg.range_max: continue
            a=msg.angle_min+idx*msg.angle_increment
            pts.append(list(lidar_polar_to_robot(float(d), float(a), self.lidar_angle_sign)))
        if (self.operation is not None and self.operation.get('type')=='nav_map'
                and self.operation_future is None and len(pts)>=20
                and stamp>=int(self.operation['stamp_ns'])
                and abs(self.last_gyro_z)<.03
                and (self.last_odom is None or abs(self.last_odom.linear_velocity_mps)<.02)):
            if self.operation_executor is None:self.operation_executor=ThreadPoolExecutor(max_workers=1)
            self.operation_future=self.operation_executor.submit(self.localize_import,
                dict(self.operation),pts,stamp*1e-9)
        if len(pts)<40:return
        pose=self.ekf.pose
        scan_timestamp=float(msg.header.stamp.sec)+float(msg.header.stamp.nanosec)*1e-9
        match=self.matcher.match(pts,pose,scan_timestamp,
            linear_velocity_mps=float(self.ekf.x[3]),angular_velocity_rps=float(self.ekf.x[4]),gyro_z_rps=float(self.last_imu.angular_velocity.z) if self.last_imu else 0.0)
        if match is not None:self.ekf.update_pose(match)
    def publish(self):
        # Repeat the state so late subscribers (including the mapper after
        # startup) always learn whether scans are currently safe to integrate.
        stable_msg=Bool(); stable_msg.data=bool(self.pose_stable); self.pub_pose_stable.publish(stable_msg)
        if self.last_fused_stamp is None:
            return
        # Repeating a pose must retain its measurement stamp. Timer time would
        # fabricate fresh poses and hide both transport delay and sensor loss.
        now=self.last_fused_stamp; x,y,yaw=self.ekf.pose
        m=Odometry(); m.header.stamp=now; m.header.frame_id='map'; m.child_frame_id='base_link'; m.pose.pose.position.x=x; m.pose.pose.position.y=y; q=quaternion_from_euler(0,0,yaw); m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w=map(float,q); m.twist.twist.linear.x=float(self.ekf.x[3]); m.twist.twist.angular.z=float(self.ekf.x[4]);
        for i in range(6):m.pose.covariance[i*6+i]=float(self.ekf.P[min(i,2),min(i,2)])
        self.pub.publish(m)
        # TF rejects repeated timestamps; the odometry heartbeat above still
        # carries the original acquisition time for the safety watchdog.
        if getattr(self,'last_tf_ns',None) == self.last_fused_ns:
            return
        self.last_tf_ns=self.last_fused_ns
        tf=TransformStamped(); tf.header.stamp=now; tf.header.frame_id='map'; tf.child_frame_id='base_link'; tf.transform.translation.x=x; tf.transform.translation.y=y; tf.transform.translation.z=0.0; tf.transform.rotation.x=0.0; tf.transform.rotation.y=0.0; tf.transform.rotation.z=float(q[2]); tf.transform.rotation.w=float(q[3]); self.tf.sendTransform(tf)
def main():
    rclpy.init();n=LocalizationNode()
    try:rclpy.spin(n)
    finally:
        if n.operation_executor is not None:n.operation_executor.shutdown(wait=False,cancel_futures=True)
        n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
