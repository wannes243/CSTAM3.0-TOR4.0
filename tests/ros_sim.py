"""ROS message/topic test doubles. This does not simulate DDS or Webots physics."""
from collections import defaultdict,deque
from copy import deepcopy
import importlib
import sys
import time
from types import ModuleType,SimpleNamespace as NS

ACTIVE_BUS=None


class Bus:
    def __init__(self,overrides=None,realtime=False):
        global ACTIVE_BUS
        ACTIVE_BUS=self
        self.overrides=overrides or {}
        self.realtime=realtime
        self.now_ns=time.time_ns() if realtime else 1_700_000_000_000_000_000
        self.subscribers=defaultdict(list)
        self.pending=deque()
        self.timers=[]
        self.nodes=[]
        self.errors=[]
    def clock_ns(self):return time.time_ns() if self.realtime else self.now_ns
    def flush(self):
        count=0
        while self.pending:
            callback,message=self.pending.popleft()
            callback(message)
            count+=1
            if count>20000:raise RuntimeError('Topic callback cycle')
    def tick(self,dt=.02):
        if not self.realtime:self.now_ns+=round(dt*1e9)
        self.flush()
        now=self.clock_ns()
        for timer in list(self.timers):
            if now-timer.last_ns>=timer.period_ns:
                timer.last_ns=now
                timer.callback()
                self.flush()


class Publisher:
    def __init__(self,bus,topic):self.bus,self.topic,self.messages=bus,topic,[]
    def publish(self,message):
        copy=deepcopy(message)
        self.messages.append(copy)
        if len(self.messages)>4096:self.messages.pop(0)
        for callback in self.bus.subscribers[self.topic]:self.bus.pending.append((callback,deepcopy(copy)))


class Node:
    def __init__(self,name):
        self.bus=ACTIVE_BUS
        if self.bus is None:self.bus=Bus()
        self.name=name
        self.parameters={}
        self.bus.nodes.append(self)
    def declare_parameter(self,name,value):self.parameters[name]=self.bus.overrides.get(self.name,{}).get(name,value)
    def get_parameter(self,name):return NS(value=self.parameters[name])
    def create_publisher(self,kind,topic,*args):return Publisher(self.bus,topic)
    def create_subscription(self,kind,topic,callback,*args):self.bus.subscribers[topic].append(callback)
    def create_timer(self,period,callback):
        timer=NS(period_ns=round(period*1e9),callback=callback,last_ns=self.bus.clock_ns())
        self.bus.timers.append(timer)
        return timer
    def create_service(self,*args):return NS()
    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=self.bus.clock_ns(),to_msg=lambda: stamp(self.bus.clock_ns())))
    def get_logger(self):
        return NS(info=lambda message:None,warning=lambda message:None,debug=lambda message:None,
                  error=lambda message:self.bus.errors.append(message))
    def destroy_node(self):pass


def stamp(ns=0):return NS(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000)
def vector():return NS(x=0.,y=0.,z=0.)
def quaternion():return NS(x=0.,y=0.,z=0.,w=1.)
def header():return NS(stamp=stamp(),frame_id='')
def pose():return NS(position=vector(),orientation=quaternion())
def twist():return NS(linear=vector(),angular=vector())


class String(NS):
    def __init__(self,data=''):super().__init__(data=data)
class Bool(NS):
    def __init__(self,data=False):super().__init__(data=data)
class Empty(NS):pass
class Odometry(NS):
    def __init__(self):super().__init__(header=header(),child_frame_id='',pose=NS(pose=pose(),covariance=[0.]*36),twist=NS(twist=twist(),covariance=[0.]*36))
class OccupancyGrid(NS):
    def __init__(self):super().__init__(header=header(),info=NS(resolution=0.,width=0,height=0,origin=pose(),map_load_time=stamp()),data=[])
class Imu(NS):
    def __init__(self):super().__init__(header=header(),orientation=quaternion(),angular_velocity=vector(),linear_acceleration=vector())
class JointState(NS):
    def __init__(self):super().__init__(header=header(),position=[],velocity=[],name=[])
class LaserScan(NS):
    def __init__(self):super().__init__(header=header(),ranges=[],angle_min=0.,angle_max=0.,angle_increment=0.,range_min=.2,range_max=12.)
class PointCloud2(NS):
    def __init__(self):super().__init__(header=header(),fields=[],width=0,height=1,data=b'',is_bigendian=False,point_step=12,row_step=0)
class PointField(NS):FLOAT32=7;FLOAT64=8
class Twist(NS):
    def __init__(self):super().__init__(linear=vector(),angular=vector())
class TwistStamped(NS):
    def __init__(self):super().__init__(header=header(),twist=twist())
class TransformStamped(NS):
    def __init__(self):super().__init__(header=header(),child_frame_id='',transform=NS(translation=vector(),rotation=quaternion()))
class PoseStamped(NS):
    def __init__(self):super().__init__(header=header(),pose=pose())
class Path(NS):
    def __init__(self):super().__init__(header=header(),poses=[])
class Trigger:
    Request=NS
    Response=NS


def install():
    definitions={
        'rclpy':dict(__fake__=True,ok=lambda:True,init=lambda:None,shutdown=lambda:None),
        'rclpy.node':dict(Node=Node),
        'std_msgs.msg':dict(String=String,Bool=Bool,Empty=Empty),
        'nav_msgs.msg':dict(Odometry=Odometry,OccupancyGrid=OccupancyGrid,Path=Path),
        'sensor_msgs.msg':dict(Imu=Imu,JointState=JointState,LaserScan=LaserScan,PointCloud2=PointCloud2,PointField=PointField),
        'geometry_msgs.msg':dict(Twist=Twist,TwistStamped=TwistStamped,TransformStamped=TransformStamped,PoseStamped=PoseStamped),
        'std_srvs.srv':dict(Trigger=Trigger,SetBool=Trigger),
        'tf2_ros':dict(TransformBroadcaster=lambda node:NS(sendTransform=lambda message:None)),
    }
    for name,values in definitions.items():
        module=ModuleType(name)
        module.__dict__.update(values)
        sys.modules[name]=module
        if '.' in name:
            parent,child=name.rsplit('.',1)
            if parent not in sys.modules:sys.modules[parent]=ModuleType(parent)
            setattr(sys.modules[parent],child,module)


def adapters():
    packages={
        'bridge':('fabtino_webots_bridge.webots_bridge_node','WebotsBridgeNode'),
        'localization':('fabtino_localization.localization_node','LocalizationNode'),
        'mapping':('fabtino_mapping.mapping_node','MappingNode'),
        'mission':('fabtino_navigation.mission_manager_node','MissionManagerNode'),
        'global_planner':('fabtino_navigation.global_planner_node','GlobalPlannerNode'),
        'local_planner':('fabtino_navigation.local_planner_node','LocalPlannerNode'),
        'safety':('fabtino_safety.safety_supervisor_node','SafetySupervisor'),
        'viewer':('fabtino_viewer.viewer_gateway','ViewerGateway'),
    }
    return {key:getattr(importlib.import_module(module),name) for key,(module,name) in packages.items()}
