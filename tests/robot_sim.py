"""Deterministic device substitutes around the production Webots controller.

Wheel motion is ideal kinematics, and LiDAR raycasts an asymmetric 2-D room.
This is deliberately separate from actual Webots physics and DDS validation.
"""
import importlib.util
import math
from pathlib import Path
import sys
import time
from types import ModuleType,SimpleNamespace as NS
from unittest.mock import patch

ROOT=Path(__file__).parents[1]
FOLDER=ROOT/'webots/controllers/fabtino_webots_bridge_controller'


def controller_module():
    sys.path.insert(0,str(FOLDER))
    stub=ModuleType('controller');stub.Supervisor=NS
    spec=importlib.util.spec_from_file_location('_pipeline_controller',FOLDER/'fabtino_webots_bridge_controller.py')
    module=importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules,{'controller':stub}):spec.loader.exec_module(module)
    return module


SEGMENTS=[(-3.,-2.,4.,-2.),(4.,-2.,4.,3.),(4.,3.,-3.,3.),(-3.,3.,-3.,-2.),
          (1.5,.8,1.5,2.7),(-2.,1.2,-.8,1.2)]


def known_map():
    import numpy as np
    resolution=.1;radius=6.;data=np.zeros((120,120),dtype=np.int8)
    for ax,ay,bx,by in SEGMENTS:
        for factor in np.linspace(0.,1.,max(2,int(math.hypot(bx-ax,by-ay)/.02))):
            x,y=ax+(bx-ax)*factor,ay+(by-ay)*factor
            data[int((y+radius)/resolution),int((x+radius)/resolution)]=100
    return dict(resolution=resolution,radius=radius,grid=data.ravel().tolist())


def ray_distance(x,y,angle):
    dx,dy=math.cos(angle),math.sin(angle)
    best=12.
    for ax,ay,bx,by in SEGMENTS:
        sx,sy=bx-ax,by-ay
        denom=dx*sy-dy*sx
        if abs(denom)<1e-9:continue
        t=((ax-x)*sy-(ay-y)*sx)/denom
        u=((ax-x)*dy-(ay-y)*dx)/denom
        if t>0 and 0<=u<=1:best=min(best,t)
    return best


class RobotDevices:
    def __init__(self,module,transport):
        from lidar_mask import AngularMask
        self.x=self.y=self.yaw=0.
        self.positions={key:0. for key in ('fl','fr','rl','rr')}
        self.velocities={key:0. for key in self.positions}
        self.v=self.w=self.sim_time=0.
        self.rear_wall=False
        self.dynamic_obstacle=None
        self.controller=object.__new__(module.Controller)
        node=self.controller
        node.transport=transport
        node.motors={key:NS(setVelocity=lambda velocity,key=key:self.velocities.__setitem__(key,velocity)) for key in self.positions}
        node.encoders={key:NS(getValue=lambda key=key:self.positions[key]) for key in self.positions}
        node.robot=NS(getTime=lambda:self.sim_time)
        node.imu=NS(getRollPitchYaw=lambda:(0.,0.,self.yaw))
        node.gyro=NS(getValues=lambda:(0.,0.,self.w))
        node.accel=NS(getValues=lambda:(0.,0.,0.))
        node.lidar=NS(getRangeImage=self.scan,getMinRange=lambda:.2,getMaxRange=lambda:12.,getFov=lambda:2*math.pi)
        node.lidar_mask=AngularMask(2*math.pi,121,1,[])
        node.gt=lambda:[self.x,self.y,0.,self.yaw]
        node.last_cmd_mono=time.monotonic();node.last_cmd=(0.,0.)
        node.scanning=True;node.seq=0;node.reset_id=None
        self.module=module

    def scan(self):
        values=[]
        for i in range(121):
            angle=math.pi-i*2*math.pi/120
            distance=ray_distance(self.x,self.y,angle+self.yaw)
            if self.dynamic_obstacle is not None:
                x,y,radius=self.dynamic_obstacle
                dx,dy=x-self.x,y-self.y
                projection=dx*math.cos(angle+self.yaw)+dy*math.sin(angle+self.yaw)
                discriminant=radius*radius-(dx*dx+dy*dy-projection*projection)
                if discriminant>=0 and projection>0:distance=min(distance,projection-math.sqrt(discriminant))
            if self.rear_wall and abs(abs(angle)-math.pi)<.25:distance=.45
            values.append(distance)
        return values

    def step(self,dt):
        node=self.controller
        node.apply_commands();node.drive(*node.last_cmd)
        left=(self.velocities['fl']+self.velocities['rl'])*.05
        right=(self.velocities['fr']+self.velocities['rr'])*.05
        self.v=(left+right)*.5;self.w=(right-left)/(2*self.module.K)
        self.x+=self.v*math.cos(self.yaw+self.w*dt*.5)*dt
        self.y+=self.v*math.sin(self.yaw+self.w*dt*.5)*dt
        self.yaw=(self.yaw+self.w*dt+math.pi)%(2*math.pi)-math.pi
        for key in self.positions:self.positions[key]+=self.velocities[key]*dt
        self.sim_time+=dt
        node.send_sensor()
