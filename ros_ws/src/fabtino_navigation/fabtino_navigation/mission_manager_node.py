from __future__ import annotations
import json,math
from pathlib import Path
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from fabtino_navigation.goal_control import Goal
from fabtino_core.navigation.planner import OccupancyGrid as CoreGrid
from fabtino_core.navigation.zones import validate_config,point_denied,zone_mask,approach_target
from fabtino_core.navigation.delivery import Delivery


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.explore_mode = False
        self.goal = None
        self.map = None
        self.pose = None
        self.visited_frontiers = set()
        self.config=validate_config({});self.config_revision=0;self.config_result={}
        self.delivery=Delivery();self.pending_import_config=None;self.pose_yaw=0.
        self.last_navigation_map_ns=None
        self.declare_parameter('navigation_clearance_m',.60)
        self.declare_parameter('state_file','')
        self.state_file=str(self.get_parameter('state_file').value)
        if self.state_file and Path(self.state_file).is_file():
            try:self.config=validate_config(json.loads(Path(self.state_file).read_text(encoding='utf-8')))
            except (OSError,ValueError,KeyError,TypeError) as exc:self.get_logger().warning(f'Could not load navigation configuration: {exc}')
        self.pub = self.create_publisher(String, '/navigation/goal', 10)
        self.status = self.create_publisher(String, '/navigation/mission_status', 10)
        self.config_pub=self.create_publisher(String,'/navigation/config',10)
        self.operation_pub=self.create_publisher(String,'/viewer/operation_status',10)
        self.create_subscription(String, '/navigation/request', self.req_cb, 10)
        self.create_subscription(String, '/navigation/local_status', self.local_cb, 10)
        self.create_subscription(String, '/navigation/global_status', self.global_cb, 10)
        self.create_subscription(Odometry, '/fabtino/odometry/filtered', self.pose_cb, 10)
        self.create_subscription(OccupancyGrid, '/fabtino/map', self.map_cb, 3)
        self.create_subscription(OccupancyGrid,'/fabtino/navigation_map',lambda msg:self.map_cb(msg,True),3)
        self.create_subscription(TwistStamped,'/teleop/cmd_vel',self.teleop_cb,10)
        self.create_subscription(String,'/viewer/request',self.map_request_cb,10)
        self.create_subscription(String,'/viewer/operation_status',self.map_operation_cb,10)
        self.create_timer(.1,self.delivery_tick)
        self.create_timer(.5,self.publish_config)

    def req_cb(self, msg):
        request={}
        try:
            request = json.loads(msg.data)
            if not isinstance(request,dict):raise ValueError('Navigation request must be an object')
            kind = request.get('type')
            if kind=='configure':
                self.set_config(request,request.get('request_id'));return
            if kind=='delivery_start':
                self.start_delivery(request);return
            if kind=='delivery_return':
                self.return_base();return
            if kind == 'goal':
                goal = Goal.from_request(request)
                if point_denied(goal.x,goal.y,self.config['zones'],float(self.get_parameter('navigation_clearance_m').value)):
                    raise ValueError('Target is inside a denied zone or its robot clearance')
                self.delivery.cancel('Replaced by a new navigation goal')
                self.explore_mode = False
                self.goal = goal
                self.publish_goal()
            elif kind == 'explore':
                self.cancel()
                self.visited_frontiers.clear()
                self.explore_mode = True
                self.choose_frontier()
            elif kind in ('stop', 'nav_stop'):
                self.cancel()
            else:
                raise ValueError('Unknown navigation request type')
        except (ValueError, KeyError, TypeError,OSError) as exc:
            if isinstance(request,dict) and request.get('type')=='configure':
                self.config_result=dict(request_id=request.get('request_id'),state='error',reason=str(exc));self.publish_config()
            self.log('invalid_request', str(exc))

    def cancel(self):
        self.delivery.cancel()
        self.goal = None
        self.explore_mode = False
        msg = String()
        msg.data = json.dumps({'type': 'stop'})
        self.pub.publish(msg)
        self.log('stopped')

    def pose_cb(self, msg):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q=msg.pose.pose.orientation
        self.pose_yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if self.explore_mode and self.goal is None:
            self.choose_frontier()

    def map_cb(self, msg, navigation=False):
        now=self.get_clock().now().nanoseconds
        if navigation:self.last_navigation_map_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        elif self.last_navigation_map_ns is not None and now-self.last_navigation_map_ns<600_000_000:return
        self.map = msg
        if self.explore_mode and self.goal is None:
            self.choose_frontier()

    def publish_goal(self):
        if self.goal is not None:
            msg = String()
            msg.data = json.dumps(dict(self.goal.payload(),navigation_config=self.config,config_revision=self.config_revision))
            self.pub.publish(msg)
            self.log('goal_active')

    def choose_frontier(self):
        if self.map is None or self.pose is None:
            self.log('waiting_for_map_or_pose')
            return
        width, height = self.map.info.width, self.map.info.height
        data = self.map.data
        resolution = self.map.info.resolution
        ox, oy = self.map.info.origin.position.x, self.map.info.origin.position.y
        blocked=None
        if self.config['zones']:
            navigation_grid=CoreGrid(resolution,-ox);navigation_grid.load(resolution,-ox,data)
            blocked=navigation_grid.inflated_obstacles(float(self.get_parameter('navigation_clearance_m').value))|zone_mask(navigation_grid,self.config['zones'],float(self.get_parameter('navigation_clearance_m').value))
        best = None
        for gy in range(1, height-1):
            for gx in range(1, width-1):
                x, y = ox+(gx+.5)*resolution, oy+(gy+.5)*resolution
                key = (round(x, 6), round(y, 6))
                if data[gy*width+gx] != 0 or key in self.visited_frontiers or (blocked is not None and blocked[gy,gx]):
                    continue
                neighbours = [(gx+1, gy), (gx-1, gy), (gx, gy+1), (gx, gy-1)]
                if any(data[ny*width+nx] < 0 for nx, ny in neighbours):
                    distance = (x-self.pose[0])**2+(y-self.pose[1])**2
                    if best is None or distance < best[0]:
                        best = (distance, x, y)
        if best is None:
            self.explore_mode = False
            self.log('exploration_complete')
            return
        _, x, y = best
        self.goal = Goal.from_request({'x': x, 'y': y})
        self.publish_goal()

    def local_cb(self, msg):
        payload = json.loads(msg.data)
        if self.goal is None or payload.get('goal_id') != self.goal.goal_id:
            return
        state = payload.get('state')
        if self.delivery.active and state in ('goal_reached','goal_failed'):
            if state=='goal_failed':self.delivery.fail(payload.get('reason','Delivery navigation failed'));self.stop_leg()
            else:
                self.delivery.reached(self.seconds(),payload['goal_id']);self.stop_leg()
            self.log('delivery_'+self.delivery.state,self.delivery.reason)
            return
        if state == 'goal_failed':
            self.explore_mode = False
            self.log(state, payload.get('reason', ''))
            self.goal = None
            return
        if state == 'goal_reached':
            self.log(state)
            self.visited_frontiers.add((round(self.goal.x, 6), round(self.goal.y, 6)))
            self.goal = None
            if self.explore_mode:
                self.choose_frontier()
        else:
            self.log(state)

    def log(self, state, reason=''):
        msg = String()
        msg.data = json.dumps({'state': state, 'reason': reason, 'explore': self.explore_mode,
                               'goal': [self.goal.x, self.goal.y] if self.goal else None,
                               'goal_id': self.goal.goal_id if self.goal else None,
                               'yaw_deg': self.goal.payload()['yaw_deg'] if self.goal else None,
                               'delivery':self.delivery.snapshot(self.seconds())})
        self.status.publish(msg)

    def global_cb(self, msg):
        payload = json.loads(msg.data)
        if self.goal is None or payload.get('goal_id') != self.goal.goal_id:
            return
        # path_ready repeats during replanning; retain the local execution state.
        if self.delivery.active and payload.get('state') in ('pathfinding_failed','goal_outside_map','goal_in_denied_zone','robot_in_denied_zone'):
            self.delivery.fail(payload.get('reason') or payload['state']);self.stop_leg();self.log('delivery_failed',self.delivery.reason);return
        if payload.get('state') not in ('path_ready', 'goal_reached'):
            self.log(payload.get('state'), payload.get('reason', ''))

    def seconds(self):return self.get_clock().now().nanoseconds*1e-9

    def publish_config(self):
        msg=String();msg.data=json.dumps(dict(self.config,revision=self.config_revision,**self.config_result))
        self.config_pub.publish(msg)

    def set_config(self,payload,request_id=None):
        config=validate_config(payload)
        if self.state_file:
            file=Path(self.state_file);file.parent.mkdir(parents=True,exist_ok=True)
            temporary=file.with_name(file.name+'.tmp')
            temporary.write_text(json.dumps(config,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8');temporary.replace(file)
        self.cancel();self.config=config;self.config_revision+=1
        self.config_result=dict(request_id=request_id,state='complete',reason='');self.publish_config()

    def map_request_cb(self,msg):
        request=json.loads(msg.data)
        if request.get('type')=='reset_odom':self.commit_map_config({},request['request_id'])
        elif request.get('type')=='nav_map':
            self.pending_import_config=(request['request_id'],validate_config(request.get('navigation_config',{})))
        elif request.get('type')=='cancel_operation' and self.pending_import_config is not None and request.get('request_id')==self.pending_import_config[0]:
            self.pending_import_config=None

    def map_operation_cb(self,msg):
        status=json.loads(msg.data)
        if self.pending_import_config is None or status.get('request_id')!=self.pending_import_config[0]:return
        if status.get('state')=='error':self.pending_import_config=None
        elif status.get('node')=='mapping' and status.get('state')=='complete':
            config=self.pending_import_config[1];self.pending_import_config=None;self.commit_map_config(config,status['request_id'])

    def commit_map_config(self,config,request_id):
        result=dict(node='mission',request_id=request_id,state='complete',reason='',stamp_ns=self.get_clock().now().nanoseconds)
        try:self.set_config(config,request_id)
        except (OSError,ValueError,KeyError,TypeError) as exc:
            # The map frame has already changed. Old-frame exclusions cannot survive.
            self.cancel();self.config=validate_config({});self.config_revision+=1
            result.update(state='error',reason='Map annotations could not be saved: '+str(exc))
            self.config_result=dict(request_id=request_id,state='error',reason=result['reason']);self.publish_config()
        message=String();message.data=json.dumps(result);self.operation_pub.publish(message)

    def teleop_cb(self,msg):
        if self.delivery.active and (abs(msg.twist.linear.x)>1e-6 or abs(msg.twist.angular.z)>1e-6):
            self.cancel();self.delivery.reason='Cancelled by manual driving';self.log('delivery_cancelled',self.delivery.reason)

    def stop_leg(self):
        self.goal=None;self.explore_mode=False
        msg=String();msg.data=json.dumps(dict(type='stop'));self.pub.publish(msg)

    def check_base(self):
        base=self.config['base']
        if base is None:raise ValueError('Select a base position before starting delivery')
        if point_denied(base['x'],base['y'],self.config['zones'],float(self.get_parameter('navigation_clearance_m').value)):
            raise ValueError('Base is inside a denied zone or its robot clearance')
        if self.map is None or self.pose is None:raise ValueError('Wait for the navigation map and robot position')
        return base

    def start_delivery(self,request):
        self.check_base()
        delivery=Delivery();delivery.start(request.get('tasks'),self.config['zones'],request.get('return_to_base',True))
        self.cancel();self.delivery=delivery;self.start_leg()

    def start_leg(self):
        try:
            if self.delivery.state=='returning':target=self.check_base()
            else:
                grid=CoreGrid(self.map.info.resolution,-self.map.info.origin.position.x)
                grid.load(grid.resolution,grid.radius,self.map.data)
                task=self.delivery.tasks[self.delivery.index]
                target=approach_target(grid,self.pose,self.config['zones'],task['zone_id'],float(self.get_parameter('navigation_clearance_m').value))
                self.delivery.state='approaching';task['state']='approaching'
            self.goal=Goal.from_request(target);self.delivery.goal_id=self.goal.goal_id
            self.delivery.target=dict(target);self.publish_goal();self.log('delivery_'+self.delivery.state)
        except (ValueError,KeyError,TypeError) as exc:
            self.delivery.fail(str(exc));self.stop_leg();self.log('delivery_failed',str(exc))

    def return_base(self):
        self.check_base();self.cancel();self.delivery=Delivery();self.delivery.state='returning';self.start_leg()

    def delivery_tick(self):
        if self.delivery.advance(self.seconds()):
            if self.delivery.state!='complete':self.start_leg()
            else:self.log('delivery_complete')
        elif self.delivery.active:self.log('delivery_'+self.delivery.state,self.delivery.reason)


def main():
    rclpy.init()
    node = MissionManagerNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
