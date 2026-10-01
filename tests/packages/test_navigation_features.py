import json
import math
from unittest.mock import patch
import pytest
import ros_sim as ros
from fabtino_navigation.mission_manager_node import MissionManagerNode
from fabtino_navigation.global_planner_node import GlobalPlannerNode
from fabtino_safety.safety_supervisor_node import SafetySupervisor
from fabtino_core.navigation.zones import point_denied,segment_denied


def config():return dict(zones=[dict(id='a',name='Station A',x=1.2,y=0.,radius=.1)],base=dict(x=0.,y=0.,yaw_deg=0.))
def map_message(bus):
    msg=ros.OccupancyGrid();msg.header.stamp=ros.stamp(bus.clock_ns());msg.info.resolution=.1
    msg.info.width=msg.info.height=60;msg.info.origin.position.x=msg.info.origin.position.y=-3.
    msg.data=[0]*3600;return msg
def request(node,**value):node.req_cb(ros.String(json.dumps(value)))


def setup(overrides=None):
    bus=ros.Bus(overrides);mission=MissionManagerNode();planner=GlobalPlannerNode();safety=SafetySupervisor()
    mission.pose_cb(ros.Odometry());mission.map_cb(map_message(bus),True)
    planner.pose_cb(ros.Odometry());planner.map_cb(map_message(bus),True)
    return bus,mission,planner,safety


def test_config_ack_fanout_stops_previous_navigation_and_rejects_bad_edit():
    bus,mission,planner,safety=setup()
    request(mission,type='goal',x=.2,y=0.)
    request(mission,type='configure',request_id='zones',**config());bus.flush()
    assert planner.zones==safety.zones==mission.config['zones']
    assert mission.goal is None and planner.goal is None
    assert json.loads(mission.config_pub.messages[-1].data)['state']=='complete'
    request(mission,type='configure',request_id='bad',zones=config()['zones']*2)
    assert mission.config==config()
    assert json.loads(mission.config_pub.messages[-1].data)['state']=='error'


def test_delivery_arrival_wait_queue_and_return_are_coordinated():
    bus,mission,planner,_=setup();mission.set_config(config());bus.flush()
    request(mission,type='delivery_start',tasks=[dict(zone_id='a',delay_s=.5),dict(zone_id='a',delay_s=0.)])
    bus.flush();planner.plan();bus.flush()
    first=mission.goal.goal_id
    assert not point_denied(mission.goal.x,mission.goal.y,config()['zones'],.72)
    assert planner.path
    mission.local_cb(ros.String(json.dumps(dict(goal_id='old',state='goal_reached'))));assert mission.delivery.state=='approaching'
    mission.local_cb(ros.String(json.dumps(dict(goal_id=first,state='goal_reached'))));bus.flush()
    assert mission.delivery.state=='waiting' and mission.goal is None
    bus.tick(.49);assert mission.delivery.state=='waiting'
    bus.tick(.11);assert mission.delivery.state=='approaching'
    second=mission.goal.goal_id
    mission.local_cb(ros.String(json.dumps(dict(goal_id=second,state='goal_reached'))));bus.tick(.1)
    assert mission.delivery.state=='returning'
    assert mission.goal.x==mission.goal.y==0.
    mission.local_cb(ros.String(json.dumps(dict(goal_id=mission.goal.goal_id,state='goal_reached'))))
    assert mission.delivery.state=='complete' and mission.goal is None


def test_cancel_waiting_and_manual_override_cannot_resume_queue():
    bus,mission,_,_=setup();mission.set_config(config())
    request(mission,type='delivery_start',tasks=[dict(zone_id='a',delay_s=2.)])
    mission.local_cb(ros.String(json.dumps(dict(goal_id=mission.goal.goal_id,state='goal_reached'))))
    request(mission,type='stop');bus.tick(3.)
    assert mission.delivery.state=='cancelled' and mission.goal is None
    request(mission,type='delivery_start',tasks=[dict(zone_id='a',delay_s=0.)])
    manual=ros.TwistStamped();manual.twist.linear.x=.2;mission.teleop_cb(manual)
    assert mission.delivery.state=='cancelled' and mission.goal is None


def test_global_planner_detours_and_blocks_requested_goal_in_zone():
    bus,mission,planner,_=setup();mission.set_config(config());bus.flush()
    planner.goal_cb(ros.String(json.dumps(dict(type='goal',goal_id='around',x=2.4,y=0.,yaw_deg=None))))
    planner.plan()
    route=json.loads(planner.route_pub.messages[-1].data)['points']
    assert route and max(abs(y) for x,y in route)>.7
    assert not any(segment_denied(a,b,config()['zones'],.6) for a,b in zip(route,route[1:]))
    request(mission,type='goal',x=1.2,y=0.)
    assert json.loads(mission.status.messages[-1].data)['state']=='invalid_request'


def test_safety_stops_swept_navigation_entry_but_allows_manual_recovery():
    bus,_,_,safety=setup();safety.config_cb(ros.String(json.dumps(config())))
    odom=ros.Odometry();odom.header.stamp=ros.stamp(bus.clock_ns());odom.pose.pose.position.x=.45
    scan=ros.LaserScan();scan.header.stamp=ros.stamp(bus.clock_ns());scan.ranges=[5.]
    cmd=ros.TwistStamped();cmd.header.stamp=ros.stamp(bus.clock_ns());cmd.twist.linear.x=.5
    safety.odom_cb(odom);safety.scan_cb(scan);safety.nav_cb(cmd);safety.loop()
    assert safety.pub.messages[-1].linear.x==0.
    assert 'denied_zone' in safety.status.messages[-1].data
    cmd.twist.linear.x=-.2;safety.teleop_cb(cmd);safety.loop()
    assert safety.pub.messages[-1].linear.x==-.2


def test_safe_pose_in_a_conservatively_blocked_cell_can_depart_without_crossing_zone():
    bus,mission,planner,_=setup()
    cfg=dict(zones=[dict(id='a',name='A',x=1.15,y=.35,radius=.1),dict(id='b',name='B',x=.15,y=-1.1,radius=.1)],base=dict(x=0.,y=0.,yaw_deg=0.))
    mission.set_config(cfg);bus.flush()
    odom=ros.Odometry();odom.pose.pose.position.x=.4017;odom.pose.pose.position.y=.0639
    planner.pose_cb(odom)
    planner.goal_cb(ros.String(json.dumps(dict(type='goal',goal_id='departure',x=.05,y=-.25,yaw_deg=None))))
    planner.plan()
    route=json.loads(planner.route_pub.messages[-1].data)['points']
    assert route and not any(segment_denied(a,b,cfg['zones'],.6) for a,b in zip(route,route[1:]))


def test_annotations_are_committed_only_after_successful_map_localization_and_reset_clears():
    bus,mission,_,_=setup();mission.set_config(config())
    new=dict(zones=[],base=dict(x=-1.,y=0.,yaw_deg=90.))
    mission.map_request_cb(ros.String(json.dumps(dict(type='nav_map',request_id='import',navigation_config=new))))
    assert mission.config==config()
    mission.map_operation_cb(ros.String(json.dumps(dict(node='localization',request_id='import',state='complete'))))
    assert mission.config==config()
    mission.map_operation_cb(ros.String(json.dumps(dict(node='mapping',request_id='import',state='error'))))
    assert mission.config==config()
    mission.map_request_cb(ros.String(json.dumps(dict(type='nav_map',request_id='import2',navigation_config=new))))
    mission.map_operation_cb(ros.String(json.dumps(dict(node='mapping',request_id='import2',state='complete'))))
    assert mission.config==new
    mission.map_request_cb(ros.String(json.dumps(dict(type='reset_odom',request_id='reset'))))
    assert mission.config==dict(zones=[],base=None)


def test_persistence_roundtrip_and_failed_save_preserves_previous_config(tmp_path):
    file=tmp_path/'navigation.json'
    bus,mission,_,_=setup({'mission_manager':{'state_file':str(file)}})
    mission.set_config(config())
    restored=MissionManagerNode();assert restored.config==config() and restored.delivery.state=='idle'
    with patch('pathlib.Path.replace',side_effect=OSError('disk unavailable')):
        request(mission,type='configure',request_id='disk-error',zones=[])
    assert mission.config==config()
    assert json.loads(mission.config_pub.messages[-1].data)['state']=='error'


def test_goal_carries_zone_configuration_and_older_heartbeats_cannot_remove_it():
    from fabtino_navigation.local_planner_node import LocalPlannerNode
    bus,mission,planner,safety=setup();local=LocalPlannerNode()
    # A goal may arrive before the independently published config heartbeat.
    mission.config=config();mission.config_revision=4
    request(mission,type='goal',x=2.4,y=0.)
    message=mission.pub.messages[-1]
    for node in (planner,local,safety):
        node.goal_cb(message)
        assert node.zones==config()['zones']
        node.config_cb(ros.String(json.dumps(dict(zones=[],revision=3))))
        assert node.zones==config()['zones']
    planner.plan()
    assert planner.path


def test_persistence_supports_utf8_zone_names(tmp_path):
    file=tmp_path/'navigation.json';bus,mission,_,_=setup({'mission_manager':{'state_file':str(file)}})
    cfg=config();cfg['zones'][0]['name']='توصيل café'
    mission.set_config(cfg)
    assert json.loads(file.read_text(encoding='utf-8'))==cfg
    assert MissionManagerNode().config==cfg
