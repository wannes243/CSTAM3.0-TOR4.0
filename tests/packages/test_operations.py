"""Package-level regressions for viewer, mapping, localization and bridge."""
import json
import math
from pathlib import Path
import queue
import time
from unittest.mock import patch

import numpy as np
import pytest
import ros_sim as ros
from fabtino_core.operations import imported_grid


@pytest.fixture
def stack():
    bus=ros.Bus({'viewer_gateway':{'start_server':False},'mapping':{'resolution_m':.1,'radius_m':3.}})
    nodes=ros.adapters()
    with patch('threading.Thread.start'):
        instances={key:kind() for key,kind in nodes.items()}
    yield bus,instances
    executor=instances['localization'].operation_executor
    if executor is not None:executor.shutdown(wait=True,cancel_futures=True)


def sensors(bus,node,distance=0.,yaw=0.):
    wheel=ros.JointState();wheel.header.stamp=ros.stamp(bus.clock_ns());wheel.position=[distance/.1]*4
    imu=ros.Imu();imu.header.stamp=ros.stamp(bus.clock_ns())
    imu.orientation.z=math.sin(yaw/2);imu.orientation.w=math.cos(yaw/2)
    node.wheel_cb(wheel);node.imu_cb(imu)
    bus.flush()


def test_core_rejects_malformed_import_without_mutating_any_map():
    for payload in ({'resolution':float('nan'),'radius':3.,'grid':[]},
                    {'resolution':.001,'radius':30.,'grid':[]},
                    {'resolution':1.,'radius':1.,'grid':[0,0,0,42]}):
        with pytest.raises(ValueError):imported_grid(payload)
    grid=imported_grid({'resolution':1.,'radius':1.,'grid':[-1,0,100,0]})
    assert grid.data.tolist()==[[-1,0],[100,0]]


def test_viewer_drive_reenables_lidar_and_preserves_both_turn_signs(stack):
    bus,nodes=stack
    viewer=nodes['viewer']
    viewer.latest['scan_enabled']=False
    for angular in (-1.,1.):
        viewer.cmd('drive',{'v':.2,'omega':angular})
        assert viewer.pub_teleop.messages[-1].twist.angular.z==angular
    assert viewer.pub_scan_enable.messages[-1].data is True
    with pytest.raises(ValueError):viewer.cmd('drive',{'v':float('nan')})


def test_viewer_reports_status_even_before_first_odometry(stack):
    _,nodes=stack
    viewer=nodes['viewer']
    viewer.latest['safety']=json.dumps({'stop_active':True,'reason':'sensor_not_ready'})
    packet=viewer.packet()
    assert packet['safety']['reason']=='sensor_not_ready'
    assert 'robot' not in packet


def test_viewer_does_not_replay_stale_pose_or_map_after_reset(stack):
    bus,nodes=stack
    viewer=nodes['viewer']
    odom=ros.Odometry();odom.header.stamp=ros.stamp(bus.clock_ns());odom.pose.pose.position.x=10.
    old_map=ros.OccupancyGrid();old_map.header.stamp=ros.stamp(bus.clock_ns())
    viewer.latest['odom']=odom
    bus.tick(.01)
    viewer.cmd('reset_odom',{'request_id':'fresh-reset'})
    viewer.latest['map']=old_map
    packet=viewer.packet()
    assert 'robot' not in packet and 'map' not in packet
    bus.flush();bus.tick(.02)
    sensors(bus,nodes['localization'],10.,1.)
    bus.flush()
    assert viewer.latest['operation']['state']=='complete'
    assert viewer.packet()['robot']['x']==0.


def test_mapping_clear_has_ack_and_does_not_reintegrate_queued_old_scans(stack):
    bus,nodes=stack
    mapping,viewer=nodes['mapping'],nodes['viewer']
    mapping.grid.data[1,1]=100
    scan=ros.LaserScan();scan.header.stamp=ros.stamp(bus.clock_ns()-1_000_000);scan.ranges=[2.]
    mapping.scan_cb(scan)
    viewer.cmd('clear_map',{'request_id':'clear-test'})
    assert viewer.latest['operation']['state']=='pending'
    bus.flush()
    assert viewer.latest['operation']['state']=='complete'
    assert not (mapping.grid.data==100).any()
    assert not mapping.scan_queue
    mapping.scan_cb(scan)
    assert not mapping.scan_queue
    assert mapping.pub_map.messages[-1].data.count(-1)==mapping.grid.size**2


def test_localization_reset_consumes_new_absolute_encoders_and_resets_yaw(stack):
    bus,nodes=stack
    localization,viewer=nodes['localization'],nodes['viewer']
    sensors(bus,localization,10.,1.2)
    bus.tick()
    sensors(bus,localization,10.1,1.3)
    assert localization.ekf.pose[0]>.05
    viewer.cmd('reset_odom',{'request_id':'reset-test'})
    bus.flush()
    assert viewer.latest['operation']['state']=='pending'
    bus.tick()
    sensors(bus,localization,20.,2.)
    assert viewer.latest['operation']['state']=='complete'
    assert np.allclose(localization.ekf.pose,(0.,0.,0.))
    bus.tick()
    sensors(bus,localization,20.01,2.)
    assert abs(localization.ekf.pose[0]-.01)<1e-6


def test_bridge_keeps_scan_reset_and_newest_motion_when_motor_updates_flood(stack):
    _,nodes=stack
    bridge=nodes['bridge']
    bridge.send({'type':'scan_enable','enabled':False})
    bridge.send({'type':'reset','request_id':'reset'})
    for i in range(1000):
        bridge.last_cmd=time.monotonic()
        bridge.send({'type':'cmd_vel','v':i/10000.,'omega':0.})
    commands=bridge.outbound_commands()
    assert [command['type'] for command in commands]==['scan_enable','reset','cmd_vel']
    assert commands[-1]['v']==.0999
    assert bridge.outbound_commands()==[]


def test_navigation_and_safety_commands_reach_bridge_after_full_topic_fanout(stack):
    bus,nodes=stack
    localization,mapping,viewer=nodes['localization'],nodes['mapping'],nodes['viewer']
    sensors(bus,localization)
    scan=ros.LaserScan();scan.header.stamp=ros.stamp(bus.clock_ns());scan.ranges=[2.]*41
    scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/40
    for callback in bus.subscribers['/fabtino/scan']:bus.pending.append((callback,scan))
    mapping.publish_grid(mapping.pub_map,mapping.grid.data,mapping.grid.resolution,mapping.grid.radius,'map')
    bus.flush()
    viewer.cmd('nav_goal',{'x':1.,'y':0.})
    bus.flush();bus.tick(.1)
    output=nodes['safety'].pub.messages[-1]
    assert output.linear.x>0, {key:node.status.messages[-1].data for key,node in nodes.items() if hasattr(node,'status')}
    assert nodes['bridge'].latest_motion['v']>0
    viewer.cmd('stop',{});bus.flush();bus.tick(.02)
    assert nodes['bridge'].latest_motion['v']==0.


def test_import_error_releases_viewer_and_preserves_previous_map(stack):
    bus,nodes=stack
    viewer,mapping=nodes['viewer'],nodes['mapping']
    mapping.grid.data[1,1]=100
    payload={'request_id':'image','resolution':1.,'radius':2.,'grid':[0]*16}
    viewer.cmd('nav_map',payload);bus.flush()
    assert viewer.operation is not None
    status=ros.String(json.dumps({'node':'localization','request_id':'image','state':'error','reason':'No matching scan'}))
    for callback in bus.subscribers['/viewer/operation_status']:bus.pending.append((callback,status))
    bus.flush()
    assert viewer.operation is None
    assert viewer.latest['operation']['state']=='error'
    assert mapping.pending_import is None
    assert mapping.grid.data[1,1]==100


def test_configure_map_changes_ros_grid_and_completes(stack):
    bus,nodes=stack
    nodes['viewer'].cmd('configure_map',{'resolution':.2,'radius':2.,'request_id':'settings'})
    bus.flush()
    assert nodes['mapping'].grid.size==20
    assert nodes['viewer'].latest['operation']['state']=='complete'


def test_operation_timeout_cancels_workers_and_allows_new_drive(stack):
    bus,nodes=stack
    viewer=nodes['viewer']
    viewer.cmd('nav_map',{'resolution':1.,'radius':2.,'grid':[0]*16,'request_id':'timeout'})
    bus.flush()
    bus.now_ns+=31_000_000_000
    viewer.push();bus.flush()
    assert viewer.operation is None
    assert nodes['localization'].operation is None
    assert nodes['mapping'].pending_import is None
    viewer.cmd('drive',{'v':.1})
    assert viewer.pub_teleop.messages[-1].twist.linear.x==.1


def test_new_clear_cancels_pending_import_in_both_nodes(stack):
    bus,nodes=stack
    viewer=nodes['viewer']
    viewer.cmd('nav_map',{'resolution':1.,'radius':2.,'grid':[0]*16,'request_id':'old'})
    bus.flush()
    viewer.cmd('clear_map',{'request_id':'new'});bus.flush()
    assert nodes['localization'].operation is None
    assert nodes['mapping'].pending_import is None
    assert viewer.latest['operation']['request_id']=='new'
    assert viewer.latest['operation']['state']=='complete'


def test_bridge_reset_service_resets_estimator_and_map(stack):
    bus,nodes=stack
    sensors(bus,nodes['localization'],10.,.8)
    response=nodes['bridge'].reset_cb(None,ros.NS())
    bus.flush()
    assert response.success
    assert nodes['localization'].operation['type']=='reset_odom'
    bus.tick();sensors(bus,nodes['localization'],10.,.8)
    assert np.allclose(nodes['localization'].ekf.pose,(0.,0.,0.))


def known_room():
    from robot_sim import known_map,ray_distance
    points=[]
    for angle in np.linspace(-math.pi,math.pi,121):
        distance=ray_distance(0.,0.,angle)
        points.append((distance*math.cos(angle),distance*math.sin(angle)))
    return known_map(),points


def test_default_known_map_matching_recovers_asymmetric_room():
    from fabtino_localization.localization_node import LocalizationNode
    payload,points=known_room()
    match=LocalizationNode.localize_import(payload,points,1.)
    assert math.hypot(match.x_m,match.y_m)<.15
    assert abs(match.yaw_rad)<.1


def test_successful_import_commits_after_localization_and_preserves_imu_heading(stack):
    from concurrent.futures import Future
    from fabtino_localization.localization_node import LocalizationNode
    bus,nodes=stack
    viewer,mapping,localization=nodes['viewer'],nodes['mapping'],nodes['localization']
    sensors(bus,localization,10.,1.2)
    payload,points=known_room()
    viewer.cmd('nav_map',dict(payload,request_id='success'));bus.flush()
    assert mapping.mode=='live'
    # Complete with the real matching result; the realtime worker start is
    # separately covered below, without repeating an expensive global search.
    match=LocalizationNode.localize_import(payload,points,1.)
    future=Future();future.set_result(match);localization.operation_future=future
    localization.poll_operations();bus.flush()
    assert viewer.latest['operation']['state']=='complete'
    assert viewer.latest['scan_matching']['known_map_request_id']=='success'
    assert mapping.mode=='known' and mapping.map_id=='success'
    assert np.array_equal(mapping.grid.data.ravel(),payload['grid'])
    yaw=localization.ekf.pose[2]
    bus.tick();sensors(bus,localization,10.,1.2)
    assert abs(localization.ekf.pose[2]-yaw)<1e-6
    viewer.cmd('nav_clear_map',{'request_id':'live'});bus.flush()
    assert mapping.mode=='live'
    assert not (mapping.grid.data==100).any()


def test_import_worker_starts_only_for_fresh_stationary_scan(stack):
    from unittest.mock import Mock
    bus,nodes=stack
    localization,viewer=nodes['localization'],nodes['viewer']
    sensors(bus,localization)
    viewer.cmd('nav_map',{'resolution':1.,'radius':2.,'grid':[0]*16,'request_id':'worker'});bus.flush()
    executor=Mock();localization.operation_executor=executor
    scan=ros.LaserScan();scan.ranges=[2.]*41;scan.angle_min=-math.pi;scan.angle_increment=math.pi/20
    scan.header.stamp=ros.stamp(bus.clock_ns()-1_000_000)
    localization.process_scan(scan)
    executor.submit.assert_not_called()
    scan.header.stamp=ros.stamp(bus.clock_ns()+1_000_000)
    localization.last_gyro_z=.5;localization.process_scan(scan)
    executor.submit.assert_not_called()
    scan.header.stamp=ros.stamp(bus.clock_ns()+2_000_000)
    localization.last_gyro_z=0.;localization.process_scan(scan)
    executor.submit.assert_called_once()
    localization.operation_executor=None
