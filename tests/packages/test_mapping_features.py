import struct
import numpy as np
import ros_sim as ros
from fabtino_mapping.mapping_node import MappingNode


def setup():
    bus=ros.Bus({'mapping':{'resolution_m':.1,'radius_m':3.,'lidar_subsample':1}})
    return bus,MappingNode()


def observe(bus,node,distance=1.,pose=(0.,0.,0.)):
    scan=ros.LaserScan();scan.header.stamp=ros.stamp(bus.clock_ns());scan.ranges=[distance]
    node.integrate_scan(1,scan,pose,'test')
    bus.tick(.1)
    return scan


def test_temporary_returns_are_immediate_navigation_obstacles_but_not_static():
    bus,node=setup()
    observe(bus,node)
    assert node.grid.data[node.grid.cell(1.,0.)[1],node.grid.cell(1.,0.)[0]]!=100
    assert node.pub_points.messages[-1].width==0
    assert node.pub_live_points.messages[-1].width==1
    local=np.array(node.pub_navigation.messages[-1].data).reshape(node.grid.data.shape)
    cell=node.grid.cell(1.,0.);assert local[cell[1],cell[0]]==100
    bus.tick(.6);node.publish_local()
    assert not node.dynamic_cells
    assert 100 not in node.pub_navigation.messages[-1].data


def test_stationary_returns_need_over_three_seconds_and_free_rays_remove_departed_object():
    bus,node=setup()
    for index in range(31):observe(bus,node)
    assert node.pub_points.messages[-1].width==0 # exactly 3 seconds
    observe(bus,node)
    assert node.pub_points.messages[-1].width==1
    assert 100 in node.grid.data
    observe(bus,node,2.)
    cell=node.grid.cell(1.,0.)
    assert node.grid.data[cell[1],cell[0]]==0
    assert cell not in node.static_filter.candidates


def test_translating_robot_observes_the_same_fixed_world_hit():
    bus,node=setup()
    for index in range(33):observe(bus,node,2.-index*.01,(index*.01,0.,0.))
    assert node.pub_points.messages[-1].width==1
    assert struct.unpack('<fff',node.pub_points.messages[-1].data)[:2]==(2.,0.)


def test_moving_object_stale_replays_and_clear_cannot_promote_old_evidence():
    bus,node=setup()
    for index in range(40):observe(bus,node,1.+index*.02)
    assert not (node.grid.data==100).any()
    old=observe(bus,node,1.)
    before=node.grid.data.copy();node.integrate_scan(999,old,(10.,0.,0.),'stale')
    assert np.array_equal(before,node.grid.data)
    node.clear_cb(ros.String())
    observe(bus,node)
    assert not (node.grid.data==100).any()


def test_imported_static_map_is_not_erased_or_promoted_by_transient_points():
    bus,node=setup();node.mode='known';node.grid.data.fill(0);node.grid.set_cell(node.grid.cell(2.,0.),100)
    before=node.grid.data.copy()
    for _ in range(33):observe(bus,node,1.)
    assert np.array_equal(before,node.grid.data)
    assert node.pub_points.messages[-1].width==0
    assert node.pub_live_points.messages[-1].width==1
