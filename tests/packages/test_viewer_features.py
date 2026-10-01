import json
import struct
import pytest
import ros_sim as ros
from fabtino_viewer.viewer_gateway import ViewerGateway


@pytest.fixture
def viewer():
    bus=ros.Bus({'viewer_gateway':{'start_server':False}})
    return bus,ViewerGateway()


def zone():return dict(id='a',name='Station',x=1.,y=0.,radius=.2)


def test_config_delivery_and_return_requests_are_forwarded_and_bad_zones_are_atomic(viewer):
    bus,node=viewer
    node.cmd('set_navigation_config',dict(request_id='zones',zones=[zone()],base=None))
    request=json.loads(node.pub_req.messages[-1].data)
    assert request['type']=='configure' and request['zones']==[zone()]
    count=len(node.pub_req.messages)
    with pytest.raises(ValueError):node.cmd('set_navigation_config',dict(zones=[zone(),zone()]))
    assert len(node.pub_req.messages)==count
    for kind in ('delivery_start','delivery_return'):
        node.cmd(kind,dict(tasks=[dict(zone_id='a',delay_s=.2)]))
        assert json.loads(node.pub_req.messages[-1].data)['type']==kind


def test_bad_import_metadata_preserves_current_map_and_does_not_start_operation(viewer):
    _,node=viewer;original=ros.OccupancyGrid();node.latest['map']=original
    with pytest.raises(ValueError):
        node.cmd('nav_map',dict(resolution=1.,radius=1.,grid=[0]*4,navigation_config=dict(zones=[zone(),zone()])))
    assert node.operation is None and node.latest['map'] is original


def test_map_reset_waits_for_annotation_confirmation_and_blocks_new_delivery(viewer):
    bus,node=viewer;node.cmd('reset_odom',dict(request_id='reset'))
    for name in ('mapping','localization'):
        node.operation_cb(ros.String(json.dumps(dict(node=name,request_id='reset',state='complete',stamp_ns=bus.clock_ns()))))
    assert node.operation is not None
    with pytest.raises(ValueError):node.cmd('delivery_start',dict(tasks=[]))
    node.operation_cb(ros.String(json.dumps(dict(node='mission',request_id='reset',state='complete'))))
    assert node.operation is None and node.latest['operation']['state']=='complete'


def test_live_scan_layer_expires_without_replaying_old_returns(viewer):
    bus,node=viewer
    cloud=ros.PointCloud2();cloud.header.stamp=ros.stamp(bus.clock_ns());cloud.width=1
    cloud.fields=[ros.PointField(name=name,offset=index*4,datatype=7) for index,name in enumerate(('x','y','z'))]
    cloud.data=struct.pack('<fff',1.,2.,0.);node.latest_live_cloud=cloud
    assert node.packet()['live_points']==[[1.,2.,0.]]
    bus.tick(.6);assert node.packet()['live_points']==[]
