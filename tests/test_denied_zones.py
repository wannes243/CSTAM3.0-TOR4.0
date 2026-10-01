import math
import numpy as np
import pytest
from fabtino_core.navigation.planner import OccupancyGrid,astar
from fabtino_core.navigation.zones import validate_config,point_denied,segment_denied,zone_mask,approach_target,motion_denied


def zones(x=0.,y=0.,radius=.3):return [dict(id='station',name='Station',x=x,y=y,radius=radius)]
def grid():
    result=OccupancyGrid(.1,3.);result.data.fill(0);return result


@pytest.mark.parametrize('bad',[None,[],{'zones':[{}]},{'zones':[dict(id='a',name='A',radius=float('nan'),x=0,y=0)]},
    {'zones':zones()*2},{'zones':[dict(id='a',name='A',radius=.01,x=0,y=0)]},
    {'base':{'x':0,'y':float('inf')}}])
def test_invalid_config_is_rejected(bad):
    with pytest.raises((ValueError,KeyError,TypeError)):validate_config(bad)


def test_routes_detour_and_every_segment_keeps_robot_outside_circle():
    map=grid();denied=zones();blocked=zone_mask(map,denied,.6)
    route=astar(map,map.cell(-2.,0.),map.cell(2.,0.),blocked)
    assert route
    points=[map.point(cell) for cell in route]
    assert max(abs(y) for x,y in points)>.9
    assert not any(segment_denied(a,b,denied,.6) for a,b in zip(points,points[1:]))


def test_approach_uses_closest_reachable_safe_boundary_and_faces_zone():
    map=grid();denied=zones(1.,0.,.2)
    target=approach_target(map,(-1.,0.),denied,'station')
    assert not point_denied(target['x'],target['y'],denied,.72)
    assert .92<=math.hypot(target['x']-1,target['y'])<1.02
    assert target['x']<1.
    assert abs(math.remainder(math.radians(target['yaw_deg'])-math.atan2(-target['y'],1-target['x']),2*math.pi))<1e-10


def test_approach_respects_unreachable_side_and_other_zones():
    map=grid();map.data[:,30]=100 # entire wall x=0: right side unreachable
    denied=zones(1.,0.,.2)
    target=approach_target(map,(-2.,0.),denied,'station')
    assert target['x']<-.6
    assert not point_denied(target['x'],target['y'],denied,.6)


def test_inside_zone_start_cannot_produce_delivery_approach():
    with pytest.raises(ValueError,match='safe start'):approach_target(grid(),(0.,0.),zones(),'station')


@pytest.mark.parametrize('v,yaw',[(.5,0.),(-.5,math.pi)])
def test_swept_motor_guard_blocks_forward_and_reverse_entry(v,yaw):
    denied=zones(.85,0.,.1)
    assert motion_denied((0.,0.,yaw),v,0.,denied,.6,horizon=.4)
    assert motion_denied((0.,0.,yaw),v,-1.,denied,.6,horizon=.4)
    assert not motion_denied((0.,0.,yaw),0.,1.,denied,.6)


def test_cell_intersections_are_blocked_even_if_cell_centre_is_outside():
    map=grid();denied=zones(.01,.01,.05)
    assert zone_mask(map,denied)[map.cell(0.,0.)[1],map.cell(0.,0.)[0]]
