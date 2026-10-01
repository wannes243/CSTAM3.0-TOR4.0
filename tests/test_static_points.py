import math
import pytest
from fabtino_core.static_points import StaticPointFilter


def test_strictly_more_than_three_seconds_and_clear_restarts():
    filter=StaticPointFilter(.1)
    for tenth in range(31):assert not filter.observe({(0,0):[(0.,0.)]},tenth*100_000_000)
    assert filter.observe({(0,0):[(0.,0.)]},3_100_000_000)=={(0,0)}
    filter.clear()
    assert not filter.observe({(0,0):[(0.,0.)]},4_000_000_000)


@pytest.mark.parametrize('options',[dict(persistence_s=2.),dict(persistence_s=float('nan')),dict(max_gap_s=0.),dict(tolerance_m=float('inf'))])
def test_invalid_parameters_cannot_disable_the_three_second_constraint(options):
    with pytest.raises(ValueError):StaticPointFilter(.1,**options)


def test_anchor_never_follows_a_slow_moving_point_inside_one_cell():
    filter=StaticPointFilter(.1)
    for tenth in range(100):
        x=.001*tenth
        assert not filter.observe({(0,0):[(x,0.)]},tenth*100_000_000)
    assert filter.candidates[(0,0)].first_ns>=5_000_000_000


def test_missing_observation_restarts_and_stale_scans_never_accumulate_time():
    filter=StaticPointFilter(.1)
    for tenth in range(21):filter.observe({(0,0):[(0.,0.)]},tenth*100_000_000)
    assert not filter.observe({(0,0):[(0.,0.)]},2_600_000_000)
    assert filter.candidates[(0,0)].first_ns==2_600_000_000
    assert not filter.observe({(0,0):[(0.,0.)]},2_600_000_000)
    assert not filter.observe({(0,0):[(0.,0.)]},1_000_000_000)
    filter.forget([(0,0)])
    assert not filter.observe({(0,0):[(0.,0.)]},2_700_000_000)


@pytest.mark.parametrize('angular',[-1.,1.])
def test_rotating_sensor_keeps_a_world_point_static(angular):
    from fabtino_core.localization.frames import robot_to_world_point
    filter=StaticPointFilter(.1)
    accepted=set()
    for step in range(33):
        yaw=angular*step*.04
        local=(2*math.cos(yaw),-2*math.sin(yaw))
        world=robot_to_world_point((0.,0.,yaw),local)
        accepted=filter.observe({(20,0):[world]},step*100_000_000)
    assert accepted=={(20,0)}
