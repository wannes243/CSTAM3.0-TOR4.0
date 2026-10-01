import pytest
from fabtino_core.navigation.delivery import Delivery


def start(returning=True):
    delivery=Delivery();delivery.start([dict(zone_id='a',delay_s=3.),dict(zone_id='a',delay_s=0.)],[dict(id='a')],returning)
    delivery.state='approaching';delivery.goal_id='first';return delivery


def test_wait_starts_only_on_matching_settled_arrival_and_then_returns():
    delivery=start()
    delivery.reached(10.,'old');assert delivery.state=='approaching'
    delivery.reached(11.,'first');assert delivery.state=='waiting'
    assert not delivery.advance(13.999)
    assert delivery.snapshot(13.)['remaining_s']==1.
    assert delivery.advance(14.) and delivery.state=='planning'
    delivery.state='approaching';delivery.goal_id='second';delivery.reached(20.,'second')
    assert delivery.advance(20.) and delivery.state=='returning'
    delivery.goal_id='base';delivery.reached(21.,'second');assert delivery.state=='returning'
    delivery.reached(22.,'base');assert delivery.state=='complete'
    assert delivery.snapshot(22.)['completed']==2


@pytest.mark.parametrize('waiting',[False,True])
def test_cancel_during_motion_or_wait_never_advances(waiting):
    delivery=start()
    if waiting:delivery.reached(1.,'first')
    delivery.cancel();assert delivery.state=='cancelled'
    assert not delivery.advance(100.)
    delivery.reached(100.,'first');assert delivery.state=='cancelled'


def test_no_return_and_failure_stop_queue():
    delivery=start(False);delivery.reached(1.,'first');delivery.advance(4.)
    delivery.state='approaching';delivery.goal_id='second';delivery.reached(10.,'second');delivery.advance(10.)
    assert delivery.state=='complete'
    delivery=start();delivery.fail('No safe route');assert not delivery.advance(100.)
    assert delivery.snapshot(100.)['tasks'][0]['state']=='failed'


@pytest.mark.parametrize('tasks',[[],None,[None],[dict(zone_id='missing')],[dict(zone_id='a',delay_s=-1)],[dict(zone_id='a',delay_s=float('nan'))]])
def test_malformed_tasks_do_not_replace_existing_queue(tasks):
    delivery=start();old=delivery.snapshot(0.)
    with pytest.raises((ValueError,TypeError,KeyError)):delivery.start(tasks,[dict(id='a')])
    assert delivery.snapshot(0.)==old
