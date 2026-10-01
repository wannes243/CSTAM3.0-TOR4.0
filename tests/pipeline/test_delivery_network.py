"""Delivery over actual WS/TCP using production nodes and controller.

ROS topics and robot physics are substituted; time and networking are real.
"""
import asyncio
import json
import math
from pathlib import Path
import time
import pytest
import websockets
import ros_sim as ros
from robot_sim import controller_module,RobotDevices


def test_realtime_delivery_zones_waits_cancel_and_return():asyncio.run(run())


async def run():
    module=controller_module();transport=module.BridgeTransport(port=0);transport.start()
    assert transport.ready.wait(2.)
    bus=ros.Bus({'viewer_gateway':{'host':'127.0.0.1','port':0},'webots_bridge':{'port':transport.port},
        'mapping':{'radius_m':3.,'resolution_m':.1},
        # Isolate navigation from scan-matcher corrections on the ideal test device.
        'localization':{'use_lidar_scan_matcher':False}},realtime=True)
    nodes={key:kind() for key,kind in ros.adapters().items()};robot=RobotDevices(module,transport)
    running=True;packets=[];trace=[];ages=[];start=time.monotonic()
    zones=[dict(id='a',name='Station A',x=1.15,y=.35,radius=.1),
           dict(id='b',name='Station B',x=.15,y=-1.1,radius=.1)]
    async def pump():
        last=time.monotonic()
        while running:
            now=time.monotonic()
            if now-last>=.02:robot.step(now-last);last=now
            bus.tick()
            delivery=nodes['mission'].delivery
            trace.append((now,robot.x,robot.y,robot.v,robot.w,delivery.state,delivery.index))
            stamp=nodes['bridge'].last_sensor_stamp_ns
            if stamp is not None:ages.append((time.time_ns()-stamp)*1e-9)
            await asyncio.sleep(.005)
    async def until(predicate,timeout=8.):
        end=time.monotonic()+timeout
        while not predicate():
            if nodes['mission'].delivery.state=='failed' or time.monotonic()>=end:
                pytest.fail(json.dumps(dict(delivery=nodes['mission'].delivery.snapshot(time.time()),pose=list(nodes['localization'].ekf.pose),
                    motor=(robot.v,robot.w),safety=nodes['viewer'].latest.get('safety'),nav=nodes['viewer'].latest.get('nav')),indent=2))
            await asyncio.sleep(.01)
    task=asyncio.create_task(pump());viewer=nodes['viewer']
    try:
        assert await asyncio.to_thread(viewer.server_ready.wait,2.)
        async with websockets.connect(f'ws://127.0.0.1:{viewer.server_port}') as ws:
            async def receive():
                async for raw in ws:packets.append((time.monotonic(),json.loads(raw)))
            reader=asyncio.create_task(receive())
            async def send(kind,**payload):await ws.send(json.dumps(dict(type=kind,**payload)))
            await until(lambda:any('robot' in packet for _,packet in packets))
            config=dict(zones=zones,base=dict(x=0.,y=0.,yaw_deg=0.))
            await send('set_navigation_config',request_id='delivery-zones',**config)
            await until(lambda:(viewer.latest.get('navigation_config') or {}).get('request_id')=='delivery-zones')
            assert viewer.latest['navigation_config']['state']=='complete'
            await send('set_navigation_config',request_id='bad-zones',zones=zones*2)
            await until(lambda:any(p.get('type')=='command_result' and p.get('request_id')=='bad-zones' for _,p in packets))
            assert nodes['mission'].config==config
            await send('delivery_start',tasks=[dict(zone_id='a',delay_s=.4),dict(zone_id='b',delay_s=.3)],return_to_base=True)
            await until(lambda:nodes['mission'].delivery.state=='complete',65.)
            assert nodes['mission'].delivery.snapshot(time.time())['completed']==2
            assert math.hypot(*nodes['localization'].ekf.pose[:2])<.06
            assert abs(robot.v)<1e-6 and abs(robot.w)<1e-6
            first_trace=list(trace)
            for index,delay in [(0,.4),(1,.3)]:
                waits=[row for row in trace if row[5]=='waiting' and row[6]==index]
                assert waits and waits[-1][0]-waits[0][0]>=delay-.08
                assert all(abs(row[3])<.01 and abs(row[4])<.03 for row in waits[2:])
            # Cancelling a long arrival delay must not launch the next task or return.
            await send('delivery_start',tasks=[dict(zone_id='a',delay_s=2.),dict(zone_id='b',delay_s=0.)])
            await until(lambda:nodes['mission'].delivery.state=='waiting',25.)
            cancelled=time.monotonic();await send('nav_stop')
            await until(lambda:nodes['mission'].delivery.state=='cancelled')
            await asyncio.sleep(2.2)
            assert nodes['mission'].delivery.state=='cancelled' and nodes['mission'].delivery.index==0
            assert abs(robot.v)<1e-6 and abs(robot.w)<1e-6
            await send('delivery_return')
            await until(lambda:nodes['mission'].delivery.state=='complete',30.)
            assert math.hypot(*nodes['localization'].ekf.pose[:2])<.06
            reader.cancel()
            try:await reader
            except asyncio.CancelledError:pass
        for _,x,y,*_ in trace:
            assert all(math.hypot(x-z['x'],y-z['y'])>z['radius']+.52 for z in zones),'Robot footprint entered a denied zone'
        assert not bus.errors,bus.errors
        scans=[stamp for stamp,p in packets if p.get('type')=='scan'];rate=(len(scans)-1)/(scans[-1]-scans[0])
        ages.sort();p95=ages[int(.95*(len(ages)-1))]
        assert rate>=7. and p95<.2,(rate,p95)
        report=dict(result='passed',elapsed_s=round(time.monotonic()-start,3),viewer_hz=round(rate,2),sensor_age_p95_s=round(p95,4),
            tasks_completed=2,checks=['all sampled physical footprint positions outside both circles','settled arrival delays','automatic base return','cancel during delay','explicit base return','invalid config rejected'],
            scope='Real WS/TCP, production adapters/controller; simulated ROS topics and ideal robot devices; scan-matcher correction disabled to isolate navigation')
        (Path(__file__).parents[1]/'results/delivery_pipeline_validation.json').write_text(json.dumps(report,indent=2)+'\n')
    finally:
        running=False;await task;viewer.close();nodes['bridge'].close();transport.close()
        executor=nodes['localization'].operation_executor
        if executor is not None:executor.shutdown(wait=True,cancel_futures=True)
