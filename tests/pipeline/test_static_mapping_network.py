"""Real-time transient/static layers across controller TCP and viewer WS."""
import asyncio
import json
import math
import time
import numpy as np
import websockets
import ros_sim as ros
from robot_sim import controller_module,RobotDevices


def test_realtime_moving_object_is_not_static_and_stationary_walls_are_confirmed():asyncio.run(run())


async def run():
    module=controller_module();transport=module.BridgeTransport(port=0);transport.start();assert transport.ready.wait(2.)
    bus=ros.Bus({'viewer_gateway':{'host':'127.0.0.1','port':0},'webots_bridge':{'port':transport.port},
        'mapping':{'radius_m':3.,'resolution_m':.1,'lidar_subsample':1},
        'localization':{'use_lidar_scan_matcher':False}},realtime=True)
    nodes={key:kind() for key,kind in ros.adapters().items()};robot=RobotDevices(module,transport)
    running=True;start=time.monotonic();packets=[]
    async def pump():
        last=time.monotonic()
        while running:
            now=time.monotonic()
            if now-last>=.02:
                robot.dynamic_obstacle=(1.2,.45*math.sin(2*math.pi*(now-start)/1.8),.12)
                robot.step(now-last);last=now
            bus.tick();await asyncio.sleep(.005)
    task=asyncio.create_task(pump());viewer=nodes['viewer']
    try:
        assert await asyncio.to_thread(viewer.server_ready.wait,2.)
        async with websockets.connect(f'ws://127.0.0.1:{viewer.server_port}') as ws:
            deadline=time.monotonic()+4.5
            while time.monotonic()<deadline:
                packet=json.loads(await asyncio.wait_for(ws.recv(),2.));packets.append(packet)
                elapsed=time.monotonic()-start
                if elapsed<2.9:
                    assert not (nodes['mapping'].grid.data==100).any()
                if elapsed>.6:
                    assert packet.get('live_points'), 'Current scan disappeared while waiting for static evidence'
        grid=nodes['mapping'].grid
        assert (grid.data==100).any(),'Stationary walls were never promoted after 3 seconds'
        for gy in range(grid.size):
            for gx in range(grid.size):
                x,y=grid.point((gx,gy))
                if .95<x<1.4 and -.7<y<.7:
                    assert grid.data[gy,gx]!=100,'Moving object left a static trail'
        assert any(p.get('new_points') for p in packets)
        assert not bus.errors,bus.errors
    finally:
        running=False;await task;viewer.close();nodes['bridge'].close();transport.close()
