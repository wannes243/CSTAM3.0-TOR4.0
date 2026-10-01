"""Real TCP/WS + production adapters, with simulated ROS topics and devices."""
import asyncio
import json
import math
from pathlib import Path
import time

import pytest
import websockets
import ros_sim as ros
from robot_sim import controller_module,RobotDevices,known_map


def test_realtime_controller_to_viewer_pipeline():
    asyncio.run(run_pipeline())


async def run_pipeline():
    module=controller_module()
    transport=module.BridgeTransport(port=0);transport.start()
    assert transport.ready.wait(2.),'Controller TCP server did not start'
    bus=ros.Bus({'viewer_gateway':{'host':'127.0.0.1','port':0},
                 'webots_bridge':{'port':transport.port},
                 'mapping':{'radius_m':6.,'resolution_m':.1}},realtime=True)
    nodes={key:kind() for key,kind in ros.adapters().items()}
    robot=RobotDevices(module,transport)
    running=True
    sensor_ages=[];received=[];start=time.monotonic()

    async def pump():
        last=time.monotonic()
        while running:
            now=time.monotonic()
            if now-last>=.02:
                robot.step(now-last);last=now
            bus.tick()
            if nodes['bridge'].connected and nodes['bridge'].last_sensor_stamp_ns is not None:
                sensor_ages.append((time.time_ns()-nodes['bridge'].last_sensor_stamp_ns)*1e-9)
            await asyncio.sleep(.005)

    async def until(predicate,timeout=5.):
        end=time.monotonic()+timeout
        while not predicate():
            assert time.monotonic()<end,'Pipeline condition timed out'
            await asyncio.sleep(.01)

    task=asyncio.create_task(pump())
    viewer=nodes['viewer']
    try:
        assert await asyncio.to_thread(viewer.server_ready.wait,2.)
        async with websockets.connect(f'ws://127.0.0.1:{viewer.server_port}') as ws:
            async def receive():
                async for raw in ws:received.append((time.monotonic(),json.loads(raw)))
            reader=asyncio.create_task(receive())
            async def send(kind,**payload):await ws.send(json.dumps(dict(type=kind,**payload)))
            async def held(v=0.,omega=0.,duration=.3):
                end=time.monotonic()+duration
                while time.monotonic()<end:
                    await send('drive',v=v,omega=omega);await asyncio.sleep(.08)

            await until(lambda:any('robot' in packet for _,packet in received))
            import_start=time.monotonic()
            await send('nav_map',request_id='network-import',**known_map())
            await until(lambda:viewer.latest.get('operation',{}).get('state') in ('complete','error'),20.)
            assert viewer.latest['operation']['state']=='complete',viewer.latest['operation']
            import_delay=time.monotonic()-import_start
            assert nodes['mapping'].mode=='known'
            await until(lambda:any(packet.get('scan_matching',{}).get('known_map_request_id')=='network-import' for _,packet in received))
            await send('nav_clear_map',request_id='network-live')
            await until(lambda:viewer.latest.get('operation',{}).get('request_id')=='network-live' and viewer.latest['operation']['state']=='complete')
            assert nodes['mapping'].mode=='live'
            await send('set_scan',enabled=False)
            await until(lambda:not robot.controller.scanning)
            before=robot.yaw
            await held(omega=.6)
            await until(lambda:robot.yaw>before+.04)
            assert robot.controller.scanning,'Drive did not restart disabled LiDAR'
            await send('stop');await until(lambda:abs(robot.w)<1e-6)
            before=robot.yaw
            await held(omega=-.6)
            await until(lambda:robot.yaw<before-.04)
            await send('stop');await until(lambda:abs(robot.w)<1e-6)
            before=robot.x
            await held(v=.3)
            await until(lambda:robot.x>before+.03)
            await send('stop');await until(lambda:abs(robot.v)<1e-6)
            before=robot.x
            await held(v=-.3)
            await until(lambda:robot.x<before-.03)
            await send('stop');await until(lambda:abs(robot.v)<1e-6)

            # Reconnect must restore scan publishing even when simulation time restarts.
            robot.sim_time=0.
            reconnect_start=time.monotonic()
            nodes['bridge'].sock.shutdown(2)
            old_stamp=nodes['bridge'].pub_scan.messages[-1].header.stamp
            await until(lambda:not nodes['bridge'].connected,2.)
            await until(lambda:nodes['bridge'].connected and nodes['bridge'].pub_scan.messages[-1].header.stamp!=old_stamp,3.)
            reconnect_delay=time.monotonic()-reconnect_start
            assert reconnect_delay<1.5

            robot.rear_wall=True
            await asyncio.sleep(.15)
            before=robot.x
            await held(v=-.3,duration=.25)
            assert abs(robot.x-before)<.005,'Rear wall did not stop reverse motion'
            assert 'rear_obstacle' in json.loads(nodes['safety'].status.messages[-1].data)['reason']
            robot.rear_wall=False
            await send('stop');await asyncio.sleep(.2)

            await send('reset_odom',request_id='network-reset')
            await until(lambda:viewer.latest.get('operation',{}).get('request_id')=='network-reset'
                and viewer.latest['operation']['state']=='complete')
            assert math.hypot(*nodes['localization'].ekf.pose[:2])<.01
            assert abs(nodes['localization'].ekf.pose[2])<.01

            await send('clear_map',request_id='network-clear')
            await until(lambda:viewer.latest.get('operation',{}).get('request_id')=='network-clear' and viewer.latest['operation']['state']=='complete')
            assert nodes['mapping'].map_id=='network-clear'
            assert any(all(value==-1 for value in message.data) for message in nodes['mapping'].pub_map.messages)
            await until(lambda:any(packet.get('map',{}).get('map_id')=='network-clear' for _,packet in received))

            await send('nav_goal',x=.25,y=0.,yaw_deg=15.)
            await until(lambda:abs(robot.v)>.01 or abs(robot.w)>.01)
            await until(lambda:any(packet.get('navigation',{}).get('state')=='goal_reached' for _,packet in received),15.)
            assert math.hypot(nodes['localization'].ekf.pose[0]-.25,nodes['localization'].ekf.pose[1])<.06
            assert abs(robot.v)<.01 and abs(robot.w)<.03

            await send('nav_goal',x=1.,y=0.)
            await until(lambda:robot.v>.01)
            stop_start=time.monotonic();await send('nav_stop')
            await until(lambda:abs(robot.v)<1e-6)
            stop_delay=time.monotonic()-stop_start
            assert stop_delay<.4

            # Lost browser heartbeats must stop the robot without an explicit stop.
            await held(v=.2,duration=.2)
            lost_start=time.monotonic()
            await until(lambda:abs(robot.v)<1e-6,1.)
            watchdog_delay=time.monotonic()-lost_start
            assert watchdog_delay<.7
            await send('drive',v=float('nan'))
            await until(lambda:any(packet.get('type')=='command_result' and packet.get('state')=='error' for _,packet in received))
            reader.cancel()
            try:await reader
            except asyncio.CancelledError:pass

        elapsed=time.monotonic()-start
        scan_packets=[stamp for stamp,packet in received if packet.get('type')=='scan']
        rate=(len(scan_packets)-1)/(scan_packets[-1]-scan_packets[0])
        ages=sorted(sensor_ages)
        p95=ages[int(.95*(len(ages)-1))]
        assert rate>=7.,f'Viewer rate fell to {rate:.1f} Hz'
        assert p95<.2,f'Sensor age p95 is {p95:.3f}s'
        assert not bus.errors,bus.errors
        report={'transport':'real TCP and WebSocket','ros':'in-process message/topic doubles',
                'robot':'ideal wheel kinematics and raycast LiDAR; not Webots physics',
                'elapsed_s':round(elapsed,3),'viewer_hz':round(rate,2),
                'sensor_age_p95_s':round(p95,4),'cancel_stop_s':round(stop_delay,4),
                'reconnect_first_scan_s':round(reconnect_delay,4),
                'map_import_localization_s':round(import_delay,4),
                'lost_heartbeat_stop_s':round(watchdog_delay,4),'result':'passed'}
        out=Path(__file__).parents[1]/'results/realtime_pipeline_validation.json'
        out.write_text(json.dumps(report,indent=2)+'\n')
    finally:
        running=False
        await task
        viewer.close();nodes['bridge'].close();transport.close()
        executor=nodes['localization'].operation_executor
        if executor is not None:executor.shutdown(wait=True,cancel_futures=True)
