"""Verify the RUNNING ROS Humble/Webots system using genuine DDS and WebSocket.

Start run_full_project.sh first. Commands move the simulation a short distance.
No ROS or robot substitutes are imported. Missing runtime dependencies fail
with an explicit 'unavailable' report. An optional map JSON tests live import.
"""
import argparse
import asyncio
from collections import defaultdict,deque
import json
import math
from pathlib import Path
import sys
import time


def yaw(odom):
    q=odom.pose.pose.orientation
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def angular_delta(a,b):return (a-b+math.pi)%(2*math.pi)-math.pi


def capture_time(message):return message.header.stamp.sec+message.header.stamp.nanosec*1e-9


async def verify(args):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry,OccupancyGrid
    from sensor_msgs.msg import LaserScan,JointState
    from std_msgs.msg import String
    import websockets
    assert not getattr(rclpy,'__fake__',False),'Live test must use genuine rclpy'
    rclpy.init()
    node=Node('fabtino_live_validation')
    latest={};times=defaultdict(list);ages=defaultdict(list);grids=deque(maxlen=100)
    packets=[];checks={};running=True
    def record(key,message):
        latest[key]=message;times[key].append(time.monotonic())
        if key in ('scan','odom','wheels'):ages[key].append(time.time()-capture_time(message))
        if key=='map':grids.append(message)
    def status(key,message):latest[key]=json.loads(message.data)
    for kind,topic,key in ((Odometry,'/fabtino/odometry/filtered','odom'),
                           (Odometry,'/fabtino/ground_truth/odom','truth'),
                           (LaserScan,'/fabtino/scan','scan'),
                           (JointState,'/fabtino/joint_states','wheels'),
                           (OccupancyGrid,'/fabtino/map','map'),
                           (Twist,'/fabtino/cmd_vel','command')):
        node.create_subscription(kind,topic,lambda message,key=key:record(key,message),20)
    for topic,key in (('/fabtino/safety_status','safety'),('/navigation/mission_status','nav'),
                      ('/fabtino/bridge_status','bridge'),('/fabtino/mapping_status','mapping')):
        node.create_subscription(String,topic,lambda message,key=key:status(key,message),10)

    async def spin():
        while running:
            rclpy.spin_once(node,timeout_sec=0.);await asyncio.sleep(.005)
    async def until(predicate,label,timeout=8.):
        deadline=time.monotonic()+timeout
        while not predicate():
            if time.monotonic()>deadline:
                raise AssertionError(f'{label}: safety={latest.get("safety")}; navigation={latest.get("nav")}')
            await asyncio.sleep(.02)
    task=asyncio.create_task(spin())
    try:
        async with websockets.connect(args.ws_url,open_timeout=5.) as ws:
            async def read():
                async for raw in ws:packets.append((time.monotonic(),json.loads(raw)))
            reader=asyncio.create_task(read())
            async def send(kind,**payload):await ws.send(json.dumps(dict(type=kind,**payload),allow_nan=False))
            async def hold(v=0.,omega=0.,duration=.6):
                deadline=time.monotonic()+duration
                while time.monotonic()<deadline:
                    await send('drive',v=v,omega=omega);await asyncio.sleep(.08)
            async def stopped(label):
                await until(lambda:'command' in latest and abs(latest['command'].linear.x)<1e-6
                    and abs(latest['command'].angular.z)<1e-6,label,2.)
            async def operation(kind,**payload):
                request_id=f'live-{kind}-{time.time_ns()}'
                await send(kind,request_id=request_id,**payload)
                await until(lambda:any((packet.get('operation') or {}).get('request_id')==request_id
                    and packet['operation'].get('state') in ('complete','error') for _,packet in packets if packet.get('operation')),
                    f'{kind} did not return a completion',32.)
                result=next(packet['operation'] for _,packet in reversed(packets)
                    if (packet.get('operation') or {}).get('request_id')==request_id
                    and packet['operation']['state'] in ('complete','error'))
                assert result['state']=='complete',result
                return request_id

            try:
                await send('set_scan',enabled=True)
                await until(lambda:all(key in latest for key in ('odom','scan','wheels','truth','command'))
                    and latest.get('bridge',{}).get('connected'), 'Bridge/sensors/ground truth not ready')
                await until(lambda:any('robot' in packet for _,packet in packets),'Viewer has no estimated pose')
                assert time.time()-capture_time(latest['odom'])<.5,'Odometry is stale'
                assert time.time()-capture_time(latest['scan'])<.5,'LiDAR is stale'
                checks['sensors_and_viewer']='passed'
                await send('stop');await stopped('Initial stop')

                for angular,label in ((.3,'left'),(-.3,'right')):
                    before=yaw(latest['odom']);physical_before=yaw(latest['truth'])
                    await hold(omega=angular)
                    await until(lambda:angular_delta(yaw(latest['odom']),before)*angular>.008,f'{label} estimated turn failed',2.)
                    await until(lambda:angular_delta(yaw(latest['truth']),physical_before)*angular>.008,f'{label} Webots body turn failed',2.)
                    await send('stop');await stopped(f'{label} release');await asyncio.sleep(.3)
                    checks[label]='passed'
                for velocity,label in ((.15,'forward'),(-.15,'reverse')):
                    before=latest['truth'];x,y=before.pose.pose.position.x,before.pose.pose.position.y;heading=yaw(before)
                    await hold(v=velocity)
                    await until(lambda:((latest['truth'].pose.pose.position.x-x)*math.cos(heading)
                        +(latest['truth'].pose.pose.position.y-y)*math.sin(heading))*velocity>.002,
                        f'{label} Webots body movement failed',2.)
                    await send('stop');await stopped(f'{label} release');await asyncio.sleep(.3)
                    checks[label]='passed'

                if args.map_json:
                    payload=json.loads(Path(args.map_json).read_text(encoding='utf-8'))
                    await operation('nav_map',resolution=payload['resolution'],radius=payload['radius'],grid=payload['grid'],navigation_config=payload.get('navigation_config',{}))
                    await until(lambda:latest.get('mapping',{}).get('mode')=='known','Imported map was not installed')
                    checks['map_import']='passed'
                    await operation('nav_clear_map');checks['live_map_mode']='passed'
                else:checks['map_import']='not run: supply --map-json with a known occupancy map'

                grids.clear();request_id=await operation('clear_map')
                assert any(message.data and all(value==-1 for value in message.data) for message in grids),'Mapper never published an empty grid'
                await until(lambda:any(packet.get('map',{}).get('map_id')==request_id for _,packet in packets),'Viewer did not receive new map generation')
                checks['clear_map']='passed'
                await operation('reset_odom')
                await until(lambda:math.hypot(latest['odom'].pose.pose.position.x,latest['odom'].pose.pose.position.y)<.05
                    and abs(yaw(latest['odom']))<.05,'Reset did not reach the estimator')
                checks['reset_odom']='passed'

                stamp=time.monotonic()
                await send('nav_goal',x=.18,y=0.,yaw_deg=0.)
                await until(lambda:abs(latest['command'].linear.x)>.005 or abs(latest['command'].angular.z)>.01,'Target produced no motion',4.)
                await until(lambda:any(arrival>stamp and packet.get('navigation',{}).get('state')=='goal_reached' for arrival,packet in packets),'Goal did not settle',20.)
                await stopped('Goal motor output did not stop');checks['target_navigation']='passed'
                await send('nav_goal',x=.6,y=0.)
                await until(lambda:latest['command'].linear.x>.01,'Cancellation target produced no motion',4.)
                stamp=time.monotonic();await send('nav_stop');await stopped('Nav cancellation did not stop')
                cancel_delay=time.monotonic()-stamp
                assert cancel_delay<.4,f'Navigation cancellation took {cancel_delay:.3f}s'
                checks['navigation_cancel']='passed'
                await hold(v=.15,duration=.3)
                stamp=time.monotonic();await stopped('Missing browser heartbeat did not stop')
                watchdog_delay=time.monotonic()-stamp
                assert watchdog_delay<.7,f'Heartbeat stop took {watchdog_delay:.3f}s'
                checks['heartbeat_timeout']='passed'
                if args.delivery_json:
                    config=json.loads(Path(args.delivery_json).read_text(encoding='utf-8'))
                    tasks=config['tasks']
                    assert tasks and all(0<=float(task.get('delay_s',0))<=3 for task in tasks),'Use delivery test delays of 0–3 seconds'
                    request_id=f'live-zones-{time.time_ns()}'
                    await send('set_navigation_config',request_id=request_id,zones=config['zones'],base=config['base'])
                    await until(lambda:any((packet.get('navigation_config') or {}).get('request_id')==request_id for _,packet in packets),'Zones were not confirmed')
                    result=next(packet['navigation_config'] for _,packet in reversed(packets) if (packet.get('navigation_config') or {}).get('request_id')==request_id)
                    assert result['state']=='complete',result
                    stamp=time.monotonic();await send('delivery_start',tasks=tasks,return_to_base=True)
                    await until(lambda:latest.get('nav',{}).get('delivery',{}).get('state') in ('approaching','waiting'),'Delivery did not start')
                    deadline=time.monotonic()+len(tasks)*60+30;waiting={};closest=float('inf')
                    while latest.get('nav',{}).get('delivery',{}).get('state')!='complete':
                        delivery=latest['nav'].get('delivery',{})
                        assert delivery.get('state') not in ('failed','cancelled'),delivery
                        assert time.monotonic()<deadline,'Live delivery timed out'
                        p=latest['odom'].pose.pose.position
                        for zone in config['zones']:
                            separation=math.hypot(p.x-zone['x'],p.y-zone['y'])-zone['radius']
                            closest=min(closest,separation)
                            assert separation>.52,'Estimated robot footprint entered a denied circle'
                        index=delivery.get('task_index',0)
                        if delivery.get('state')=='waiting':
                            waiting.setdefault(index,time.monotonic())
                            await stopped('Delivery moved during its arrival delay')
                        for previous,arrival in list(waiting.items()):
                            if index>previous:
                                assert time.monotonic()-arrival>=float(tasks[previous].get('delay_s',0))-.15,'Task advanced before its delay'
                        await asyncio.sleep(.02)
                    await stopped('Delivery did not stop at base')
                    p=latest['odom'].pose.pose.position;base=config['base']
                    assert math.hypot(p.x-base['x'],p.y-base['y'])<.06,'Delivery did not return to base'
                    assert latest['nav']['delivery']['completed']==len(tasks)
                    checks['delivery_zones_delays_and_return']='passed'
                    checks['minimum_estimated_zone_clearance_m']=round(closest,4)
                else:checks['delivery_zones_delays_and_return']='not run: supply --delivery-json with zones, base and tasks suited to this world'
                await asyncio.sleep(2.)
                metrics={}
                for key,min_rate in (('wheels',15.),('odom',15.),('scan',5.),('command',30.)):
                    samples=times[key]
                    rate=(len(samples)-1)/(samples[-1]-samples[0])
                    assert rate>=min_rate,f'{key} rate {rate:.2f} Hz < {min_rate} Hz'
                    metrics[key+'_hz']=round(rate,2)
                arrivals=[arrival for arrival,packet in packets if packet.get('type')=='scan']
                viewer_rate=(len(arrivals)-1)/(arrivals[-1]-arrivals[0])
                assert viewer_rate>=7.,f'Viewer rate is {viewer_rate:.2f} Hz'
                metrics['viewer_hz']=round(viewer_rate,2)
                for key in ('wheels','odom','scan'):
                    values=sorted(ages[key]);p95=values[int(.95*(len(values)-1))]
                    assert 0<=p95<.3,f'{key} p95 age {p95:.3f}s'
                    metrics[key+'_age_p95_s']=round(p95,4)
                metrics.update(cancel_stop_s=round(cancel_delay,4),lost_heartbeat_stop_s=round(watchdog_delay,4))
                checks['realtime_rates_and_freshness']='passed'
                return dict(result='passed',scope='Genuine ROS DDS, WebSocket and running Webots; physical motion checked with diagnostic ground truth',checks=checks,metrics=metrics)
            finally:
                try:await send('stop')
                finally:
                    reader.cancel()
                    try:await reader
                    except asyncio.CancelledError:pass
    finally:
        running=False;await task;node.destroy_node();rclpy.shutdown()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ws-url',default='ws://127.0.0.1:8765')
    parser.add_argument('--map-json',help='Known map: resolution, radius and flat [-1,0,100] grid')
    parser.add_argument('--delivery-json',help='Live-world delivery fixture: zones, base, tasks; test delays 0–3 seconds')
    parser.add_argument('--report',default=str(Path(__file__).parent/'results/live_pipeline_validation.json'))
    args=parser.parse_args()
    try:report=asyncio.run(verify(args));code=0
    except ImportError as exc:report=dict(result='unavailable',reason=str(exc),scope='Live ROS/Webots; no substitutes used');code=2
    except Exception as exc:report=dict(result='failed',reason=str(exc),scope='Live ROS/Webots');code=1
    destination=Path(args.report);destination.parent.mkdir(parents=True,exist_ok=True)
    destination.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    return code


if __name__=='__main__':raise SystemExit(main())
