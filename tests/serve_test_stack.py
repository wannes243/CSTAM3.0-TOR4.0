"""Serve the actual viewer/ROS adapters with test devices for Chrome E2E.

No ROS installation is needed. DDS and Webots physics are replaced explicitly.
Write 'stop\n' to stdin for a clean shutdown.
"""
from http.server import SimpleHTTPRequestHandler,ThreadingHTTPServer
import json
from pathlib import Path
import struct
import sys
import threading
import time
import zlib

ROOT=Path(__file__).parents[1]
sys.path.insert(0,str(ROOT/'tests'))
for folder in (ROOT/'ros_ws/src').iterdir():
    if folder.is_dir():sys.path.insert(0,str(folder))
import ros_sim as ros
from robot_sim import controller_module,RobotDevices,known_map


def png(payload):
    size=round(2*payload['radius']/payload['resolution'])
    rows=[]
    for gy in reversed(range(size)):
        row=bytes(0 if value==100 else 255 if value==0 else 127 for value in payload['grid'][gy*size:(gy+1)*size])
        rows.append(b'\0'+row)
    def chunk(kind,data):
        return struct.pack('>I',len(data))+kind+data+struct.pack('>I',zlib.crc32(kind+data)&0xffffffff)
    return b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',size,size,8,0,0,0,0))+chunk(b'IDAT',zlib.compress(b''.join(rows)))+chunk(b'IEND',b'')


def main():
    ros.install()
    module=controller_module();transport=module.BridgeTransport(port=0);transport.start()
    assert transport.ready.wait(2.)
    bus=ros.Bus({'viewer_gateway':{'host':'127.0.0.1','port':0},'webots_bridge':{'port':transport.port},
                 'mapping':{'radius_m':6.,'resolution_m':.1}},realtime=True)
    nodes={key:kind() for key,kind in ros.adapters().items()}
    robot=RobotDevices(module,transport);stop=threading.Event()
    commands=[];original_command=nodes['viewer'].cmd
    def command(kind,payload):
        commands.append(dict(type=kind,request_id=payload.get('request_id')))
        return original_command(kind,payload)
    nodes['viewer'].cmd=command
    class Handler(SimpleHTTPRequestHandler):
        def __init__(self,*args,**kwargs):super().__init__(*args,directory=str(ROOT),**kwargs)
        def log_message(self,*args):pass
        def do_GET(self):
            if self.path=='/__test_state':
                body=json.dumps(dict(v=robot.v,w=robot.w,x=robot.x,y=robot.y,yaw=robot.yaw,
                    pose=list(nodes['localization'].ekf.pose),mode=nodes['mapping'].mode,
                    map_id=nodes['mapping'].map_id,errors=bus.errors,
                    safety=nodes['viewer'].latest.get('safety'),nav=nodes['viewer'].latest.get('nav'),
                    operation=nodes['viewer'].latest.get('operation'))).encode()
                body=json.dumps(dict(json.loads(body),commands=commands[-6:],
                    config=nodes['mission'].config,
                    viewer_config=nodes['viewer'].latest.get('navigation_config'),
                    clients=len(nodes['viewer'].ws_clients),
                    requests=[json.loads(msg.data) for msg in nodes['viewer'].pub_req.messages[-3:]])).encode()
                content_type='application/json'
            elif self.path=='/__test_map.png':body=png(known_map());content_type='image/png'
            else:return super().do_GET()
            self.send_response(200);self.send_header('Content-Type',content_type)
            self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
    server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
    http_thread=threading.Thread(target=server.serve_forever,daemon=True);http_thread.start()
    threading.Thread(target=lambda:(sys.stdin.readline(),stop.set()),daemon=True).start()
    try:
        assert nodes['viewer'].server_ready.wait(2.)
        print('READY '+json.dumps(dict(http_port=server.server_port,ws_port=nodes['viewer'].server_port)),flush=True)
        last=time.monotonic()
        while not stop.is_set():
            now=time.monotonic()
            if now-last>=.02:robot.step(now-last);last=now
            bus.tick();stop.wait(.005)
    finally:
        server.shutdown();server.server_close();http_thread.join(timeout=1.)
        nodes['viewer'].close();nodes['bridge'].close();transport.close()
        executor=nodes['localization'].operation_executor
        if executor is not None:executor.shutdown(wait=True,cancel_futures=True)


if __name__=='__main__':main()
