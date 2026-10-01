"""Webots-only adapter for the ROS 2 system.

This controller deliberately contains no localization, mapping, planning,
browser, or WebSocket-server logic. It exposes Webots sensors over a small
newline-delimited JSON TCP protocol and accepts final /cmd_vel commands.
ROS-side safety is authoritative; this controller keeps only a local
receipt-time command timeout as the last-resort motor stop.
"""
from __future__ import annotations
import json, math, queue, socket, threading, time
from controller import Supervisor
from lidar_mask import DEFAULT_CONFIG_PATH, load_mask

HOST = "127.0.0.1"
PORT = 8766
COMMAND_TIMEOUT_S = 0.40
SENSOR_QUEUE_DEPTH = 4
WHEEL_RADIUS = 0.1000
HALF_LENGTH = 0.284979
HALF_WIDTH = 0.236938
K = HALF_LENGTH + HALF_WIDTH
MAX_V = 0.5
MAX_W = 1.5
LIDAR_SUBSAMPLE = 1

def finite(v, default=0.0):
    try:
        v = float(v)
        return v if math.isfinite(v) else default
    except Exception:
        return default


def encode_sensor_packet(packet):
    # JSON has no infinity literal. Preserve every range slot with null on the
    # wire; the ROS bridge restores +inf before publishing the LaserScan.
    wire = dict(packet)
    if 'lidar' in packet:
        wire['lidar'] = dict(packet['lidar'])
        wire['lidar']['ranges'] = [None if value == math.inf else finite(value, -1.0)
                                   for value in packet['lidar']['ranges']]
    return (json.dumps(wire, separators=(',', ':'), allow_nan=False)+'\n').encode()

class BridgeTransport:
    def __init__(self, host=HOST, port=PORT):
        self.host,self.port=host,port
        self.ready=threading.Event()
        self.incoming = queue.Queue(maxsize=32)
        self.outgoing = queue.Queue(maxsize=SENSOR_QUEUE_DEPTH)
        self.stop_event = threading.Event()
        self.sock = None
        self.client = None
        self.thread = threading.Thread(target=self._thread, daemon=True)

    def start(self):
        self.thread.start()

    def send(self, packet):
        try:
            self.outgoing.put_nowait(packet)
        except queue.Full:
            try: self.outgoing.get_nowait()
            except queue.Empty: pass
            try: self.outgoing.put_nowait(packet)
            except queue.Full: pass

    def commands(self):
        items=[]
        while True:
            try: items.append(self.incoming.get_nowait())
            except queue.Empty: return items

    def _thread(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.host, self.port)); srv.listen(1); srv.settimeout(0.5)
        self.sock = srv
        self.port=srv.getsockname()[1]
        self.ready.set()
        print(f"[BRIDGE] TCP endpoint listening on {self.host}:{self.port}")
        while not self.stop_event.is_set():
            try: conn, addr = srv.accept()
            except socket.timeout: continue
            except OSError: break
            conn.settimeout(0.02); self.client = conn
            print(f"[BRIDGE] ROS bridge connected from {addr}")
            try:
                self._client_loop(conn)
            except Exception as exc:
                print(f"[BRIDGE] connection error: {exc}")
            finally:
                try: conn.close()
                except Exception: pass
                self.client = None
                print("[BRIDGE] ROS bridge disconnected")
        try: srv.close()
        except Exception: pass

    def _client_loop(self, conn):
        rx = b""
        while not self.stop_event.is_set():
            try: data = conn.recv(65536)
            except socket.timeout: data = None
            except OSError: return
            if data == b"":
                return
            if data:
                rx += data
                while b"\n" in rx:
                    line, rx = rx.split(b"\n",1)
                    if not line: continue
                    try: msg=json.loads(line.decode('utf-8'))
                    except Exception: continue
                    try: self.incoming.put_nowait(msg)
                    except queue.Full:
                        try: self.incoming.get_nowait()
                        except queue.Empty: pass
                        self.incoming.put_nowait(msg)
            # Drain latest outbound packet(s), newest packet wins.
            latest=None
            while True:
                try: latest=self.outgoing.get_nowait()
                except queue.Empty: break
            if latest is not None:
                raw=encode_sensor_packet(latest)
                try: conn.sendall(raw)
                except OSError: return
            time.sleep(0.001)

    def close(self):
        self.stop_event.set()
        try:
            if self.sock: self.sock.close()
            if self.client:self.client.close()
        except Exception: pass
        if self.thread.is_alive():self.thread.join(timeout=1.)

class Controller:
    def __init__(self):
        self.robot=Supervisor(); self.dt=int(self.robot.getBasicTimeStep())
        self.self_node=self.robot.getSelf()
        names={
            'fl':'front_left_wheel_joint','fr':'front_right_wheel_joint',
            'rl':'back_left_wheel_joint','rr':'back_right_wheel_joint'}
        self.motors={}
        for k,n in names.items():
            m=self.robot.getDevice(n)
            if m is None: raise RuntimeError(f'Missing motor {n}')
            m.setPosition(float('inf')); m.setVelocity(0.0); self.motors[k]=m
        self.encoders={}
        for k,n in [('fl','front_left_wheel_joint_sensor'),('fr','front_right_wheel_joint_sensor'),
                    ('rl','back_left_wheel_joint_sensor'),('rr','back_right_wheel_joint_sensor')]:
            s=self.robot.getDevice(n)
            if s is None: raise RuntimeError(f'Missing encoder {n}')
            s.enable(self.dt); self.encoders[k]=s
        self.lidar=self.robot.getDevice('mapping_lidar')
        if self.lidar is None: raise RuntimeError('Missing mapping_lidar')
        self.lidar.enable(self.dt)
        self.config_path = DEFAULT_CONFIG_PATH
        self.reload_lidar_mask()  # Invalid YAML/mask stops startup before transport.
        self.imu=self.robot.getDevice('inertial unit')
        self.gyro=self.robot.getDevice('mapping_gyro')
        self.accel=self.robot.getDevice('mapping_accelerometer')
        for d in (self.imu,self.gyro,self.accel):
            if d: d.enable(self.dt)
        self.transport=BridgeTransport(); self.transport.start()
        self.scanning=True; self.last_cmd_mono=time.monotonic(); self.last_cmd=(0.0,0.0); self.seq=0
        self.reset_id=None
        print('='*72); print(' Fabtino Webots ↔ ROS 2 BRIDGE CONTROLLER'); print('='*72)
        print(f' Webots step: {self.dt} ms | TCP: {HOST}:{PORT}')
        print(f' LiDAR: {self.lidar.getHorizontalResolution()} rays / {self.lidar.getFov():.3f} rad')
        print(f' IMU: {bool(self.imu)} | Gyro: {bool(self.gyro)} | Accel: {bool(self.accel)}')
        print('='*72)

    def reload_lidar_mask(self):
        # Build first, then replace atomically. A failed reload raises and keeps
        # the previous mask; callers must not silently ignore that error.
        mask = load_mask(self.lidar, self.config_path)
        self.lidar_mask = mask
        print(f'[LIDAR] Masked {len(mask.masked_indices)}/{mask.resolution} horizontal rays '
              f'across {mask.layers} layer(s), config: {self.config_path}')

    def apply_commands(self):
        for msg in self.transport.commands():
            typ=msg.get('type')
            if typ=='cmd_vel':
                issued_ns=msg.get('issued_time_ns')
                if issued_ns is not None and (time.time_ns()-int(issued_ns))*1e-9>COMMAND_TIMEOUT_S:
                    self.last_cmd=(0.0,0.0)
                    continue
                self.last_cmd=(max(-MAX_V,min(MAX_V,finite(msg.get('v')))),
                               max(-MAX_W,min(MAX_W,finite(msg.get('omega')))))
                self.last_cmd_mono=time.monotonic()
            elif typ=='stop':
                self.last_cmd=(0.0,0.0); self.last_cmd_mono=time.monotonic()
            elif typ=='scan_enable':
                self.scanning=bool(msg.get('enabled',True))
            elif typ=='reset':
                self.last_cmd=(0.0,0.0); self.last_cmd_mono=time.monotonic()
                self.reset_id=msg.get('request_id',str(time.time_ns()))
                self.send_sensor(reset=True)

    def drive(self,v,w):
        if time.monotonic()-self.last_cmd_mono>COMMAND_TIMEOUT_S: v=w=0.0
        v=max(-MAX_V,min(MAX_V,finite(v))); w=max(-MAX_W,min(MAX_W,finite(w)))
        fl=(v-K*w)/WHEEL_RADIUS; fr=(v+K*w)/WHEEL_RADIUS
        self.motors['fl'].setVelocity(fl); self.motors['fr'].setVelocity(fr)
        self.motors['rl'].setVelocity(fl); self.motors['rr'].setVelocity(fr)

    def gt(self):
        try:
            t=self.self_node.getField('translation').getSFVec3f(); r=self.self_node.getField('rotation').getSFRotation()
            yaw=float(r[3] if abs(r[2])>0.9 else 0.0)
            # For this model the world yaw about +Z is represented by axis-angle;
            # use the rotation matrix from Webots to recover planar heading.
            ax,ay,az,ang=[float(v) for v in r]
            c=math.cos(ang); s=math.sin(ang); one_c=1-c
            R00=ax*ax*one_c+c; R10=ax*ay*one_c+az*s
            yaw=math.atan2(R10,R00)
            return [finite(t[0]),finite(t[1]),finite(t[2]),yaw]
        except Exception:
            return None

    def sensor_packet(self):
        t=self.robot.getTime(); stamp_ns=time.time_ns()
        wheels={k:finite(s.getValue()) for k,s in self.encoders.items()}
        rpy=self.imu.getRollPitchYaw() if self.imu else (0.0,0.0,0.0)
        gyro=self.gyro.getValues() if self.gyro else (0.0,0.0,0.0)
        accel=self.accel.getValues() if self.accel else (0.0,0.0,0.0)
        ranges=[]
        if self.scanning:
            ranges=self.lidar_mask.filter_scan(self.lidar.getRangeImage())
        gt=self.gt()
        self.seq+=1
        return {'type':'sensors','seq':self.seq,'sim_time':t,'wall_time_ns':stamp_ns,
                'reset_id':getattr(self,'reset_id',None),
                'wheels':wheels,'imu':{'roll':finite(rpy[0]),'pitch':finite(rpy[1]),'yaw':finite(rpy[2]),
                'angular_velocity':[finite(x) for x in gyro],'acceleration':[finite(x) for x in accel]},
                'lidar':{'enabled':self.scanning,'ranges':ranges,'min_range':finite(self.lidar.getMinRange()),
                          'max_range':finite(self.lidar.getMaxRange()),'fov':finite(self.lidar.getFov()),
                          'resolution':self.lidar_mask.resolution, 'layers':self.lidar_mask.layers},
                'ground_truth':gt,'lidar_roll_rad':0.0}

    def send_sensor(self, reset=False):
        p=self.sensor_packet(); p['reset']=bool(reset); self.transport.send(p)

    def run(self):
        period=max(0.016, self.dt/1000.0)
        while self.robot.step(self.dt)!=-1:
            self.apply_commands(); self.drive(*self.last_cmd)
            self.send_sensor()
        self.transport.close()

if __name__=='__main__': Controller().run()
