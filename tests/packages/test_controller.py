import json
import socket
import threading
import time

from robot_sim import controller_module,RobotDevices


def test_transport_eof_releases_connection_instead_of_busy_looping():
    module=controller_module();transport=module.BridgeTransport(port=0)
    left,right=socket.socketpair();left.settimeout(.02)
    worker=threading.Thread(target=transport._client_loop,args=(left,))
    worker.start();right.close();worker.join(timeout=.3)
    try:assert not worker.is_alive()
    finally:transport.stop_event.set();left.close();worker.join(timeout=.5)


def test_controller_rejects_delayed_commands_and_its_local_watchdog_stops():
    module=controller_module();transport=module.BridgeTransport(port=0)
    robot=RobotDevices(module,transport);node=robot.controller
    transport.incoming.put({'type':'cmd_vel','v':.3,'omega':.2,'issued_time_ns':time.time_ns()-1_000_000_000})
    robot.step(.02)
    assert all(value==0. for value in robot.velocities.values())
    transport.incoming.put({'type':'cmd_vel','v':.3,'omega':0.})
    robot.step(.02)
    assert all(value>0. for value in robot.velocities.values())
    node.last_cmd_mono-=.5;robot.step(.02)
    assert all(value==0. for value in robot.velocities.values())


def test_reset_identity_survives_latest_sensor_packet_coalescing():
    module=controller_module();transport=module.BridgeTransport(port=0)
    robot=RobotDevices(module,transport)
    transport.incoming.put({'type':'reset','request_id':'persistent-reset'})
    for _ in range(20):robot.step(.02)
    latest=None
    while not transport.outgoing.empty():latest=transport.outgoing.get_nowait()
    assert latest['reset_id']=='persistent-reset'
    assert latest['reset'] is False


def test_controller_full_command_queue_retains_newest_stop():
    module=controller_module();transport=module.BridgeTransport(port=0)
    left,right=socket.socketpair();left.settimeout(.02)
    worker=threading.Thread(target=transport._client_loop,args=(left,));worker.start()
    try:
        commands=[{'type':'cmd_vel','v':.2} for _ in range(100)]+[{'type':'stop'}]
        right.sendall(''.join(json.dumps(command)+'\n' for command in commands).encode())
        deadline=time.monotonic()+1.
        while transport.incoming.qsize()!=32 and time.monotonic()<deadline:time.sleep(.005)
        assert transport.commands()[-1]['type']=='stop'
    finally:
        transport.stop_event.set();right.close();left.close();worker.join(timeout=.5)
