"""Signed turns through the production gateway, safety and both bridges.

ROS transport and Webots motor devices are substituted; this is not a physics test.
"""
import importlib.util
from pathlib import Path
import sys
import time
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

import test_navigation_flow as navigation_support
import test_safety_arbitration as safety_support
import test_rotation_mapping as rotation_support

ROOT = Path(__file__).parents[1]


class DriveChainTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        navigation_support.NavigationFlowTests.setUpClass()
        safety_support.SafetyArbitrationTests.setUpClass()
        folder = ROOT/'webots/controllers/fabtino_webots_bridge_controller'
        sys.path.insert(0, str(folder))
        controller = ModuleType('controller')
        controller.Supervisor = NS
        with patch.dict(sys.modules, {'controller': controller}):
            spec = importlib.util.spec_from_file_location('_drive_motors', folder/'fabtino_webots_bridge_controller.py')
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
        cls.Controller = module.Controller

    def test_left_and_right_reach_all_four_motors_and_release_stops(self):
        for angular in (1.5, -1.5):
            with self.subTest(angular=angular):
                safety_test = safety_support.SafetyArbitrationTests()
                safety_test.setUp()
                safety = safety_test.node
                gateway = object.__new__(navigation_support.NavigationFlowTests.Gateway)
                gateway.pub_teleop = navigation_support.Publisher()
                gateway.get_clock = safety.get_clock
                safety_test.output(10.)
                bridge = object.__new__(rotation_support.RotationMappingTests.Bridge)
                packets = []
                bridge.send = packets.append
                motors = {name: [] for name in ('fl', 'fr', 'rl', 'rr')}
                robot = object.__new__(self.Controller)
                robot.last_cmd_mono = time.monotonic()
                robot.motors = {name: NS(setVelocity=values.append) for name, values in motors.items()}
                robot.transport = NS(commands=lambda: packets)
                for requested in (angular, 0.):
                    packets.clear()
                    gateway.cmd('drive', {'v': 0., 'omega': requested})
                    safety.teleop_cb(gateway.pub_teleop.messages[-1])
                    safety.loop()
                    bridge.cmd_cb(safety.pub.messages[-1])
                    robot.apply_commands()
                    robot.drive(*robot.last_cmd)
                    if requested:
                        self.assertLess(motors['fl'][-1]*requested, 0.)
                        self.assertLess(motors['rl'][-1]*requested, 0.)
                        self.assertGreater(motors['fr'][-1]*requested, 0.)
                        self.assertGreater(motors['rr'][-1]*requested, 0.)
                    else:
                        self.assertTrue(all(values[-1] == 0. for values in motors.values()))

    def test_clear_map_reaches_ros_and_discards_stale_display_data(self):
        gateway = object.__new__(navigation_support.NavigationFlowTests.Gateway)
        gateway.pub_clear = navigation_support.Publisher()
        gateway.pub_operation = navigation_support.Publisher()
        gateway.pub_req = navigation_support.Publisher()
        gateway.pub_teleop = navigation_support.Publisher()
        gateway.now = 10.
        gateway.operation = None
        gateway.latest = {'map': NS(), 'cloud': NS(), 'odom': NS()}
        gateway.last_map_sent = 10.
        messages = ModuleType('std_msgs.msg')
        messages.Empty = messages.String = NS
        with patch.dict(sys.modules, {'std_msgs.msg': messages}):
            gateway.cmd('clear_map', {})
        self.assertEqual(len(gateway.pub_operation.messages), 1)
        self.assertEqual(gateway.latest['operation']['state'], 'pending')
        request_id = gateway.operation['request_id']
        import json
        gateway.operation_cb(NS(data=json.dumps(dict(node='mapping', request_id=request_id,
                                                     state='complete',stamp_ns=10_000_000_000))))
        self.assertNotIn('map',gateway.latest)
        self.assertNotIn('cloud',gateway.latest)
        self.assertEqual(gateway.latest['operation']['state'], 'complete')
        self.assertEqual(gateway.last_map_sent, 0.)

    def test_lidar_button_state_follows_scan_enable_after_first_scan(self):
        gateway = object.__new__(navigation_support.NavigationFlowTests.Gateway)
        gateway.pub_scan_enable = navigation_support.Publisher()
        gateway.latest = {'odom': rotation_support.odometry(), 'scan': NS()}
        gateway.cloud_sequence = gateway.last_cloud_sequence_sent = 0
        messages = ModuleType('std_msgs.msg')
        messages.Bool = NS
        with patch.dict(sys.modules, {'std_msgs.msg': messages}):
            for enabled in (False, True, False):
                gateway.cmd('set_scan', {'enabled': enabled})
                self.assertEqual(gateway.packet()['scanning'], enabled)


if __name__ == '__main__':
    unittest.main()
