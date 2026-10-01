"""A blocked robot's spinning encoders must not move its map/browser pose.

Production localization, mapping and viewer callbacks; only ROS I/O is
substituted. These tests do not run Webots physics or DDS.
"""
import importlib.util
import math
from pathlib import Path
import struct
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

import test_rotation_mapping as support

ROOT=Path(__file__).parents[1]


class WallContactTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        support.RotationMappingTests.setUpClass()
        cls.modules=support.RotationMappingTests.modules.copy()
        cls.modules['websockets']=ModuleType('websockets')
        cls.modules['geometry_msgs.msg'].TwistStamped=NS
        with patch.dict(sys.modules,cls.modules):
            spec=importlib.util.spec_from_file_location('_wall_viewer',
                ROOT/'ros_ws/src/fabtino_viewer/fabtino_viewer/viewer_gateway.py')
            module=importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            cls.Gateway=module.ViewerGateway

    def setUp(self):
        context=patch.dict(sys.modules,self.modules)
        context.start()
        self.addCleanup(context.stop)
        self.node=support.RotationMappingTests.Localization()
        self.epoch=1_790_727_188_000_000_000

    def sample(self,step,wheel_distance,scan_distance=None,angle=math.pi,order='scan_first'):
        ns=self.epoch+step*20_000_000
        self.node.now_ns=ns
        wheels=NS(header=NS(stamp=support.stamp(ns)),position=[wheel_distance/.1]*4)
        callbacks=[(self.node.wheel_cb,wheels),(self.node.imu_cb,support.imu(ns))]
        if scan_distance is not None:
            observation=support.scan(ns,distance=scan_distance,angle=angle)
            if order=='scan_first':
                callbacks.insert(0,(self.node.scan_cb,observation))
            else:
                callbacks.append((self.node.scan_cb,observation))
        for callback,message in callbacks:
            callback(message)
        return ns

    def test_holding_reverse_at_wall_for_20_seconds_keeps_map_and_viewer_still(self):
        self.sample(0,0.,.44)
        mapping=support.RotationMappingTests.Mapping()
        mapping.lidar_subsample=1
        mapping.publish_grid=lambda *a:None
        mapping.publish_local=lambda:None
        gateway=object.__new__(self.Gateway)
        gateway.latest={}
        gateway.cloud_sequence=gateway.last_cloud_sequence_sent=0
        for step in range(1,1001):
            # Encoders claim 0.5 m/s in reverse while the actual scan is fixed.
            ns=self.sample(step,-.01*step,.44 if step%5==0 else None)
            odom=self.node.pub.messages[-1]
            gateway.latest['odom']=odom
            packet=gateway.packet()
            self.assertAlmostEqual(packet['robot']['x'],0.,places=10)
            self.assertAlmostEqual(packet['robot']['y'],0.,places=10)
            self.assertEqual(odom.twist.twist.linear.x,0.)
            mapping.pose_cb(odom)
            if step%5==0:
                mapping.scan_cb(support.scan(ns,distance=.44,angle=math.pi))
                mapping.update()
        self.assertEqual(len(mapping.pub_points.messages),200)
        self.assertTrue(any(cloud.data for cloud in mapping.pub_points.messages))
        for cloud in mapping.pub_points.messages:
            if cloud.header.stamp.sec*1_000_000_000+cloud.header.stamp.nanosec<=self.epoch+3_100_000_000:
                self.assertFalse(cloud.data)
        for cloud in mapping.pub_live_points.messages:
            x,y,_=struct.unpack('<fff',cloud.data)
            self.assertAlmostEqual(x,-.44,places=6)
            self.assertAlmostEqual(y,0.,places=6)

    def test_moving_away_does_not_replay_discarded_wheel_rotation(self):
        self.sample(0,0.,.44)
        for step in range(1,101):
            self.sample(step,-.01*step,.44)
        self.sample(101,-.99,.44)
        self.assertAlmostEqual(self.node.ekf.pose[0],.01,places=8)
        self.assertGreater(self.node.ekf.x[3],0.)

    def test_both_directions_block_only_translation_toward_the_close_obstacle(self):
        for angle,direction in ((0.,1.),(math.pi,-1.)):
            with self.subTest(angle=angle):
                self.node=support.RotationMappingTests.Localization()
                self.sample(0,0.,.44,angle)
                self.sample(1,direction*.01,.44,angle)
                self.assertAlmostEqual(self.node.ekf.pose[0],0.)
                self.sample(2,0.,.44,angle)
                self.assertAlmostEqual(self.node.ekf.pose[0],-direction*.01,places=8)

    def test_side_obstacles_do_not_prevent_normal_reverse_odometry(self):
        self.sample(0,0.,.44,math.pi/2)
        self.sample(1,-.01,.44,math.pi/2)
        self.assertAlmostEqual(self.node.ekf.pose[0],-.01,places=8)

    def test_a_stale_scan_does_not_freeze_a_new_motion_sample(self):
        self.sample(0,0.,.44)
        self.sample(30,-.01)
        self.assertAlmostEqual(self.node.ekf.pose[0],-.01,places=8)

    def test_invalid_ranges_do_not_freeze_odometry(self):
        for distance in (.01,-1.,float('nan'),float('inf'),13.):
            with self.subTest(distance=distance):
                self.node=support.RotationMappingTests.Localization()
                self.sample(0,0.,distance)
                self.sample(1,-.01,distance)
                self.assertAlmostEqual(self.node.ekf.pose[0],-.01,places=8)

    def test_scan_order_after_fusion_still_blocks_from_recent_prior_scan(self):
        self.sample(0,0.,.44,order='scan_last')
        for step in range(1,101):
            self.sample(step,-.01*step,.44,order='scan_last')
        self.assertAlmostEqual(self.node.ekf.pose[0],0.,places=8)


if __name__=='__main__':
    unittest.main()
