"""Portable world/configuration checks; not a Webots physics execution."""
import ast
import math
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).parents[1]


class WorldIntegrationTests(unittest.TestCase):
    def test_new_cafe_is_default_and_preserves_furniture_and_sensors(self):
        launch = (ROOT/'run_full_project.sh').read_text(encoding='utf-8')
        self.assertIn('webots/worlds/world_simulation.wbt', launch)
        world = (ROOT/'webots/worlds/world_simulation.wbt').read_text(encoding='utf-8')
        self.assertEqual(world.count('Fabtino {'), 1)
        self.assertEqual(len(re.findall(r'^Table \{', world, re.M)), 8)
        self.assertEqual(len(re.findall(r'^WoodenChair \{', world, re.M)), 16)
        for name in ('mapping_lidar', 'mapping_gyro', 'mapping_accelerometer', 'inertial unit'):
            self.assertIn('name "'+name+'"', world)
        self.assertIn('controller "fabtino_webots_bridge_controller"', world)

    def test_robot_worlds_have_mecanum_contacts_and_portable_assets(self):
        for name in ('world_fixed.wbt', 'world_simulation.wbt'):
            with self.subTest(world=name):
                file = ROOT/'webots/worlds'/name
                text = file.read_text(encoding='utf-8')
                for material in ('InteriorWheelMat', 'ExteriorWheelMat'):
                    self.assertIn('material1 "'+material+'"', text)
                self.assertIn('coordinateSystem "ENU"', text)
                self.assertNotRegex(text, r'(?m)^\s*hidden ')
                for url in re.findall(r'"([^"\n]+\.obj)"', text):
                    asset = (file.parent/url).resolve()
                    self.assertTrue(asset.is_relative_to(ROOT.resolve()))
                    self.assertTrue(asset.is_file())
                # Brackets/braces outside strings and comments must balance.
                stripped = re.sub(r'"[^"\n]*"|#[^\n]*', '', text)
                stack = []
                for char in stripped:
                    if char in '[{':
                        stack.append(char)
                    elif char in ']}':
                        self.assertEqual(stack.pop(), '[' if char == ']' else '{')
                self.assertEqual(stack, [])

    def test_wheel_radius_matches_official_r2025a_collision_geometry(self):
        # Fabtino.proto BO_WHEEL Cylinder radius=0.1, not visual mesh diameter.
        controller = ROOT/'webots/controllers/fabtino_webots_bridge_controller/fabtino_webots_bridge_controller.py'
        assignments = {}
        for node in ast.parse(controller.read_text(encoding='utf-8')).body:
            if isinstance(node, ast.Assign) and isinstance(node.value, ast.Constant):
                for target in node.targets:
                    if isinstance(target, ast.Name):
                        assignments[target.id] = node.value.value
        self.assertEqual(assignments['WHEEL_RADIUS'], .1)
        for name in ('config/robot_parameters.yaml', 'ros_ws/src/fabtino_core/fabtino_core/config/robot_parameters.yaml'):
            text = (ROOT/name).read_text(encoding='utf-8')
            self.assertEqual(float(re.search(r'wheel_radius_m:\s*([\d.]+)', text)[1]), .1)
        launch = (ROOT/'ros_ws/src/fabtino_bringup/launch/fabtino_sim.launch.py').read_text(encoding='utf-8')
        self.assertEqual(float(re.search(r"'wheel_radius_m':([\d.]+)", launch)[1]), .1)
        # The bounding corner centre plus cylinder radius needs about 0.509 m.
        radius = float(re.search(r"'robot_radius_m':([\d.]+)", launch)[1])
        self.assertGreaterEqual(radius, math.hypot(.363, .243)+.072)


if __name__ == '__main__':
    unittest.main()
