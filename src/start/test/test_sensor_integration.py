"""Offline wiring checks; actual Xacro expansion also runs when xacro is installed."""

import ast
from pathlib import Path
import shutil
import subprocess
import unittest
from xml.etree import ElementTree as ET


ROOT = Path(__file__).resolve().parents[3]
LAUNCH = ROOT / 'src/start/launch'
URDF = ROOT / 'src/description/urdf/basic_robot.urdf'
WORLDS = ROOT / 'map/worlds'
LAUNCHERS = (
    ('teleop.launch.py', 'lunabotics_arena'),
    ('terrain_teleop.launch.py', 'lunabotics_terrain_arena'),
)
XACRO = '{http://www.ros.org/wiki/xacro}'


def node(tree, variable):
    return next(
        item.value for item in ast.walk(tree)
        if isinstance(item, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == variable
                for target in item.targets)
    )


def keyword(call, name):
    return next(item.value for item in call.keywords if item.arg == name)


def parameters(call):
    mapping = keyword(call, 'parameters').elts[0]
    return {ast.literal_eval(key): value
            for key, value in zip(mapping.keys, mapping.values)}


class SensorIntegrationTests(unittest.TestCase):
    def test_all_launch_files_have_valid_python(self):
        for path in LAUNCH.glob('*.py'):
            with self.subTest(launch=path.name):
                ast.parse(path.read_text(), filename=str(path))

    def test_map_launchers_expand_xacro_and_use_sim_time(self):
        for filename, _ in LAUNCHERS:
            with self.subTest(launch=filename):
                tree = ast.parse((LAUNCH / filename).read_text())
                config = parameters(node(tree, 'rsp_node'))
                description = config['robot_description']
                self.assertEqual(description.func.id, 'ParameterValue')
                self.assertEqual(keyword(description, 'value_type').id, 'str')
                command = description.args[0]
                self.assertEqual(command.func.id, 'Command')
                self.assertEqual(ast.literal_eval(command.args[0].elts[0]).strip(), 'xacro')
                self.assertTrue(ast.literal_eval(config['use_sim_time']))
                driver_config = parameters(node(tree, 'driver_node'))
                self.assertTrue(ast.literal_eval(driver_config['use_sim_time']))

    def test_spawn_joint_feedback_and_clock_match_each_world(self):
        robot = ET.parse(URDF).getroot().get('name')
        self.assertEqual(robot, 'melbot')
        for filename, world in LAUNCHERS:
            with self.subTest(launch=filename):
                tree = ast.parse((LAUNCH / filename).read_text())
                spawn = ast.literal_eval(keyword(node(tree, 'spawn_node'), 'arguments'))
                self.assertEqual(spawn[spawn.index('-name') + 1], robot)
                # Retain the map's safe starting area rather than the empty-world origin.
                self.assertEqual(spawn[spawn.index('-x') + 1], '1.0')
                self.assertEqual(spawn[spawn.index('-y') + 1], '0.75')
                bridge = node(tree, 'bridge_node')
                topics = ast.literal_eval(keyword(bridge, 'arguments'))
                remaps = ast.literal_eval(keyword(bridge, 'remappings'))
                joint_topic = f'/world/{world}/model/{robot}/joint_state'
                clock_topic = f'/world/{world}/clock'
                self.assertIn(joint_topic + '@sensor_msgs/msg/JointState[gz.msgs.Model', topics)
                self.assertIn(clock_topic + '@rosgraph_msgs/msg/Clock[gz.msgs.Clock', topics)
                self.assertIn((joint_topic, '/joint_states'), remaps)
                self.assertIn((clock_topic, '/clock'), remaps)
                self.assertFalse(any('/world/empty/' in topic for topic in topics))

    def test_both_launchers_bridge_all_sensors_and_camera_control(self):
        for filename, _ in LAUNCHERS:
            with self.subTest(launch=filename):
                tree = ast.parse((LAUNCH / filename).read_text())
                topics = ast.literal_eval(keyword(node(tree, 'bridge_node'), 'arguments'))
                self.assertEqual(len(topics), len(set(topics)))
                self.assertIn('/imu@sensor_msgs/msg/Imu[gz.msgs.IMU', topics)
                self.assertIn('/cmd_vel/camera@std_msgs/msg/Float64]gz.msgs.Double', topics)
                for camera in ('front_camera', 'top_camera'):
                    for suffix, ros_type, gz_type in (
                        ('image', 'Image', 'Image'),
                        ('depth_image', 'Image', 'Image'),
                        ('camera_info', 'CameraInfo', 'CameraInfo'),
                        ('points', 'PointCloud2', 'PointCloudPacked'),
                    ):
                        self.assertIn(
                            f'/{camera}/{suffix}@sensor_msgs/msg/{ros_type}[gz.msgs.{gz_type}',
                            topics,
                        )

    def test_worlds_and_template_load_sensor_systems_once(self):
        paths = [WORLDS / 'lunabotics_arena.sdf']
        paths += [WORLDS / f'lunabotics_terrain_{number:02d}.sdf' for number in range(1, 7)]
        paths += [ROOT / 'map/templates/lunabotics_terrain_template.sdf']
        for path in paths:
            with self.subTest(world=path.name):
                world = ET.parse(path).getroot().find('world')
                imu = world.findall("./plugin[@name='gz::sim::systems::Imu']")
                sensors = world.findall("./plugin[@name='gz::sim::systems::Sensors']")
                self.assertEqual(len(imu), 1)
                self.assertEqual(imu[0].get('filename'), 'gz-sim-imu-system')
                self.assertEqual(len(sensors), 1)
                self.assertEqual(sensors[0].get('filename'), 'gz-sim-sensors-system')
                self.assertEqual(sensors[0].findtext('render_engine'), 'ogre2')
        robot = ET.parse(URDF).getroot()
        for name in ('Imu', 'Sensors'):
            self.assertIsNone(robot.find(f".//plugin[@name='gz::sim::systems::{name}']"))

    def test_terrain_preserves_ogre1_gui_but_uses_ogre2_for_rgbd(self):
        source = (LAUNCH / 'terrain_teleop.launch.py').read_text()
        self.assertIn('--render-engine-gui ogre', source)
        self.assertIn('--render-engine-server ogre2', source)
        self.assertNotIn('--render-engine ogre', source)

    def test_new_robot_contains_imu_and_both_camera_instances(self):
        robot = ET.parse(URDF).getroot()
        self.assertIsNotNone(robot.find("./link[@name='imu_link']"))
        self.assertEqual(robot.findtext("./gazebo[@reference='imu_link']/sensor/topic"), 'imu')
        includes = robot.findall(f'{XACRO}include')
        self.assertEqual([entry.get('filename') for entry in includes], ['camera.xacro'])
        camera = ET.parse(URDF.parent / includes[0].get('filename')).getroot()
        self.assertIsNotNone(camera.find(".//sensor[@type='rgbd_camera']"))
        instances = robot.findall(f'{XACRO}realsense')
        self.assertEqual({entry.get('name') for entry in instances}, {'front_camera', 'top_camera'})
        self.assertIsNotNone(robot.find("./joint[@name='top_camera_holder_joint']"))
        controller = robot.find("./gazebo/plugin[topic='/cmd_vel/camera']")
        self.assertEqual(controller.findtext('joint_name'), 'top_camera_holder_joint')

    def test_runtime_dependencies_and_description_installation(self):
        start = ET.parse(ROOT / 'src/start/package.xml').getroot()
        dependencies = {entry.text for entry in start.findall('exec_depend')}
        self.assertTrue({'xacro', 'sensor_msgs', 'std_msgs', 'rosgraph_msgs'} <= dependencies)
        description = ET.parse(ROOT / 'src/description/package.xml').getroot()
        self.assertIn('xacro', {entry.text for entry in description.findall('exec_depend')})
        cmake = (ROOT / 'src/description/CMakeLists.txt').read_text()
        self.assertIn('DIRECTORY urdf', cmake)
        self.assertIn('/../../map/worlds', cmake)
        self.assertIn('/../../map/assets', cmake)

    def test_sim_entry_point_includes_the_integrated_flat_launcher(self):
        tree = ast.parse((LAUNCH / 'sim.launch.py').read_text())
        constants = {item.value for item in ast.walk(tree) if isinstance(item, ast.Constant)}
        self.assertIn('teleop.launch.py', constants)
        self.assertNotIn('empty.sdf', constants)

    @unittest.skipUnless(shutil.which('xacro'), 'Xacro expansion requires a ROS/Xacro installation')
    def test_expanded_robot_has_valid_links_joints_and_three_sensors(self):
        expanded = subprocess.check_output(['xacro', str(URDF)], text=True)
        robot = ET.fromstring(expanded)
        links = {entry.get('name') for entry in robot.findall('link')}
        self.assertTrue({'base_link', 'chassis', 'imu_link', 'front_camera_link',
                         'top_camera_link', 'top_camera_holder_link'} <= links)
        children = []
        for joint in robot.findall('joint'):
            self.assertIn(joint.find('parent').get('link'), links)
            child = joint.find('child').get('link')
            self.assertIn(child, links)
            children.append(child)
        self.assertEqual(len(children), len(set(children)))
        self.assertEqual(links - set(children), {'base_link'})
        self.assertEqual(len(robot.findall('.//sensor')), 3)
        self.assertFalse(any(element.tag.startswith(XACRO) for element in robot.iter()))


if __name__ == '__main__':
    unittest.main()
