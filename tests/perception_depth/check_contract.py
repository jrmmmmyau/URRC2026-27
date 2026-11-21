#!/usr/bin/env python3
"""Check the standalone camera-depth interface and removal of world mode."""

import ast
from pathlib import Path
import xml.etree.ElementTree as ET

from catkin_pkg.package import parse_package
import yaml


ROOT = Path(__file__).resolve().parents[2]


def launch(path):
    tree = ast.parse(path.read_text())
    calls = [item for item in ast.walk(tree) if isinstance(item, ast.Call)
             and isinstance(item.func, ast.Name)]
    defaults = {}
    for call in calls:
        if call.func.id == 'DeclareLaunchArgument':
            value = next((k.value for k in call.keywords if k.arg == 'default_value'), None)
            if isinstance(value, ast.Constant):
                defaults[ast.literal_eval(call.args[0])] = value.value
    node = next(call for call in calls if call.func.id == 'Node')
    remaps = next(k.value for k in node.keywords if k.arg == 'remappings')
    connections = {ast.literal_eval(p.elts[0]): ast.literal_eval(p.elts[1].args[0])
                   for p in remaps.elts}
    return defaults, connections


def main():
    for package in ['perception', 'apriltag_localization']:
        directory = ROOT / 'src' / package
        parse_package(str(directory / 'package.xml'))
        for path in (directory / 'launch').glob('*.py'):
            ast.parse(path.read_text())
        for path in (directory / 'config').glob('*.yaml'):
            yaml.safe_load(path.read_text())
    p, pr = launch(ROOT / 'src/perception/launch/depth_observation.launch.py')
    assert p['use_sim_time'] == 'false'
    assert p['points_topic'] == '/perception/depth/points'
    assert pr == {'depth/image': 'depth_image_topic', 'depth/camera_info': 'camera_info_topic',
                  'depth/points': 'points_topic'}
    for option in ['output_mode', 'world_frame', 'base_frame', 'sensor_frame', 'pose_topic', 'observations_topic']:
        assert option not in p
    source = (ROOT / 'src/perception/src/depth_observer.cpp').read_text()
    for removed in ['PoseHistory', 'on_pose', 'process_pending', 'lookupTransform',
                    'tf2_ros', 'DepthObservation', 'world_frame', 'localization/pose']:
        assert removed not in source, removed
    assert 'write_observation_cloud(points, msg->header, cloud)' in source
    assert not (ROOT / 'src/perception/include/perception/world_projection.hpp').exists()
    assert not (ROOT / 'src/perception/msg/DepthObservation.msg').exists()
    build = (ROOT / 'src/perception/CMakeLists.txt').read_text()
    assert 'DepthObservation.msg' not in build
    assert 'ament_target_dependencies(depth_observer rclcpp sensor_msgs cv_bridge)' in build
    assert 'target_link_libraries(depth_observer ${OpenCV_LIBRARIES})' in build
    xml = ET.parse(ROOT / 'src/perception/package.xml').getroot()
    dependencies = {n.text for n in xml if n.tag.endswith('depend')}
    assert not {'mapping', 'apriltag_localization'} & dependencies
    assert not (ROOT / 'src/mapping').exists(), 'Removed mapper unexpectedly restored'
    print('PASS: camera depth topics, original header, package declarations and removal of world/mapping additions.')
    print('Scope: static declarations and source checks, not ROS interface generation or actual launch.')


if __name__ == '__main__':
    main()
