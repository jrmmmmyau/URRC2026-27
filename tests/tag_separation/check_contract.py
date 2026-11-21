#!/usr/bin/env python3
"""Check the public schema, launch connections, configuration and package boundary."""

import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml
from catkin_pkg.package import parse_package


ROOT = Path(__file__).resolve().parents[2]


def keywords(call):
    return {item.arg: item.value for item in call.keywords}


def calls(tree, name):
    return [item for item in ast.walk(tree) if isinstance(item, ast.Call)
            and isinstance(item.func, ast.Name) and item.func.id == name]


def declaration_defaults(tree):
    result = {}
    for call in calls(tree, 'DeclareLaunchArgument'):
        value = keywords(call).get('default_value')
        result[ast.literal_eval(call.args[0])] = ast.literal_eval(value) if value else None
    return result


def message_fields(name):
    text = (ROOT / 'src/perception/msg' / name).read_text()
    return [line.split('#', 1)[0].strip() for line in text.splitlines()
            if line.split('#', 1)[0].strip()]


def main():
    assert message_fields('TagObservation.msg') == ['int64 id', 'geometry_msgs/Transform transform']
    assert message_fields('TagObservationArray.msg') == [
        'std_msgs/Header header', 'TagObservation[] observations']
    for package in ['perception', 'apriltag_localization', 'start']:
        folder = ROOT / 'src' / package
        parse_package(str(folder / 'package.xml'))
        for path in (folder / 'launch').glob('*.py'):
            ast.parse(path.read_text(), filename=str(path))
        for path in (folder / 'config').glob('*.yaml'):
            yaml.safe_load(path.read_text())

    configs = {}
    for package, filename, node in [
        ('perception', 'tag_observer.yaml', 'tag_observer'),
        ('apriltag_localization', 'localization.yaml', 'tag_localizer'),
    ]:
        configs[node] = yaml.safe_load(
            (ROOT / 'src' / package / 'config' / filename).read_text())[node]['ros__parameters']
    assert configs['tag_observer']['output_frame'] == configs['tag_localizer']['base_frame'] == 'base_link'
    detector = yaml.safe_load((ROOT / 'src/perception/config/apriltag.yaml').read_text())['apriltag']['ros__parameters']
    assert detector['tag']['frames'] == [
        configs['tag_observer']['tag_frame_prefix'] + str(tag) for tag in detector['tag']['ids']]
    assert configs['tag_observer']['tf_timeout'] == 0.1
    assert configs['tag_localizer']['move_threshold'] == 0.1
    assert 'tag_frame_prefix' not in configs['tag_localizer']

    launches = [
        ('perception/launch/tag_observation.launch.py', 'tag_observer', 'output_frame'),
        ('apriltag_localization/launch/apriltag.launch.py', 'tag_localizer', 'base_frame'),
    ]
    for filename, executable, frame_parameter in launches:
        tree = ast.parse((ROOT / 'src' / filename).read_text())
        defaults = declaration_defaults(tree)
        assert defaults['base_frame'] == 'base_link'
        assert defaults['observations_topic'] == '/perception/tag/observations'
        assert defaults['use_sim_time'] == 'false'
        nodes = calls(tree, 'Node')
        assert len(nodes) == 1
        args = keywords(nodes[0])
        assert ast.literal_eval(args['executable']) == executable
        remaps = {ast.literal_eval(item.elts[0]): ast.dump(item.elts[1])
                  for item in args['remappings'].elts}
        assert remaps['tag/observations'] == ast.dump(ast.parse(
            "LaunchConfiguration('observations_topic')", mode='eval').body)
        overrides = args['parameters'].elts[1]
        values = {ast.literal_eval(k): v for k, v in zip(overrides.keys, overrides.values)}
        assert ast.dump(values[frame_parameter]) == ast.dump(ast.parse(
            "ParameterValue(LaunchConfiguration('base_frame'), value_type=str)", mode='eval').body)

    tree = ast.parse((ROOT / 'src/start/launch/apriltag_test.launch.py').read_text())
    expected = {'rs_launch.py', 'sim.launch.py', 'apriltag_detection.launch.py',
                'tag_observation.launch.py', 'apriltag.launch.py'}
    includes = calls(tree, 'IncludeLaunchDescription')
    assert len(includes) == len(expected)
    seen = set()
    for include in includes:
        filenames = [item.value for item in ast.walk(include.args[0])
                     if isinstance(item, ast.Constant) and isinstance(item.value, str)
                     and item.value.endswith('.py')]
        assert len(filenames) == 1
        filename = filenames[0]
        assert filename not in seen
        seen.add(filename)
        if filename in {'tag_observation.launch.py', 'apriltag.launch.py'}:
            arguments = keywords(include)['launch_arguments'].func.value
            arguments = {ast.literal_eval(k): v for k, v in zip(arguments.keys, arguments.values)}
            for name in ['base_frame', 'observations_topic']:
                assert ast.dump(arguments[name]) == ast.dump(ast.parse(
                    'LaunchConfiguration(' + repr(name) + ')', mode='eval').body)
    assert seen == expected

    localizer = (ROOT / 'src/apriltag_localization/src/tag_localizer.cpp').read_text()
    for token in ['lookupTransform', 'tf2_ros', 'AprilTagDetectionArray', 'tag_frame_prefix']:
        assert token not in localizer, token
    assert 'map_to_tag * base_to_tag.inverse()' in localizer
    graph = {}
    for path in (ROOT / 'src').glob('*/package.xml'):
        package = ET.parse(path).getroot()
        graph[package.findtext('name')] = {item.text for item in package
                                         if item.tag.endswith('depend')}
    assert 'perception' in graph['apriltag_localization']
    assert not {'apriltag_msgs', 'tf2_ros'} & graph['apriltag_localization']
    assert not {'mapping', 'apriltag_localization'} & graph['perception']

    def visit(name, active):
        assert name not in active, 'Dependency cycle: ' + name
        for dependency in graph[name] & graph.keys():
            visit(dependency, active | {name})
    for package in graph:
        visit(package, set())
    print('PASS: public message schema, tag frames, topic remapping, and single startup composition.')
    print('PASS: affected manifests, YAML/Python syntax, consumer boundary, and acyclic package dependencies.')
    print('Scope: static contract checks; generated ROS messages and launch execution require ROS.')


if __name__ == '__main__':
    main()
