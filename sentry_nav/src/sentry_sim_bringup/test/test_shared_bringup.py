# Copyright 2026 diting
# Licensed under the Apache License, Version 2.0 (the "License");

"""Check the generated launch contracts without executing any ROS process."""

from copy import deepcopy
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction,
)
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.utilities import evaluate_parameters, normalize_parameters
import pytest
from sentry_sim_bringup import bringup_config
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH_FILES = {
    'dual': 'dual_sentry_simulation_launch.py',
    'navigation': 'nav2_ground_truth_launch.py',
    'lio': 'lio_navigation_launch.py',
}


def _text(context, value):
    """Resolve one launch argument without visiting its action."""
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def _context(module, overrides=None):
    """Apply declarations only; timers, nodes and includes are never executed."""
    context = LaunchContext()
    context.launch_configurations.update(overrides or {})
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    return context


@pytest.fixture
def launches(monkeypatch, tmp_path):
    """Load source launches and observe actual Node construction arguments."""
    modules = {}
    node_specs = {}
    source_paths = {}

    def package_share(name):
        if name == 'sentry_sim_bringup':
            return str(PACKAGE)
        return get_package_share_directory(name)

    def observe_node(**kwargs):
        node = Node(**kwargs)
        node_specs[id(node)] = kwargs
        return node

    for role, filename in LAUNCH_FILES.items():
        spec = importlib.util.spec_from_file_location(
            'shared_bringup_contract_' + role, PACKAGE / 'launch' / filename)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        monkeypatch.setattr(module, 'get_package_share_directory', package_share)
        monkeypatch.setattr(module, 'Node', observe_node)
        if hasattr(module, 'PythonLaunchDescriptionSource'):
            source_class = module.PythonLaunchDescriptionSource

            def observe_source(path, source_class=source_class):
                source = source_class(path)
                source_paths[id(source)] = Path(path)
                return source

            monkeypatch.setattr(module, 'PythonLaunchDescriptionSource', observe_source)
        modules[role] = module

    # Keep generated bridge/model/Nav2 files within pytest's temporary directory.
    temporary_file = modules['navigation'].tempfile.NamedTemporaryFile
    temporary_directory = modules['navigation'].tempfile.mkdtemp

    def isolated_file(*args, **kwargs):
        kwargs['dir'] = str(tmp_path)
        return temporary_file(*args, **kwargs)

    def isolated_directory(*args, **kwargs):
        kwargs['dir'] = str(tmp_path)
        return temporary_directory(*args, **kwargs)

    monkeypatch.setattr(modules['navigation'].tempfile, 'NamedTemporaryFile', isolated_file)
    monkeypatch.setattr(modules['navigation'].tempfile, 'mkdtemp', isolated_directory)
    return SimpleNamespace(modules=modules, node_specs=node_specs, source_paths=source_paths)


def _nodes(launches, actions):
    """Collect Node specifications, including those waiting in spawn timers."""
    result = []
    for action in actions:
        if isinstance(action, Node):
            result.append(launches.node_specs[id(action)])
        elif isinstance(action, TimerAction):
            result.extend(_nodes(launches, action.actions))
    return result


def _node(launches, actions, name):
    """Find exactly one named node rather than silently accepting duplicates."""
    matches = [item for item in _nodes(launches, actions) if item.get('name') == name]
    assert len(matches) == 1, (name, matches)
    return matches[0]


def _parameters(context, node):
    """Evaluate dictionary substitutions through launch_ros's own evaluator."""
    dictionaries = [item for item in node.get('parameters', []) if isinstance(item, dict)]
    result = {}
    for item in evaluate_parameters(context, normalize_parameters(dictionaries)):
        result.update(item)
    return result


def _nav_yaml(context, node, namespace):
    """Read the parameter file actually supplied to the Nav2 action."""
    files = [item for item in node['parameters'] if isinstance(item, ParameterFile)]
    assert len(files) == 1
    path = _text(context, files[0].param_file)
    with open(path, encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    if namespace:
        assert set(document) == {namespace}
        return document[namespace]
    return document


def _assert_same_navigation(single, dual, namespace, path=()):
    """Allow namespace spelling changes while rejecting all tuning differences."""
    assert type(single) is type(dual), path
    if isinstance(single, dict):
        assert single.keys() == dual.keys(), path
        for key in single:
            _assert_same_navigation(single[key], dual[key], namespace, path + (key,))
    elif isinstance(single, list):
        assert len(single) == len(dual), path
        for index, (left, right) in enumerate(zip(single, dual)):
            _assert_same_navigation(left, right, namespace, path + (index,))
    elif single != dual:
        assert isinstance(single, str), (path, single, dual)
        allowed = {namespace + '/' + single.lstrip('/'), '/' + namespace + '/' +
                   single.lstrip('/')}
        assert dual in allowed, (path, single, dual)


def test_namespace_changes_preserve_navigation_tuning(launches):
    """Changing a navigation namespace must preserve all navigation tuning."""
    module = launches.modules['navigation']
    documents = []
    for namespace in ('', 'sentry_robot1'):
        context = _context(module, {'namespace': namespace, 'spin_speed': '0'})
        actions = module._launch_setup(context)
        planner = _node(launches, actions, 'planner_server')
        params = _nav_yaml(context, planner, namespace)
        documents.append(params)
        assert params['planner_server']['ros__parameters']['GridBased']['plugin'] == (
            'tdt_yastar_planner/YastarPlanner')
        assert params['controller_server']['ros__parameters']['failure_tolerance'] == 0.6
        for name in ('local_costmap', 'global_costmap'):
            assert params[name][name]['ros__parameters']['robot_radius'] == 0.4
        navigator = params['bt_navigator']['ros__parameters']
        directories = set()
        for key in ('default_nav_to_pose_bt_xml', 'default_nav_through_poses_bt_xml'):
            generated_path = Path(navigator[key])
            directories.add(generated_path.parent)
            assert generated_path.is_file()
            generated = ET.parse(generated_path).getroot()
            original = ET.parse(PACKAGE / 'behavior_trees' / generated_path.name).getroot()
            for node in generated.iter():
                if 'robot_base_frame' in node.attrib:
                    expected = namespace + '/gimbal_yaw_fake' if namespace else 'gimbal_yaw_fake'
                    assert node.attrib['robot_base_frame'] == expected
                    node.attrib['robot_base_frame'] = 'gimbal_yaw_fake'
            assert ET.tostring(generated) == ET.tostring(original)
            if key == 'default_nav_through_poses_bt_xml':
                passed_goals = generated.find('.//RemovePassedGoals')
                assert passed_goals is not None
                assert passed_goals.attrib['robot_base_frame'] == 'gimbal_yaw_fake'
            # Generated directories are intentionally unique per launch. Their
            # contents and source filenames, rather than random paths, must match.
            navigator[key] = 'behavior_trees/' + generated_path.name
        assert len(directories) == 1
    _assert_same_navigation(documents[0], documents[1], 'sentry_robot1')


@pytest.mark.parametrize('namespace', ['', 'sentry_robot1', 'test_team/primary'])
def test_gateway_is_unique_and_cannot_feed_its_own_input(launches, namespace):
    """Autonomous commands must pass through exactly one isolated watchdog."""
    module = launches.modules['navigation']
    context = _context(module, {
        'namespace': namespace, 'sentry_name': 'custom_sentry', 'spin_speed': '0',
    })
    actions = module._launch_setup(context)
    gateways = [node for node in _nodes(launches, actions)
                if node['executable'] == 'sim_serial_gateway']
    assert len(gateways) == 1
    gateway = _parameters(context, gateways[0])
    prefix = '/' + namespace + '/' if namespace else '/'
    assert gateway['input_topic'] == prefix + 'cmd_vel_chassis'
    assert gateway['output_topic'] == '/custom_sentry/cmd_vel'
    assert gateway['input_topic'] != gateway['output_topic']
    assert gateway['timeout_ms'] == 200
    assert gateway['publish_rate_hz'] == 50.0
    assert type(gateway['idle_spin_speed']) is float
    assert gateway['idle_spin_speed'] == 0.0
    fake = _node(launches, actions, 'fake_vel_transform')
    params = _nav_yaml(context, fake, namespace)['fake_vel_transform']['ros__parameters']
    assert params['output_cmd_vel_topic'] == gateway['input_topic']
    assert params['input_cmd_vel_topic'] == prefix + 'cmd_vel_nav2_result'
    assert type(params['init_spin_speed']) is float
    assert params['init_spin_speed'] == 0.0
    assert all(not item.get('relay_cmd_vel', False)
               for item in [_parameters(context, node)
                            for node in _nodes(launches, actions)])


@pytest.mark.parametrize('namespace', ['', 'test_team/primary'])
def test_tf_sensors_and_rviz_follow_the_selected_robot(launches, namespace):
    """Global map/TF stay shared while robot frames and sensor inputs isolate."""
    module = launches.modules['navigation']
    context = _context(module, {'namespace': namespace, 'sentry_name': 'custom_sentry'})
    actions = module._launch_setup(context)
    frame_prefix = namespace + '/' if namespace else ''
    topic_prefix = '/' + frame_prefix
    static_tf = _node(launches, actions, 'map_to_odom_static_transform')
    assert static_tf['arguments'][-2:] == ['map', frame_prefix + 'odom']
    odometry = _parameters(context, _node(launches, actions, 'sim_ground_truth_odometry'))
    assert odometry['odom_input'] == '/custom_sentry/odometry_raw'
    assert odometry['odom_output'] == topic_prefix + 'odometry'
    assert odometry['parent_frame'] == frame_prefix + 'odom'
    assert odometry['child_frame'] == frame_prefix + 'chassis'
    cloud = _node(launches, actions, 'pointcloud_to_laserscan')
    assert dict(cloud['remappings']) == {
        'cloud_in': '/custom_sentry/points_raw', 'scan': topic_prefix + 'obstacle_scan'}
    params = _nav_yaml(context, cloud, namespace)
    assert params['global_costmap']['global_costmap']['ros__parameters']['global_frame'] == (
        'map')
    assert params['pointcloud_to_laserscan']['ros__parameters']['target_frame'] == (
        frame_prefix + 'gimbal_yaw_fake')
    rviz = _node(launches, actions, 'rviz2')
    assert rviz['namespace'] == namespace
    assert _parameters(context, rviz)['use_sim_time'] is True
    assert rviz['condition'].evaluate(context) is False
    context.launch_configurations['use_rviz'] = 'True'
    assert rviz['condition'].evaluate(context) is True


def test_parameter_rewrite_preserves_keys_and_shared_transport():
    """Namespace expansion must never mutate joystick keys or shared TF topics."""
    source = {
        'map_server': {'ros__parameters': {'yaml_filename': 'old.yaml'}},
        'fake_vel_transform': {'ros__parameters': {'output_cmd_vel_topic': '/cmd_vel'}},
        'bt_navigator': {'ros__parameters': {
            'default_nav_to_pose_bt_xml': '/old/custom.xml',
            'default_nav_through_poses_bt_xml': '/old/other.xml',
        }},
        'joystick': {'ros__parameters': {
            'axis_chassis': {'x': 1}, 'scale_chassis': {'x': 0.5},
            'scale_chassis_turbo': {'x': 1.0},
            'description': 'chassis and /cmd_vel are literal prose',
        }},
        'probe': {'ros__parameters': {
            'global_frame': 'map', 'robot_base_frame': 'chassis',
            'topic': ['/tf', '/tf_static', '/clock', '/obstacle_scan'],
        }},
    }
    original = deepcopy(source)
    rewritten = bringup_config.navigation_parameters(
        source, 'team/sentry', 'field.yaml', True, '/common_bt')
    assert source == original
    assert rewritten['joystick'] == original['joystick']
    navigator = rewritten['bt_navigator']['ros__parameters']
    assert navigator['default_nav_to_pose_bt_xml'] == (
        '/common_bt/navigate_to_pose_w_replanning_and_recovery.xml')
    assert navigator['default_nav_through_poses_bt_xml'] == (
        '/common_bt/navigate_through_poses_w_replanning_and_recovery.xml')
    probe = rewritten['probe']['ros__parameters']
    assert probe['global_frame'] == 'map'
    assert probe['robot_base_frame'] == 'team/sentry/chassis'
    assert probe['topic'] == ['/tf', '/tf_static', '/clock', '/team/sentry/obstacle_scan']


def test_shared_model_only_changes_transport_not_obstacle_geometry():
    """Renaming a robot must preserve every link, joint, collision and inertia."""
    source = (PACKAGE / 'models' / 'sentry_rm25' / 'model.sdf').read_text(encoding='utf-8')
    generated = bringup_config.sentry_model(source, 'custom_sentry', 'team/sentry')
    original_elements = list(ET.fromstring(source).iter())
    rewritten_elements = list(ET.fromstring(generated).iter())
    assert len(original_elements) == len(rewritten_elements)
    differences = []
    for original, rewritten in zip(original_elements, rewritten_elements):
        assert original.tag == rewritten.tag
        assert original.attrib == rewritten.attrib
        before = (original.text or '').strip()
        after = (rewritten.text or '').strip()
        if before != after:
            differences.append((original.tag, before, after))
    assert sorted(differences) == sorted([
        ('ignition_frame_id', 'lidar_link', 'team/sentry/lidar_link'),
        ('topic', '/sentry_robot1/gimbal_yaw_cmd', '/custom_sentry/gimbal_yaw_cmd'),
    ])
    template = (PACKAGE / 'config' / 'sentry_bridge.yaml.in').read_text(encoding='utf-8')
    bridge = yaml.safe_load(bringup_config.sentry_bridge(template, 'custom_sentry', 'team/sentry'))
    by_topic = {entry['ros_topic_name']: entry for entry in bridge}
    assert by_topic['/custom_sentry/cmd_vel']['gz_topic_name'] == '/custom_sentry/cmd_vel'
    assert by_topic['/custom_sentry/cmd_vel']['direction'] == 'ROS_TO_GZ'
    assert by_topic['/custom_sentry/points_raw']['gz_topic_name'] == (
        '/world/default/model/custom_sentry/link/lidar_link/sensor/lidar_3d/scan/points')
    assert by_topic['/custom_sentry/odometry_raw']['gz_topic_name'] == (
        '/custom_sentry/odometry')
    assert by_topic['/team/sentry/joint_states']['gz_topic_name'] == (
        '/world/default/model/custom_sentry/joint_state')


def _walk(actions):
    """Inspect scoped groups without executing their nodes or includes."""
    for action in actions:
        yield action
        if isinstance(action, GroupAction):
            yield from _walk(action.get_sub_entities())


def _include(launches, actions, filename):
    """Select exactly one include by its source file, without loading it."""
    matches = [action for action in _walk(actions)
               if isinstance(action, IncludeLaunchDescription)
               and launches.source_paths[id(action.launch_description_source)].name == filename]
    assert len(matches) == 1, filename
    return matches[0]


def _include_arguments(context, include):
    """Resolve the actual arguments forwarded by a scene wrapper."""
    return {_text(context, key): _text(context, value)
            for key, value in include.launch_arguments}


def test_dual_scene_owns_both_robots_and_one_navigation_stack(launches):
    """The only scene must spawn both robots without the retired wrapper."""
    dual = launches.modules['dual']
    context = _context(dual, {
        'localization': 'ground_truth', 'spin_speed': '0', 'use_rviz': 'True'})
    actions = dual._launch_setup(context)
    assert not (PACKAGE / 'launch' / 'bringup_sentry_simulation_launch.py').exists()
    groups = [action for action in actions if isinstance(action, GroupAction)]
    assert len(groups) == 1
    assert any(type(action).__name__ == 'PopLaunchConfigurations'
               for action in groups[0].get_sub_entities())
    world = _include(launches, actions, 'gazebo.launch.py')
    assert world.condition.evaluate(context) is True
    context.launch_configurations['start_simulator'] = 'False'
    assert world.condition.evaluate(context) is False
    context.launch_configurations['start_simulator'] = 'True'
    include_nav = _include(launches, actions, LAUNCH_FILES['navigation'])
    forwarded = _include_arguments(context, include_nav)
    assert forwarded['namespace'] == 'sentry_robot1'
    assert forwarded['sentry_name'] == 'sentry_robot1'
    assert forwarded['spin_speed'] == '0'
    assert forwarded['use_rviz'] == 'True'
    assert forwarded['planner_plugin'] == 'tdt_yastar_planner/YastarPlanner'
    assert Path(forwarded['params_file']) == PACKAGE / 'config' / 'nav2_params_ground_truth.yaml'
    assert Path(forwarded['bt_xml_dir']) == PACKAGE / 'behavior_trees'
    includes = [action for action in _walk(actions)
                if isinstance(action, IncludeLaunchDescription)]
    assert len(includes) == 2  # One world, one navigation component.

    scene_nodes = _nodes(launches, actions)
    assert {node['executable'] for node in scene_nodes} == {
        'sim_serial_gateway', 'parameter_bridge', 'create'}
    manual_gateways = [node for node in scene_nodes
                       if node['executable'] == 'sim_serial_gateway']
    assert len(manual_gateways) == 1
    manual = _parameters(context, manual_gateways[0])
    assert manual['input_topic'] == '/rm25_robot1/cmd_vel'
    assert manual['output_topic'] == '/rm25_robot1/cmd_vel_safe'
    assert manual['timeout_ms'] == 1000
    assert manual['idle_spin_speed'] == 0.0
    assert manual['spin_topic'] == ''
    spawns = [node for node in scene_nodes if node['executable'] == 'create']
    assert len(spawns) == 2
    by_name = {node['arguments'][node['arguments'].index('-name') + 1]: node
               for node in spawns}
    assert set(by_name) == {'sentry_robot1', 'rm25_robot1'}
    for name, expected in (
        ('sentry_robot1', ['-5.3', '3.0', '0.3', '-1.57079632679']),
        ('rm25_robot1', ['5.3', '-3.0', '0.3', '0.0']),
    ):
        arguments = by_name[name]['arguments']
        assert [arguments[arguments.index(flag) + 1] for flag in ('-x', '-y', '-z', '-Y')] == (
            expected)
    sdf = Path(by_name['sentry_robot1']['arguments'][
        by_name['sentry_robot1']['arguments'].index('-file') + 1])
    assert 'sentry_robot1/lidar_link' in [
        node.text for node in ET.parse(sdf).findall('.//ignition_frame_id')]

    bridges = [node for node in scene_nodes if node['executable'] == 'parameter_bridge']
    assert {node['name'] for node in bridges} == {'sentry_sensor_bridge', 'manual_robot_bridge'}
    topics = []
    for node in bridges:
        config = Path(_parameters(context, node)['config_file'])
        topics.extend(yaml.safe_load(config.read_text()))
    command_bridges = [entry for entry in topics if entry['direction'] == 'ROS_TO_GZ'
                       and entry['gz_topic_name'].endswith('/cmd_vel')]
    assert {(entry['ros_topic_name'], entry['gz_topic_name']) for entry in command_bridges} == {
        ('/sentry_robot1/cmd_vel', '/sentry_robot1/cmd_vel'),
        ('/rm25_robot1/cmd_vel_safe', '/rm25_robot1/cmd_vel'),
    }
    assert len(command_bridges) == 2
    assert any(entry['ros_topic_name'] == '/sentry_robot1/joint_states' for entry in topics)
    nav_context = _context(launches.modules['navigation'], forwarded)
    nav_actions = launches.modules['navigation']._launch_setup(nav_context)
    assert not any(node['executable'] == 'create' for node in _nodes(launches, nav_actions))
    gateway = _parameters(nav_context, _node(launches, nav_actions, 'sim_serial_gateway'))
    assert gateway['output_topic'] == '/sentry_robot1/cmd_vel'
    assert gateway['input_topic'] == '/sentry_robot1/cmd_vel_chassis'


@pytest.mark.parametrize('params_file', ['', '/custom/navigation.yaml'])
def test_dual_forwards_custom_params_and_clock(launches, params_file):
    """Scene refactoring must keep configuration overrides and clock propagation."""
    dual = launches.modules['dual']
    context = _context(dual, {
        'localization': 'ground_truth', 'params_file': params_file, 'use_sim_time': 'False'})
    actions = dual._launch_setup(context)
    nav = _include(launches, actions, LAUNCH_FILES['navigation'])
    arguments = _include_arguments(context, nav)
    assert arguments['params_file'] == (params_file or str(
        PACKAGE / 'config' / 'nav2_params_ground_truth.yaml'))
    assert arguments['use_sim_time'] == 'False'
    for node in _nodes(launches, actions):
        if node['executable'] in ('parameter_bridge', 'sim_serial_gateway'):
            assert _parameters(context, node)['use_sim_time'] is False


@pytest.mark.parametrize('mapping', [False, True])
def test_sensor_localization_is_independent_of_ground_truth(launches, mapping):
    """Only the reference monitor may consume simulator pose in sensor mode."""
    module = launches.modules['lio']
    context = _context(module, {'slam': str(mapping), 'spin_speed': '0'})
    actions = module._launch_setup(context)
    specs = _nodes(launches, actions)
    names = {node.get('name') for node in specs}
    assert 'sim_ground_truth_odometry' not in names
    assert 'map_to_odom_static_transform' not in names
    assert {'point_lio', 'loam_interface', 'sensor_scan_generation',
            'livox_cloud_adapter', 'base_footprint_to_chassis'} <= names
    gateway = _parameters(context, _node(launches, actions, 'sim_serial_gateway'))
    assert gateway['require_localization'] is (not mapping)
    assert gateway['localization_topic'] == '/sentry_robot1/odometry'
    lock = _parameters(context, _node(launches, actions, 'gimbal_world_lock'))
    assert lock['feedback_source'] == 'imu'
    assert lock['imu_topic'] == '/sentry_robot1/imu/data'
    adapter = _parameters(context, _node(launches, actions, 'livox_cloud_adapter'))
    assert adapter['input_topic'] == '/sentry_robot1/points_raw'
    assert adapter['output_topic'] == '/sentry_robot1/velodyne_points'
    assert adapter['target_frame'] == 'sentry_robot1/chassis'
    point_lio = _node(launches, actions, 'point_lio')
    params = _nav_yaml(context, point_lio, 'sentry_robot1')
    assert params['point_lio']['ros__parameters']['common']['imu_topic'] == (
        '/sentry_robot1/imu/data')
    assert params['point_lio']['ros__parameters']['publish']['tf_send_en'] is False
    assert params['loam_interface']['ros__parameters']['odom_frame'] == 'sentry_robot1/odom'
    for costmap in ('local_costmap', 'global_costmap'):
        assert params[costmap][costmap]['ros__parameters']['robot_radius'] == 0.4
    if mapping:
        assert {'planner_server', 'controller_server', 'small_gicp_relocalization',
                'fake_vel_transform', 'localization_error_monitor'}.isdisjoint(names)
        assert 'mapping_map_to_odom' in names
        assert _parameters(context, point_lio)['pcd_save.pcd_save_en'] is True
    else:
        assert 'mapping_map_to_odom' not in names
        assert 'small_gicp_relocalization' in names
        gicp = _parameters(context, _node(launches, actions, 'small_gicp_relocalization'))
        assert Path(gicp['prior_pcd_file']).is_file()
        assert list(gicp['init_pose']) == [-5.3, 3.0, 0.0, 0.0, 0.0, -1.57079632679]
        reference = _parameters(context, _node(launches, actions, 'sim_reference_pose'))
        assert reference['odom_output'] == '/sentry_robot1/reference/odometry'
        assert reference['pose_output'] == '/sentry_robot1/reference/pose'
        params = _nav_yaml(context, _node(launches, actions, 'planner_server'), 'sentry_robot1')
        assert params['planner_server']['ros__parameters']['GridBased']['plugin'] == (
            'tdt_yastar_planner/YastarPlanner')
        assert params['controller_server']['ros__parameters']['failure_tolerance'] == 0.6


def test_dual_selects_lio_without_restoring_single_scene(launches):
    """The approved default is sensor localization; GT remains an explicit option."""
    module = launches.modules['dual']
    context = _context(module)
    assert context.launch_configurations['localization'] == 'lio'
    actions = module._launch_setup(context)
    forwarded = _include_arguments(context, _include(launches, actions, LAUNCH_FILES['lio']))
    assert forwarded['localization'] == 'lio'
    assert forwarded['slam'] == 'False'
    assert Path(forwarded['params_file']).name == 'nav2_params_lio.yaml'
    assert Path(forwarded['prior_pcd_file']).name == 'diting_3v3_field_mapped.pcd'
    assert len([node for node in _nodes(launches, actions) if node['executable'] == 'create']) == 2
