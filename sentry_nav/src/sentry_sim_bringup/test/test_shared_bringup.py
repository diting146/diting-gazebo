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
        document = document[namespace]
    navigator = document['bt_navigator']['ros__parameters']
    plugins = navigator['plugin_lib_names']
    # A misindented YAML list can parse successfully as one long string; it
    # still cannot be loaded by Nav2. Check the actual generated parameters.
    assert 'sentry_tunnel_bt_node' in plugins
    assert 'nav2_compute_path_to_pose_action_bt_node' in plugins
    assert 'nav2_follow_path_action_bt_node' in plugins
    assert all(isinstance(item, str) and ' ' not in item for item in plugins)
    for client in ('navigate_to_pose', 'navigate_through_poses'):
        name = 'bt_navigator_' + client + '_rclcpp_node'
        assert document[name]['ros__parameters']['use_sim_time'] == navigator['use_sim_time']
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


@pytest.mark.parametrize('role', ['navigation', 'lio'])
def test_namespace_changes_preserve_navigation_tuning(launches, role):
    """Changing a navigation namespace must preserve all navigation tuning."""
    module = launches.modules[role]
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
            costmap = params[name][name]['ros__parameters']
            _assert_body_footprint(costmap, namespace)
        navigator = params['bt_navigator']['ros__parameters']
        directories = set()
        for key in ('default_nav_to_pose_bt_xml', 'default_nav_through_poses_bt_xml'):
            generated_path = Path(navigator[key])
            directories.add(generated_path.parent)
            assert generated_path.is_file()
            generated = ET.parse(generated_path).getroot()
            original = ET.parse(PACKAGE / 'behavior_trees' / generated_path.name).getroot()
            for source_node in original.iter():
                if source_node.tag in {'TunnelPassage', 'TunnelRouteGuard'}:
                    source_node.set('tunnel_half_width', '0.4')
            for node, source_node in zip(generated.iter(), original.iter()):
                for attribute in ('robot_base_frame', 'body_frame', 'control_frame'):
                    if attribute in source_node.attrib:
                        original_frame = source_node.attrib[attribute]
                        expected = (namespace + '/' + original_frame
                                    if namespace else original_frame)
                        assert node.attrib[attribute] == expected
                        node.attrib[attribute] = original_frame
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
        'sim_serial_gateway', 'parameter_bridge', 'create', 'startup_navigation',
        'referee_hp.py'}
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
        ('sentry_robot1', ['-4.6', '3.475', '0.3', '0.0']),
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
        _assert_body_footprint(
            params[costmap][costmap]['ros__parameters'], 'sentry_robot1')
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
        assert list(gicp['initial_robot_pose']) == [-4.6, 3.475, 0.0, 0.0, 0.0, 0.0]
        assert 'init_pose' not in gicp
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


def _assert_body_footprint(costmap, namespace):
    """Verify full dimensions in the body frame rather than a half-width circle."""
    expected = (namespace + '/' if namespace else '') + 'chassis'
    assert costmap['robot_base_frame'] == expected
    assert 'robot_radius' not in costmap
    assert costmap['footprint_padding'] == 0.005
    assert yaml.safe_load(costmap['footprint']) == [
        [-0.288, -0.267], [0.288, -0.267], [0.288, 0.267], [-0.288, 0.267]]


def test_body_and_control_frames_remain_distinct(launches):
    """Changing the footprint frame must not double-rotate chassis commands."""
    for mode in ('navigation', 'lio'):
        context = _context(launches.modules[mode], {'namespace': 'sentry_robot1'})
        actions = launches.modules[mode]._launch_setup(context)
        params = _nav_yaml(context, _node(launches, actions, 'controller_server'), 'sentry_robot1')
        controller = params['controller_server']['ros__parameters']['FollowPath']
        assert controller['control_frame'] == 'sentry_robot1/gimbal_yaw_fake'
        assert controller['use_polygon_collision_check'] is True
        assert controller['collision_prediction_time'] == 0.5
        transform = params['fake_vel_transform']['ros__parameters']
        assert transform['robot_base_frame'] == 'sentry_robot1/chassis'
        assert transform['fake_robot_base_frame'] == 'sentry_robot1/gimbal_yaw_fake'


@pytest.mark.parametrize('role', ['dual', 'navigation', 'lio'])
def test_default_spin_is_off_and_explicit_spin_is_preserved(launches, role):
    """A normal launch must match basic navigation; gyro requires an explicit value."""
    module = launches.modules[role]
    for overrides, expected in (({}, 0.0), ({'spin_speed': '3.14'}, 3.14)):
        context = _context(module, overrides)
        assert float(context.launch_configurations['spin_speed']) == expected
        actions = module._launch_setup(context)
        if role == 'dual':
            forwarded = _include_arguments(
                context, _include(launches, actions, LAUNCH_FILES['lio']))
            assert float(forwarded['spin_speed']) == expected
        else:
            namespace = context.launch_configurations['namespace']
            gateway = _parameters(context, _node(launches, actions, 'sim_serial_gateway'))
            assert gateway['idle_spin_speed'] == expected
            params = _nav_yaml(
                context, _node(launches, actions, 'fake_vel_transform'), namespace)
            assert params['fake_vel_transform']['ros__parameters']['init_spin_speed'] == expected


def test_tunnel_passage_can_be_disabled_without_changing_recovery():
    """The diagnostic switch removes preparation and ongoing route protection."""
    from sentry_sim_bringup.bringup_config import behavior_tree_xml
    template = (PACKAGE / 'behavior_trees' /
                'navigate_to_pose_w_replanning_and_recovery.xml').read_text()
    enabled = ET.fromstring(behavior_tree_xml(template, 'team/sentry'))
    disabled = ET.fromstring(behavior_tree_xml(template, 'team/sentry', False))
    guard = enabled.find('.//TunnelPassage')
    assert guard.attrib['body_frame'] == 'team/sentry/chassis'
    assert guard.attrib['control_frame'] == 'team/sentry/gimbal_yaw_fake'
    assert guard.attrib['goal'] == '{goal}'
    assert disabled.find('.//TunnelPassage') is None
    assert disabled.find('.//TunnelRouteGuard') is None
    route_guard = enabled.find('.//TunnelRouteGuard')
    assert route_guard is not None
    assert route_guard.attrib['path'] == guard.attrib['path'] == '{path}'
    assert route_guard.attrib['tunnels'] == guard.attrib['tunnels']
    pipeline = enabled.find('.//PipelineSequence')
    assert [child.tag for child in pipeline] == [
        'RateController', 'TunnelRouteGuard', 'FollowPath']
    assert pipeline.find('RateController/ComputePathToPose').attrib['path'] == '{path}'
    assert enabled.find('.//RecoveryNode').attrib == disabled.find('.//RecoveryNode').attrib
    assert ET.tostring(enabled.find('.//ReactiveFallback')) == ET.tostring(
        disabled.find('.//ReactiveFallback'))


@pytest.mark.parametrize('filename', [
    'navigate_to_pose_w_replanning_and_recovery.xml',
    'navigate_through_poses_w_replanning_and_recovery.xml',
])
def test_tunnel_clock_follows_explicit_launch_time_mode(filename):
    """The BT private client must not compare wall epoch against simulation seconds."""
    from sentry_sim_bringup.bringup_config import behavior_tree_xml
    template = (PACKAGE / 'behavior_trees' / filename).read_text()
    for use_sim_time in (True, False):
        tree = ET.fromstring(behavior_tree_xml(
            template, 'sentry_robot1', True, use_sim_time))
        assert tree.find('.//TunnelPassage').attrib['use_sim_time'] == str(use_sim_time).lower()


@pytest.mark.parametrize('localization', ['lio', 'ground_truth'])
def test_source_normal_speed_defaults_are_one_meter_per_second(localization):
    """Direct parameter-file users get the same normal-route bound as launches."""
    params = yaml.safe_load((PACKAGE / 'config' /
                             f'nav2_params_{localization}.yaml').read_text())
    controller = params['controller_server']['ros__parameters']['FollowPath']
    assert controller['v_linear_min'] == -1.0
    assert controller['v_linear_max'] == 1.0


@pytest.mark.parametrize('role', ['dual', 'navigation', 'lio'])
def test_default_normal_speed_is_one_and_forwarded(launches, role):
    """The scene and either localization expose one explicit normal-speed API."""
    module = launches.modules[role]
    context = _context(module)
    assert float(context.launch_configurations['normal_speed']) == 1.0
    actions = module._launch_setup(context)
    if role == 'dual':
        forwarded = _include_arguments(
            context, _include(launches, actions, LAUNCH_FILES['lio']))
        assert float(forwarded['normal_speed']) == 1.0
    else:
        namespace = context.launch_configurations['namespace']
        params = _nav_yaml(context, _node(launches, actions, 'controller_server'), namespace)
        controller = params['controller_server']['ros__parameters']['FollowPath']
        assert controller['v_linear_min'] == -1.0
        assert controller['v_linear_max'] == 1.0


@pytest.mark.parametrize('role', ['navigation', 'lio'])
@pytest.mark.parametrize('namespace', ['', 'sentry_robot1', 'test_team/primary'])
@pytest.mark.parametrize('normal_speed', [0.8, 0.2])
def test_normal_speed_only_changes_linear_bounds(launches, role, namespace, normal_speed):
    """A speed override must preserve PID, lookahead, collision and tunnel settings."""
    module = launches.modules[role]
    documents = []
    for overrides in ({}, {'normal_speed': str(normal_speed)}):
        context = _context(module, {'namespace': namespace, **overrides})
        actions = module._launch_setup(context)
        params = _nav_yaml(context, _node(launches, actions, 'controller_server'), namespace)
        navigator = params['bt_navigator']['ros__parameters']
        for key in ('default_nav_to_pose_bt_xml', 'default_nav_through_poses_bt_xml'):
            generated_path = Path(navigator[key])
            # Ignore randomized temporary directories, but compare actual BT contents.
            navigator[key] = generated_path.read_text()
        documents.append(params)
    expected = deepcopy(documents[0])
    controller = expected['controller_server']['ros__parameters']['FollowPath']
    controller['v_linear_min'] = -normal_speed
    controller['v_linear_max'] = normal_speed
    assert documents[1] == expected
    passage = ET.fromstring(expected['bt_navigator']['ros__parameters'][
        'default_nav_to_pose_bt_xml']).find('.//TunnelPassage')
    assert float(passage.attrib['approach_speed']) == 0.45
    assert float(passage.attrib['approach_slowdown_distance']) == 1.0
    assert float(passage.attrib['rolling_alignment_speed']) == 0.45
    assert float(passage.attrib['passage_speed']) == 0.8


@pytest.mark.parametrize('localization,role', [('lio', 'lio'), ('ground_truth', 'navigation')])
def test_dual_forwards_normal_speed_to_either_localization(launches, localization, role):
    """The unique dual scene passes the override to the actual selected component."""
    dual_context = _context(launches.modules['dual'], {
        'localization': localization, 'normal_speed': '0.8'})
    actions = launches.modules['dual']._launch_setup(dual_context)
    forwarded = _include_arguments(
        dual_context, _include(launches, actions, LAUNCH_FILES[role]))
    assert forwarded['normal_speed'] == '0.8'
    context = _context(launches.modules[role], forwarded)
    navigation = launches.modules[role]._launch_setup(context)
    params = _nav_yaml(context, _node(launches, navigation, 'controller_server'), 'sentry_robot1')
    controller = params['controller_server']['ros__parameters']['FollowPath']
    assert controller['v_linear_min'] == -0.8
    assert controller['v_linear_max'] == 0.8


@pytest.mark.parametrize('role', ['navigation', 'lio'])
@pytest.mark.parametrize('normal_speed', ['nan', 'inf', '-inf', '-0.1', '0', 'invalid'])
def test_normal_speed_must_be_finite_and_positive(launches, role, normal_speed):
    """Invalid values fail before any navigation process or temporary tree is created."""
    module = launches.modules[role]
    context = _context(module, {'normal_speed': normal_speed})
    with pytest.raises(ValueError, match='normal_speed must be finite and positive'):
        module._launch_setup(context)


@pytest.mark.parametrize('legacy,enabled,auto_start,expected', [
    ('False', 'False', 'True', False),
    ('False', 'True', 'False', True),
    ('True', 'False', 'False', True),
    ('True', 'True', 'True', True),
])
def test_fixed_route_node_enable_and_auto_start_are_independent(
        launches, legacy, enabled, auto_start, expected):
    """The legacy off switch still creates no task unless explicitly enabled."""
    context = _context(launches.modules['dual'], {
        'auto_nav_to_origin': legacy, 'enable_route_task': enabled,
        'route_auto_start': auto_start})
    actions = launches.modules['dual']._launch_setup(context)
    tasks = [node for node in _nodes(launches, actions)
             if node['executable'] == 'startup_navigation']
    assert len(tasks) == int(expected)
    referees = [node for node in _nodes(launches, actions)
                if node['executable'] == 'referee_hp.py']
    assert len(referees) == int(expected)
    if expected:
        params = _parameters(context, tasks[0])
        assert tasks[0]['namespace'] == 'sentry_robot1'
        assert params['auto_start'] is (auto_start == 'True')
        assert params['base_frame'] == 'sentry_robot1/chassis'
        assert params['enable_low_hp_return'] is True
        assert list(params['route_points']) == [-4.0, 3.475, -2.5, 3.475, 0.0, 0.0]
        assert all(type(point) is float for point in params['route_points'])


def test_fixed_route_is_disabled_during_mapping(launches):
    context = _context(launches.modules['dual'], {
        'slam': 'True', 'enable_route_task': 'True', 'auto_nav_to_origin': 'True'})
    actions = launches.modules['dual']._launch_setup(context)
    assert not any(node['executable'] == 'startup_navigation'
                   for node in _nodes(launches, actions))
    assert not any(node['executable'] == 'referee_hp.py'
                   for node in _nodes(launches, actions))


@pytest.mark.parametrize('mode', ['lio', 'ground_truth'])
def test_simulated_referee_only_provides_hp_to_the_route_owner(launches, mode):
    """One HP source must not send competing goals or track the manual robot."""
    context = _context(launches.modules['dual'], {'localization': mode})
    actions = launches.modules['dual']._launch_setup(context)
    referees = [node for node in _nodes(launches, actions)
                if node['executable'] == 'referee_hp.py']
    assert len(referees) == 1
    assert referees[0]['package'] == 'rmu_gazebo_simulator'
    params = _parameters(context, referees[0])
    assert list(params['robots']) == ['sentry_robot1']
    assert params['initial_hp'] == 100
    assert params['threshold'] == 0
    assert params['use_sim_time'] is True
    task = _parameters(context, _node(launches, actions, 'startup_navigation'))
    assert task['enable_low_hp_return'] is True
    assert task['hp_topic'] == '/referee/sentry_robot1/hp'
    assert type(task['return_hp_threshold']) is int
    assert task['return_hp_threshold'] == 50


def test_simulated_referee_can_be_disabled_without_disabling_the_route(launches):
    """Debug/real hardware launch can leave the existing route unchanged."""
    context = _context(launches.modules['dual'], {'enable_sim_referee': 'False'})
    actions = launches.modules['dual']._launch_setup(context)
    assert not any(node['executable'] == 'referee_hp.py'
                   for node in _nodes(launches, actions))
    task = _parameters(context, _node(launches, actions, 'startup_navigation'))
    assert task['enable_low_hp_return'] is False


def test_simulated_referee_respects_wall_time_configuration(launches):
    context = _context(launches.modules['dual'], {'use_sim_time': 'False'})
    actions = launches.modules['dual']._launch_setup(context)
    referee = _parameters(context, _node(launches, actions, 'sim_referee_hp'))
    assert referee['use_sim_time'] is False


def test_fixed_route_accepts_configured_points_as_double_array(launches):
    context = _context(launches.modules['dual'], {
        'route_points': '[1, 2, 3, 4, 5, 6]'})
    actions = launches.modules['dual']._launch_setup(context)
    task = _node(launches, actions, 'startup_navigation')
    assert list(_parameters(context, task)['route_points']) == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


@pytest.mark.parametrize('route', ['[]', '[0, 0]', '[0, 0, 0, 0, 0, .nan]',
                                  '[true, 0, 0, 0, 0, 0]', '{x: 1}'])
def test_fixed_route_rejects_invalid_launch_points(launches, route):
    context = _context(launches.modules['dual'], {'route_points': route})
    with pytest.raises(ValueError, match='three finite XY pairs'):
        launches.modules['dual']._launch_setup(context)


@pytest.mark.parametrize('role', ['navigation', 'lio'])
@pytest.mark.parametrize('namespace', ['', 'sentry_robot1', 'test_team/primary'])
def test_navigation_profiles_keep_arrival_and_slowdown_independent(launches, role, namespace):
    """Both localization modes must register the profiles requested by either BT."""
    module = launches.modules[role]
    context = _context(module, {'namespace': namespace})
    actions = module._launch_setup(context)
    document = _nav_yaml(context, _node(launches, actions, 'controller_server'), namespace)
    server = document['controller_server']['ros__parameters']
    assert set(server['controller_plugins']) == {'FollowPath', 'FollowPathTunnel'}
    assert set(server['goal_checker_plugins']) == {'general_goal_checker', 'tunnel_goal_checker'}
    assert server['general_goal_checker']['xy_goal_tolerance'] == 0.5
    assert server['tunnel_goal_checker']['xy_goal_tolerance'] == 0.15
    normal = deepcopy(server['FollowPath'])
    tunnel = deepcopy(server['FollowPathTunnel'])
    assert normal.pop('approach_velocity_scaling_dist') == 0.9
    assert tunnel.pop('approach_velocity_scaling_dist') == 0.6
    assert normal == tunnel  # Same algorithm and tuning outside the separate slowdown profile.


def test_multi_pose_route_preserves_final_goal_and_uses_tunnel_preparation():
    """Automatic three-point routes must use the same guarded tunnel phase as single goals."""
    template = (PACKAGE / 'behavior_trees' /
                'navigate_through_poses_w_replanning_and_recovery.xml').read_text()
    enabled = ET.fromstring(bringup_config.behavior_tree_xml(template, 'sentry_robot1'))
    disabled = ET.fromstring(bringup_config.behavior_tree_xml(template, 'sentry_robot1', False))
    final_goal = enabled.find('.//TunnelFinalGoal')
    assert final_goal.attrib == {'goals': '{goals}', 'goal': '{tunnel_goal}'}
    passage = enabled.find('.//TunnelPassage')
    assert passage.attrib['goals'] == '{goals}'
    assert passage.attrib['goal'] == '{tunnel_goal}'
    assert passage.attrib['body_frame'] == 'sentry_robot1/chassis'
    assert passage.attrib['controller_id'] == 'FollowPathTunnel'
    assert passage.attrib['goal_checker_id'] == 'tunnel_goal_checker'
    assert enabled.find('.//TunnelRouteGuard') is not None
    follow = enabled.find('.//FollowPath')
    assert follow.attrib['controller_id'] == 'FollowPath'
    assert follow.attrib['goal_checker_id'] == 'general_goal_checker'
    for tag in ('TunnelFinalGoal', 'TunnelPassage', 'TunnelRouteGuard'):
        assert disabled.find('.//' + tag) is None
    assert enabled.find('.//RecoveryNode').attrib == disabled.find('.//RecoveryNode').attrib
    assert ET.tostring(enabled.find('.//ReactiveFallback')) == ET.tostring(
        disabled.find('.//ReactiveFallback'))


def test_spawn_and_initial_robot_guess_are_independent(launches):
    dual = launches.modules['dual']
    context = _context(dual, {'spawn_x': '-5.1', 'spawn_y': '3.15',
                              'spawn_yaw': '-1.4'})
    actions = dual._launch_setup(context)
    spawns = [node for node in _nodes(launches, actions) if node['executable'] == 'create']
    sentry = next(node for node in spawns if 'sentry_robot1' in node['arguments'])
    assert sentry['arguments'][-8:] == ['-x', '-5.1', '-y', '3.15', '-z', '0.3', '-Y', '-1.4']
    forwarded = _include_arguments(context, _include(launches, actions, LAUNCH_FILES['lio']))
    assert forwarded['initial_pose_x'] == '-4.6'
    assert forwarded['initial_pose_y'] == '3.475'
    assert forwarded['initial_pose_yaw'] == '0.0'
    component = launches.modules['lio']
    child_context = _context(component, forwarded)
    child_actions = component._launch_setup(child_context)
    gicp = _parameters(child_context, _node(launches, child_actions, 'small_gicp_relocalization'))
    assert list(gicp['initial_robot_pose']) == [-4.6, 3.475, 0.0, 0.0, 0.0, 0.0]


def test_explicit_initial_guess_reaches_gicp_without_moving_simulator(launches):
    dual = launches.modules['dual']
    context = _context(dual, {'initial_pose_x': '-5.2', 'initial_pose_y': '3.1',
                              'initial_pose_z': '.02', 'initial_pose_yaw': '-1.5'})
    actions = dual._launch_setup(context)
    sentry = next(node for node in _nodes(launches, actions)
                  if node['executable'] == 'create' and 'sentry_robot1' in node['arguments'])
    assert sentry['arguments'][-8:] == ['-x', '-4.6', '-y', '3.475', '-z', '0.3',
                                       '-Y', '0.0']
    forwarded = _include_arguments(context, _include(launches, actions, LAUNCH_FILES['lio']))
    child_context = _context(launches.modules['lio'], forwarded)
    child_actions = launches.modules['lio']._launch_setup(child_context)
    gicp = _parameters(child_context, _node(launches, child_actions, 'small_gicp_relocalization'))
    assert list(gicp['initial_robot_pose']) == [-5.2, 3.1, .02, 0.0, 0.0, -1.5]


@pytest.mark.parametrize('mode,role,expected', [('lio', 'lio', True),
                                               ('ground_truth', 'navigation', False)])
def test_only_lio_fixed_route_requires_map_registration_health(launches, mode, role, expected):
    context = _context(launches.modules['dual'], {'localization': mode})
    actions = launches.modules['dual']._launch_setup(context)
    task = _parameters(context, _node(launches, actions, 'startup_navigation'))
    assert task['require_localization_status'] is expected
    forwarded = _include_arguments(context, _include(launches, actions, LAUNCH_FILES[role]))
    assert forwarded['initial_pose_x'] == '-4.6'


@pytest.mark.parametrize('key', ['spawn_x', 'spawn_y', 'spawn_z', 'spawn_yaw',
                                  'initial_pose_x', 'initial_pose_y',
                                  'initial_pose_z', 'initial_pose_yaw'])
@pytest.mark.parametrize('value', ['nan', 'inf', 'bad'])
def test_dual_rejects_invalid_pose_parameters(launches, key, value):
    context = _context(launches.modules['dual'], {key: value})
    with pytest.raises(ValueError, match='four finite XYZYaw'):
        launches.modules['dual']._launch_setup(context)


@pytest.mark.parametrize('localization,role', [('lio', 'lio'), ('ground_truth', 'navigation')])
@pytest.mark.parametrize('overrides,deferred', [
    ({}, True),
    ({'auto_nav_to_origin': 'False', 'enable_route_task': 'False'}, False),
    ({'auto_nav_to_origin': 'False', 'enable_route_task': 'True',
      'route_auto_start': 'False'}, True),
    ({'tunnel_passage': 'False'}, False),
])
def test_fixed_route_defers_spin_in_both_command_outputs_and_multi_pose_tree(
        launches, localization, role, overrides, deferred):
    """A route cannot spin before entry; the desired speed survives into the BT."""
    context = _context(launches.modules['dual'], {
        'localization': localization, 'spin_speed': '3.14', **overrides})
    scene = launches.modules['dual']._launch_setup(context)
    forwarded = _include_arguments(context, _include(launches, scene, LAUNCH_FILES[role]))
    assert forwarded['spin_speed'] == '3.14'
    assert forwarded['defer_spin_until_exit'] == str(deferred).lower()
    child_context = _context(launches.modules[role], forwarded)
    actions = launches.modules[role]._launch_setup(child_context)
    expected_initial = 0.0 if deferred else 3.14
    gateway = _parameters(child_context, _node(launches, actions, 'sim_serial_gateway'))
    assert gateway['idle_spin_speed'] == expected_initial
    document = _nav_yaml(child_context, _node(launches, actions, 'fake_vel_transform'),
                         'sentry_robot1')
    assert document['fake_vel_transform']['ros__parameters']['init_spin_speed'] == expected_initial
    navigator = document['bt_navigator']['ros__parameters']
    multiple = ET.parse(navigator['default_nav_through_poses_bt_xml'])
    passage = multiple.find('.//TunnelPassage')
    if overrides.get('tunnel_passage') == 'False':
        assert passage is None
    else:
        assert float(passage.attrib['exit_spin_speed']) == (3.14 if deferred else 0.0)
        assert passage.attrib['keep_exit_spin_at_goal'] == str(deferred).lower()
    single = ET.parse(navigator['default_nav_to_pose_bt_xml']).find('.//TunnelPassage')
    if single is not None:
        assert float(single.attrib.get('exit_spin_speed', '0.0')) == 0.0
        assert single.attrib.get('keep_exit_spin_at_goal', 'false') == 'false'


def test_mapping_does_not_defer_spin_or_create_route_task(launches):
    """Mapping keeps the pre-existing manual spin semantics with no route owner."""
    context = _context(launches.modules['dual'], {'slam': 'True', 'spin_speed': '3.14'})
    scene = launches.modules['dual']._launch_setup(context)
    forwarded = _include_arguments(context, _include(launches, scene, LAUNCH_FILES['lio']))
    assert forwarded['defer_spin_until_exit'] == 'false'
    assert not any(node['executable'] == 'startup_navigation' for node in _nodes(launches, scene))
    child = _context(launches.modules['lio'], forwarded)
    actions = launches.modules['lio']._launch_setup(child)
    # Mapping omits some Nav2 nodes, but preserves the physical gateway command policy.
    gateway = _parameters(child, _node(launches, actions, 'sim_serial_gateway'))
    assert gateway['idle_spin_speed'] == 3.14


@pytest.mark.parametrize('role', ['navigation', 'lio'])
@pytest.mark.parametrize('speed', ['nan', 'inf', '-inf'])
def test_nonfinite_spin_fails_before_launch(launches, role, speed):
    context = _context(launches.modules[role], {'spin_speed': speed})
    with pytest.raises(ValueError, match='spin_speed must be finite'):
        launches.modules[role]._launch_setup(context)


def test_ground_truth_uses_one_odometry_feedback_world_lock(launches):
    """Ground truth uses the same compensator without introducing a second command source."""
    context = _context(launches.modules['navigation'], {'namespace': 'sentry_robot1'})
    actions = launches.modules['navigation']._launch_setup(context)
    locks = [node for node in _nodes(launches, actions)
             if node['executable'] in ('gimbal_world_lock', 'sensor_gimbal_world_lock')]
    assert len(locks) == 1
    assert locks[0]['executable'] == 'sensor_gimbal_world_lock'
    parameters = _parameters(context, locks[0])
    assert parameters['feedback_source'] == 'odometry'
    assert parameters['odom_topic'] == '/sentry_robot1/odometry_raw'
    assert parameters['command_topic'] == '/sentry_robot1/gimbal_yaw_cmd'


@pytest.mark.parametrize('mode,role', [('lio', 'lio'), ('ground_truth', 'navigation')])
def test_scene_file_reaches_task_bt_and_planner(launches, tmp_path, mode, role):
    from sentry_sim_bringup.scene_geometry import load_scene_geometry
    data = yaml.safe_load((PACKAGE / 'config/scene_geometry.yaml').read_text())
    for tunnel in data['tunnels']:
        tunnel['x_min'] += 0.2
        tunnel['x_max'] += 0.2
        tunnel['soft_cost_roi'][0] += 0.2
        tunnel['soft_cost_roi'][2] += 0.2
    data['default_route']['points'][0][0] += 0.2
    data['default_route']['yaw'] = 0.15
    scene_file = tmp_path / 'replacement_scene.yaml'
    scene_file.write_text(yaml.safe_dump(data))
    scene = load_scene_geometry(scene_file)
    context = _context(launches.modules['dual'], {
        'scene_file': str(scene_file), 'localization': mode})
    actions = launches.modules['dual']._launch_setup(context)
    task = _parameters(context, _node(launches, actions, 'startup_navigation'))
    assert task['scene_file'] == str(scene_file)
    assert list(task['route_points']) == list(scene.route_points)
    assert task['route_yaw'] == scene.route_yaw
    forwarded = _include_arguments(context, _include(launches, actions, LAUNCH_FILES[role]))
    assert forwarded['scene_file'] == str(scene_file)
    child_context = _context(launches.modules[role], forwarded)
    child_actions = launches.modules[role]._launch_setup(child_context)
    params = _nav_yaml(child_context, _node(launches, child_actions, 'planner_server'), 'sentry_robot1')
    assert params['planner_server']['ros__parameters']['GridBased']['tunnel_cost_regions'] == scene.planner_regions
    for key in ('default_nav_to_pose_bt_xml', 'default_nav_through_poses_bt_xml'):
        tree = ET.parse(params['bt_navigator']['ros__parameters'][key]).getroot()
        for node in tree.iter():
            if node.tag in {'TunnelPassage', 'TunnelRouteGuard'}:
                assert node.attrib['tunnels'] == scene.bt_tunnels
                assert float(node.attrib['tunnel_half_width']) == scene.tunnel_half_width


@pytest.mark.parametrize('role', ['navigation', 'lio'])
def test_custom_parameter_and_bt_geometry_remain_explicit_overrides(launches, tmp_path, role):
    import shutil
    config = PACKAGE / 'config' / ('nav2_params_lio.yaml' if role == 'lio' else 'nav2_params_ground_truth.yaml')
    document = yaml.safe_load(config.read_text())
    custom_roi = [1.0, 2.0, 3.0, 4.0]
    document['planner_server']['ros__parameters']['GridBased']['tunnel_cost_regions'] = custom_roi
    params_file = tmp_path / 'custom_params.yaml'
    params_file.write_text(yaml.safe_dump(document))
    trees = tmp_path / 'custom_trees'
    shutil.copytree(PACKAGE / 'behavior_trees', trees)
    for path in trees.glob('*.xml'):
        path.write_text(path.read_text().replace('-3.5;-1.5;3.475;1.5;3.5;-3.475', '10;12;2'))
    module = launches.modules[role]
    context = _context(module, {'params_file': str(params_file), 'bt_xml_dir': str(trees)})
    actions = module._launch_setup(context)
    params = _nav_yaml(context, _node(launches, actions, 'planner_server'), 'sentry_robot1')
    assert params['planner_server']['ros__parameters']['GridBased']['tunnel_cost_regions'] == custom_roi
    tree = ET.parse(params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']).getroot()
    assert tree.find('.//TunnelPassage').attrib['tunnels'] == '10;12;2'
    assert 'tunnel_half_width' not in tree.find('.//TunnelPassage').attrib


@pytest.mark.parametrize('route', [''])
def test_empty_route_uses_scene_default(launches, route):
    context = _context(launches.modules['dual'], {'route_points': route})
    actions = launches.modules['dual']._launch_setup(context)
    task = _parameters(context, _node(launches, actions, 'startup_navigation'))
    assert list(task['route_points']) == [-4.0, 3.475, -2.5, 3.475, 0.0, 0.0]
