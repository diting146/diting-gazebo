# Copyright 2026 diting
# Licensed under the Apache License, Version 2.0 (the "License");

"""Launch the autonomous sentry and a keyboard-controlled obstacle robot."""

import os
import tempfile
import uuid

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction,
    SetEnvironmentVariable, TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from sentry_sim_bringup.scene_geometry import load_scene_geometry, resolve_route
from sentry_sim_bringup.bringup_config import (
    DEFAULT_INITIAL_POSE, DEFAULT_PLANNER, DEFAULT_POSE, DEFAULT_SPAWN_DELAY, DEFAULT_SPIN_SPEED,
    finite_pose, gazebo_resource_path, sentry_bridge, sentry_model,
)


def _launch_setup(context):
    """Create one world, both physical robots and one sentry navigation stack."""
    pkg = get_package_share_directory('sentry_sim_bringup')
    rmu = get_package_share_directory('rmu_gazebo_simulator')

    def value(name):
        return LaunchConfiguration(name).perform(context)

    scene = load_scene_geometry(value('scene_file'))
    common_arguments = {name: value(name) for name in (
        'scene_file', 'map', 'params_file', 'use_sim_time', 'autostart', 'use_respawn',
        'use_rviz', 'spin_speed', 'normal_speed', 'gimbal_world_lock', 'planner_plugin',
        'bt_xml_dir', 'tunnel_passage', 'localization', 'slam', 'prior_pcd_file',
        'initial_pose_x', 'initial_pose_y', 'initial_pose_z', 'initial_pose_yaw',
    )}
    task_enabled = any(value(name).lower() in ('true', '1', 'yes', 'on')
                       for name in ('auto_nav_to_origin', 'enable_route_task'))
    mapping = value('slam').lower() in ('true', '1', 'yes', 'on')
    sim_referee = (task_enabled and not mapping
                   and value('enable_sim_referee').lower() in ('true', '1', 'yes', 'on'))
    defer_spin = (task_enabled
                  and value('tunnel_passage').lower() in ('true', '1', 'yes', 'on')
                  and not mapping)
    common_arguments['defer_spin_until_exit'] = str(defer_spin).lower()
    common_arguments.update({
        'namespace': 'sentry_robot1', 'sentry_name': 'sentry_robot1',
    })
    localization = value('localization')
    if localization not in ('lio', 'ground_truth'):
        raise ValueError('localization must be lio or ground_truth')
    if value('slam').lower() in ('true', '1') and localization != 'lio':
        raise ValueError('slam requires localization:=lio')
    common_arguments['params_file'] = value('params_file') or os.path.join(
        pkg, 'config', 'nav2_params_' + localization + '.yaml')
    namespace = 'sentry_robot1'
    with open(os.path.join(pkg, 'models', 'sentry_rm25', 'model.sdf'),
              encoding='utf-8') as stream:
        model = sentry_model(stream.read(), 'sentry_robot1', namespace)
    with tempfile.NamedTemporaryFile(
        mode='w', prefix='sentry_model_', suffix='.sdf', delete=False, encoding='utf-8'
    ) as stream:
        stream.write(model)
        sentry_model_path = stream.name
    with open(os.path.join(pkg, 'config', 'sentry_bridge.yaml.in'),
              encoding='utf-8') as stream:
        bridge = sentry_bridge(stream.read(), 'sentry_robot1', namespace)
    with tempfile.NamedTemporaryFile(
        mode='w', prefix='sentry_bridge_', suffix='.yaml', delete=False, encoding='utf-8'
    ) as stream:
        stream.write(bridge)
        sentry_bridge_path = stream.name

    # Deliberately retain the manual model and its existing two bridges.
    with tempfile.NamedTemporaryFile(
        mode='w', prefix='manual_robot_bridge_', suffix='.yaml', delete=False,
        encoding='utf-8',
    ) as bridge_file:
        bridge_file.write('''
- ros_topic_name: "/rm25_robot1/cmd_vel_safe"
  gz_topic_name: "/rm25_robot1/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "ignition.msgs.Twist"
  direction: "ROS_TO_GZ"
- ros_topic_name: "/rm25_robot1/odometry"
  gz_topic_name: "/rm25_robot1/odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "ignition.msgs.Odometry"
  direction: "GZ_TO_ROS"
''')
        bridge_path = bridge_file.name

    use_sim_time = LaunchConfiguration('use_sim_time')
    spawn = finite_pose([value('spawn_' + key) for key in ('x', 'y', 'z', 'yaw')],
                        'spawn pose')
    finite_pose([value('initial_pose_' + key) for key in ('x', 'y', 'z', 'yaw')],
                'initial pose')
    x, y, z, yaw = [str(item) for item in spawn]
    # Keep local Gazebo discovery off automatically selected virtual adapters.
    # An explicit caller address still supports external simulation servers.
    gazebo_ip = context.environment.get('IGN_IP') or '127.0.0.1'
    owns_simulator = IfCondition(LaunchConfiguration('start_simulator')).evaluate(context)
    gazebo_partition = context.environment.get('IGN_PARTITION')
    auto_partition = owns_simulator and not gazebo_partition
    if auto_partition:
        gazebo_partition = 'sentry_sim_' + uuid.uuid4().hex
    actions = [
        SetEnvironmentVariable('IGN_IP', gazebo_ip),
        LogInfo(msg=(
            'Sentry simulation settings: '
            f'localization={localization}, gazebo_ip={gazebo_ip}, '
            f'gazebo_partition={gazebo_partition or "external/default"}, '
            f'normal_speed={value("normal_speed")}, '
            f'use_rviz={value("use_rviz")}, '
            f'auto_nav_to_origin={value("auto_nav_to_origin")}, '
            f'tunnel_passage={value("tunnel_passage")}')),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', gazebo_resource_path(
            pkg, get_package_share_directory('pb2025_robot_description'), rmu,
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''))),
        GroupAction(scoped=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    pkg, 'launch', ('lio_navigation_launch.py' if localization == 'lio'
                                    else 'nav2_ground_truth_launch.py'))),
                launch_arguments=common_arguments.items(),
            ),
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(rmu, 'launch', 'gazebo.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_simulator')),
            launch_arguments={
                'world_sdf_path': os.path.join(
                    rmu, 'resource', 'worlds', value('world') + '_world.sdf'),
                'ign_config_path': os.path.join(rmu, 'resource', 'ign', 'gui.config'),
            }.items(),
        ),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='sentry_sensor_bridge', output='screen', parameters=[{
                'config_file': sentry_bridge_path,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
        TimerAction(period=float(value('spawn_delay')), actions=[Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=[
                '-world', 'default', '-file', sentry_model_path,
                '-name', 'sentry_robot1', '-allow_renaming', 'false',
                '-x', x, '-y', y, '-z', z, '-Y', yaw,
            ],
        )]),
        Node(
            package='sentry_sim_bringup', executable='sim_serial_gateway',
            name='manual_cmd_vel_watchdog', output='screen', parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'input_topic': '/rm25_robot1/cmd_vel',
                'output_topic': '/rm25_robot1/cmd_vel_safe',
                'timeout_ms': ParameterValue(
                    LaunchConfiguration('manual_command_timeout_ms'), value_type=int),
                'idle_spin_speed': 0.0, 'spin_topic': '',
            }],
        ),
        TimerAction(period=float(value('spawn_delay')), actions=[Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=[
                '-world', 'default', '-file', os.path.join(
                    rmu, 'resource', 'models', 'rm25_example_robot', 'model.sdf'),
                '-name', 'rm25_robot1', '-allow_renaming', 'false',
                '-x', '5.3', '-y', '-3.0', '-z', '0.3', '-Y', '0.0',
            ],
        )]),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge', output='screen',
            name='manual_robot_bridge',
            parameters=[{'config_file': bridge_path,
                         'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        ),
    ]
    if auto_partition:
        # Every child, bridge and spawn request must share this owned world.
        # An external server keeps the caller's exact transport environment.
        actions.insert(0, SetEnvironmentVariable('IGN_PARTITION', gazebo_partition))
    if task_enabled and not mapping:
        route, route_yaw = resolve_route(scene, value('route_points'), value('route_yaw'))
        actions.append(Node(
            package='sentry_sim_bringup', executable='startup_navigation',
            name='startup_navigation', namespace=namespace, output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'base_frame': namespace + '/chassis',
                'require_localization_status': localization == 'lio',
                'auto_start': value('route_auto_start').lower() in ('true', '1', 'yes', 'on'),
                'scene_file': value('scene_file'),
                'route_points': route, 'route_yaw': route_yaw,
                'enable_low_hp_return': sim_referee,
                'hp_topic': '/referee/sentry_robot1/hp',
                'return_hp_threshold': 50,
            }],
        ))
        if sim_referee:
            # Let the task owner cancel/serialize actions and use its recorded
            # home pose. The legacy referee must never send a second Nav2 goal.
            actions.append(Node(
                package='rmu_gazebo_simulator', executable='referee_hp.py',
                name='sim_referee_hp', output='screen', parameters=[{
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                    'robots': ['sentry_robot1'],
                    'initial_hp': 100,
                    'threshold': 0,
                }],
            ))
    return actions


def generate_launch_description():
    """Declare the sole simulation entry and its navigation/manual settings."""
    pkg = get_package_share_directory('sentry_sim_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='diting_3v3_field'),
        DeclareLaunchArgument('start_simulator', default_value='True'),
        DeclareLaunchArgument('map', default_value=os.path.join(
            pkg, 'maps', 'diting_3v3_field.yaml')),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('localization', default_value='lio'),
        *[DeclareLaunchArgument('spawn_' + key, default_value=default)
          for key, default in zip(('x', 'y', 'z', 'yaw'), DEFAULT_POSE)],
        *[DeclareLaunchArgument('initial_pose_' + key, default_value=default)
          for key, default in zip(('x', 'y', 'z', 'yaw'), DEFAULT_INITIAL_POSE)],
        DeclareLaunchArgument('slam', default_value='False'),
        DeclareLaunchArgument('prior_pcd_file', default_value=os.path.join(
            pkg, 'maps', 'diting_3v3_field_mapped.pcd')),
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('autostart', default_value='True'),
        DeclareLaunchArgument('auto_nav_to_origin', default_value='True',
                              description='Legacy enable switch for the fixed three-point route task.'),
        DeclareLaunchArgument('enable_route_task', default_value='False',
                              description='Enable route services even when the legacy switch is off.'),
        DeclareLaunchArgument('route_auto_start', default_value='True',
                              description='Run the fixed route once after navigation is ready.'),
        DeclareLaunchArgument('enable_sim_referee', default_value='True',
                              description='Enable simulated HP and low-HP return for the fixed-route task.'),
        DeclareLaunchArgument('scene_file', default_value=os.path.join(
            pkg, 'config', 'scene_geometry.yaml')),
        DeclareLaunchArgument('route_yaw', default_value='',
                              description='Empty uses scene default yaw; explicit finite radians override.'),
        DeclareLaunchArgument('route_points',
                              default_value='',
                              description='Empty uses scene default; otherwise three map-frame XY pairs.'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument('use_rviz', default_value='False'),
        DeclareLaunchArgument('gimbal_world_lock', default_value='True'),
        DeclareLaunchArgument('spawn_delay', default_value=DEFAULT_SPAWN_DELAY),
        DeclareLaunchArgument('tunnel_passage', default_value='True',
                              description='Prepare and align before an actually planned tunnel crossing.'),
        DeclareLaunchArgument('bt_xml_dir', default_value=os.path.join(pkg, 'behavior_trees')),
        DeclareLaunchArgument('manual_command_timeout_ms', default_value='1000'),
        DeclareLaunchArgument('spin_speed', default_value=DEFAULT_SPIN_SPEED),
        DeclareLaunchArgument('normal_speed', default_value='1.0',
                              description='Maximum normal-route XY speed in m/s.'),
        DeclareLaunchArgument('planner_plugin', default_value=DEFAULT_PLANNER),
        OpaqueFunction(function=_launch_setup),
    ])
