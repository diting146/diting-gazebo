# Copyright 2026 diting
# Licensed under the Apache License, Version 2.0 (the "License");

"""Launch the autonomous sentry and a keyboard-controlled obstacle robot."""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
    SetEnvironmentVariable, TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from sentry_sim_bringup.bringup_config import (
    DEFAULT_PLANNER, DEFAULT_POSE, DEFAULT_SPAWN_DELAY, DEFAULT_SPIN_SPEED,
    gazebo_resource_path, sentry_bridge, sentry_model,
)


def _launch_setup(context):
    """Create one world, both physical robots and one sentry navigation stack."""
    pkg = get_package_share_directory('sentry_sim_bringup')
    rmu = get_package_share_directory('rmu_gazebo_simulator')

    def value(name):
        return LaunchConfiguration(name).perform(context)

    common_arguments = {name: value(name) for name in (
        'map', 'params_file', 'use_sim_time', 'autostart', 'use_respawn',
        'use_rviz', 'spin_speed', 'gimbal_world_lock', 'planner_plugin',
        'bt_xml_dir', 'localization', 'slam', 'prior_pcd_file',
    )}
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
    x, y, z, yaw = DEFAULT_POSE
    return [
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
        DeclareLaunchArgument('slam', default_value='False'),
        DeclareLaunchArgument('prior_pcd_file', default_value=os.path.join(
            pkg, 'maps', 'diting_3v3_field_mapped.pcd')),
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('autostart', default_value='True'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument('use_rviz', default_value='False'),
        DeclareLaunchArgument('gimbal_world_lock', default_value='True'),
        DeclareLaunchArgument('spawn_delay', default_value=DEFAULT_SPAWN_DELAY),
        DeclareLaunchArgument('bt_xml_dir', default_value=os.path.join(pkg, 'behavior_trees')),
        DeclareLaunchArgument('manual_command_timeout_ms', default_value='1000'),
        DeclareLaunchArgument('spin_speed', default_value=DEFAULT_SPIN_SPEED),
        DeclareLaunchArgument('planner_plugin', default_value=DEFAULT_PLANNER),
        OpaqueFunction(function=_launch_setup),
    ])
