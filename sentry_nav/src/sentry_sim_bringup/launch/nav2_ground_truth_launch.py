# Copyright 2026 diting
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Ground-truth navigation component for the autonomous sentry in the dual scene."""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from sentry_sim_bringup.bringup_config import (
    DEFAULT_PLANNER, DEFAULT_SPIN_SPEED, behavior_tree_xml, frame, navigation_parameters,
    normalized_namespace, topic,
)
import yaml


NAV_SERVERS = (
    ('nav2_map_server', 'map_server', []),
    ('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_controller')]),
    ('nav2_smoother', 'smoother_server', []),
    ('nav2_planner', 'planner_server', []),
    ('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_controller')]),
    ('nav2_bt_navigator', 'bt_navigator', [('cmd_vel', 'cmd_vel_nav2_result')]),
    ('nav2_waypoint_follower', 'waypoint_follower', []),
    ('nav2_velocity_smoother', 'velocity_smoother', [
        ('cmd_vel', 'cmd_vel_controller'),
        ('cmd_vel_smoothed', 'cmd_vel_nav2_result'),
    ]),
)


def _launch_setup(context):
    """Resolve one stack; namespace changes names, never navigation tuning."""
    def value(name):
        return LaunchConfiguration(name).perform(context)

    namespace = normalized_namespace(value('namespace'))
    sentry_name = value('sentry_name')
    use_sim_time = value('use_sim_time').lower() in ('true', '1', 'yes', 'on')
    spin_speed = float(value('spin_speed'))
    bt_xml_dir = tempfile.mkdtemp(prefix='sentry_bt_')
    for filename in ('navigate_to_pose_w_replanning_and_recovery.xml',
                     'navigate_through_poses_w_replanning_and_recovery.xml'):
        with open(os.path.join(value('bt_xml_dir'), filename), encoding='utf-8') as stream:
            tree = behavior_tree_xml(stream.read(), namespace)
        with open(os.path.join(bt_xml_dir, filename), 'w', encoding='utf-8') as stream:
            stream.write(tree)
    with open(value('params_file'), encoding='utf-8') as stream:
        params = navigation_parameters(
            yaml.safe_load(stream), namespace, value('map'), use_sim_time,
            bt_xml_dir,
        )
    params['planner_server']['ros__parameters']['GridBased']['plugin'] = value('planner_plugin')
    params['fake_vel_transform']['ros__parameters']['init_spin_speed'] = spin_speed
    with tempfile.NamedTemporaryFile(
        mode='w', prefix='sentry_nav_', suffix='.yaml', delete=False, encoding='utf-8'
    ) as stream:
        yaml.safe_dump({namespace: params} if namespace else params, stream)
        params_path = stream.name
    configured_params = ParameterFile(params_path, allow_substs=True)
    common = {
        'namespace': namespace, 'output': 'screen',
        'respawn': LaunchConfiguration('use_respawn'), 'respawn_delay': 2.0,
    }
    nodes = [Node(
        package=package, executable=name, name=name, remappings=remappings,
        parameters=[configured_params], **common,
    ) for package, name, remappings in NAV_SERVERS]
    nodes.extend([
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', namespace=namespace, output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': value('autostart').lower() in ('true', '1', 'yes', 'on'),
                'node_names': [name for _, name, _ in NAV_SERVERS],
            }],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='map_to_odom_static_transform', namespace=namespace, output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', frame(namespace, 'odom')],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='sentry_sim_bringup', executable='sim_ground_truth_odometry',
            name='sim_ground_truth_odometry', namespace=namespace, output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_input': f'/{sentry_name}/odometry_raw',
                'odom_output': topic(namespace, 'odometry'),
                'parent_frame': frame(namespace, 'odom'),
                'child_frame': frame(namespace, 'chassis'),
                'relay_cmd_vel': False,
            }],
        ),
        Node(
            package='fake_vel_transform', executable='fake_vel_transform_node',
            name='fake_vel_transform', parameters=[configured_params], **common,
        ),
        Node(
            package='sentry_sim_bringup', executable='sim_serial_gateway',
            name='sim_serial_gateway', namespace=namespace, output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': topic(namespace, 'cmd_vel_chassis'),
                'output_topic': f'/{sentry_name}/cmd_vel',
                'timeout_ms': 200, 'publish_rate_hz': 50.0,
                'idle_spin_speed': spin_speed,
                'spin_topic': topic(namespace, 'cmd_spin'),
            }],
        ),
        Node(
            package='sentry_sim_bringup', executable='sentry_gimbal_tf',
            name='sentry_gimbal_tf', namespace=namespace, output='screen', parameters=[{
                'use_sim_time': use_sim_time,
                'joint_states_topic': topic(namespace, 'joint_states'),
                'chassis_frame': frame(namespace, 'chassis'),
                'gimbal_yaw_odom_frame': frame(namespace, 'gimbal_yaw_odom'),
                'gimbal_yaw_frame': frame(namespace, 'gimbal_yaw'),
                'lidar_frame': frame(namespace, 'lidar_link'),
            }],
        ),
        Node(
            package='sentry_sim_bringup', executable='gimbal_world_lock',
            name='gimbal_world_lock', namespace=namespace, output='screen',
            condition=IfCondition(LaunchConfiguration('gimbal_world_lock')),
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_topic': f'/{sentry_name}/odometry_raw',
                'joint_states_topic': topic(namespace, 'joint_states'),
                'command_topic': f'/{sentry_name}/gimbal_yaw_cmd',
            }],
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan', namespace=namespace, output='screen',
            remappings=[('cloud_in', f'/{sentry_name}/points_raw'),
                        ('scan', topic(namespace, 'obstacle_scan'))],
            parameters=[configured_params],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            namespace=namespace, output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            arguments=['-d', os.path.join(
                get_package_share_directory('pb2025_nav_bringup'),
                'rviz', 'nav2_default_view.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
    return nodes


def generate_launch_description():
    """Declare navigation component arguments; robot spawning is owned by the dual scene."""
    bringup_dir = get_package_share_directory('sentry_sim_bringup')
    arguments = [
        DeclareLaunchArgument('namespace', default_value='sentry_robot1'),
        DeclareLaunchArgument('map', default_value=os.path.join(
            bringup_dir, 'maps', 'diting_3v3_field.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            bringup_dir, 'config', 'nav2_params_ground_truth.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('autostart', default_value='True'),
        DeclareLaunchArgument('use_rviz', default_value='False'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument('sentry_name', default_value='sentry_robot1'),
        DeclareLaunchArgument('spin_speed', default_value=DEFAULT_SPIN_SPEED),
        DeclareLaunchArgument('gimbal_world_lock', default_value='True'),
        DeclareLaunchArgument('planner_plugin', default_value=DEFAULT_PLANNER),
        DeclareLaunchArgument('bt_xml_dir', default_value=os.path.join(
            bringup_dir, 'behavior_trees')),
    ]
    return LaunchDescription([*arguments, OpaqueFunction(function=_launch_setup)])
