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

"""
Nav2 bringup using simulated ground-truth localization.

This launch starts the Nav2 servers and map server with Gazebo ground-truth
odometry. ``sim_ground_truth_odometry`` publishes ``map -> odom -> chassis``;
``fake_vel_transform`` adds the world-aligned ``gimbal_yaw_fake`` navigation
frame and routes Nav2 velocity commands to the chassis. A simulation-only yaw
hold controller models gimbal world stabilization. Live 3D lidar points are
converted to ``/obstacle_scan`` for the Nav2 costmaps.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    """Declare arguments and assemble the ground-truth Nav2 stack."""
    bringup_dir = get_package_share_directory('sentry_sim_bringup')

    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    sentry_name = LaunchConfiguration('sentry_name')
    spin_speed = LaunchConfiguration('spin_speed')
    use_gimbal_world_lock = LaunchConfiguration('gimbal_world_lock')

    # The upstream params file uses '<robot_namespace>' in the costmap
    # observation-source topics.  Replace it with an empty namespace so the
    # topics stay valid even though the terrain nodes are not started.
    params_file = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': ''},
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml_file,
                'init_spin_speed': spin_speed,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace',
    )

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            bringup_dir, 'maps', 'diting_3v3_field.yaml'
        ),
        description='Full path to the map yaml file to load',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            bringup_dir, 'config', 'nav2_params_ground_truth.yaml'
        ),
        description='Full path to the Nav2 parameters file to use',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if True',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup the nav2 stack',
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to start RViz (not started by this launch)',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn Nav2 nodes if one of them crashes',
    )

    declare_sentry_name_cmd = DeclareLaunchArgument(
        'sentry_name', default_value='sentry_robot1'
    )
    declare_spin_speed_cmd = DeclareLaunchArgument(
        'spin_speed',
        default_value='3.14',
        description='Small-gyro chassis rotation speed in rad/s; use 0 to disable',
    )
    declare_gimbal_world_lock_cmd = DeclareLaunchArgument(
        'gimbal_world_lock',
        default_value='True',
        description='Hold the simulated gimbal at its initial world yaw',
    )

    node_kwargs = {
        'output': 'screen',
        'respawn': use_respawn,
        'respawn_delay': 2.0,
        'parameters': [configured_params],
    }

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        **node_kwargs,
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        remappings=[('cmd_vel', 'cmd_vel_controller')],
        **node_kwargs,
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        **node_kwargs,
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        **node_kwargs,
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        remappings=[('cmd_vel', 'cmd_vel_controller')],
        **node_kwargs,
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        remappings=[('cmd_vel', 'cmd_vel_nav2_result')],
        **node_kwargs,
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        **node_kwargs,
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        remappings=[
            ('cmd_vel', 'cmd_vel_controller'),
            ('cmd_vel_smoothed', 'cmd_vel_nav2_result'),
        ],
        **node_kwargs,
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'map_server',
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ],
        }],
    )

    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_transform',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    ground_truth_odometry = Node(
        package='sentry_sim_bringup',
        executable='sim_ground_truth_odometry',
        name='sim_ground_truth_odometry',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_input': ['/', sentry_name, '/odometry_raw'],
            'child_frame': 'chassis',
            'relay_cmd_vel': False,
        }],
        output='screen',
    )

    fake_vel_transform = Node(
        package='fake_vel_transform',
        executable='fake_vel_transform_node',
        name='fake_vel_transform',
        parameters=[configured_params],
        output='screen',
    )

    gimbal_world_lock_node = Node(
        package='sentry_sim_bringup',
        executable='gimbal_world_lock',
        name='gimbal_world_lock',
        condition=IfCondition(use_gimbal_world_lock),
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_topic': ['/', sentry_name, '/odometry_raw'],
            'joint_states_topic': '/joint_states',
            'command_topic': ['/', sentry_name, '/gimbal_yaw_cmd'],
        }],
        output='screen',
    )

    gimbal_tf = Node(
        package='sentry_sim_bringup',
        executable='sentry_gimbal_tf',
        name='sentry_gimbal_tf',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/sentry_robot1/points_raw'),
            ('scan', '/obstacle_scan'),
        ],
        parameters=[configured_params],
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_sentry_name_cmd)
    ld.add_action(declare_spin_speed_cmd)
    ld.add_action(declare_gimbal_world_lock_cmd)

    ld.add_action(map_server)
    ld.add_action(controller_server)
    ld.add_action(smoother_server)
    ld.add_action(planner_server)
    ld.add_action(behavior_server)
    ld.add_action(bt_navigator)
    ld.add_action(waypoint_follower)
    ld.add_action(velocity_smoother)
    ld.add_action(lifecycle_manager)
    ld.add_action(map_to_odom_tf)
    ld.add_action(ground_truth_odometry)
    ld.add_action(fake_vel_transform)
    ld.add_action(gimbal_world_lock_node)
    ld.add_action(gimbal_tf)
    ld.add_action(pointcloud_to_laserscan)

    return ld
