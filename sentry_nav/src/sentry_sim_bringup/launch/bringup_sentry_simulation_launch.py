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
One-shot Gazebo simulation bringup for the pb2025 sentry robot.

The launch starts the rmu_gazebo_simulator bringup (world, two rm25 robots,
referee and rm25 bridge), spawns one pb2025 sentry from the pre-generated SDF,
bridges its sensors and commands with ros_gz_bridge, and starts the fake-MCU
gateway. Ground-truth mode supports small-gyro chassis rotation and simulated
world-yaw stabilization for the gimbal.

Navigation is selected with the ``localization`` launch argument:

* ``ground_truth`` (default) includes ``nav2_ground_truth_launch.py``, which
  runs only the Nav2 servers and derives the pose from the Gazebo
  ``MecanumDrive2`` odometry through ``sim_ground_truth_odometry``.  This is
  the mode that works with the simulated sentry model, whose IMU is not
  compatible with the upstream point_lio pipeline.
* ``lio`` includes the upstream ``pb2025_nav_bringup`` simulation launch with
  point_lio, small_gicp and the terrain analysis stack.  It is kept as an
  optional mode for experiments and is not expected to converge with this
  Gazebo IMU model.

The sentry SDF is a static asset generated once with xmacro4sdf, so no
xacro/sdformat conversion is required at launch time.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    """Convert a launch-argument string to a boolean."""
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _value(context, name):
    """Return the current string value of a launch argument."""
    return LaunchConfiguration(name).perform(context)


def _launch_setup(context):
    """Create the actions that depend on the sentry name substitutions."""
    sentry_name = _value(context, 'sentry_name')
    use_sim_time = _as_bool(_value(context, 'use_sim_time'))
    start_nav2 = _as_bool(_value(context, 'start_nav2'))

    pkg_share = get_package_share_directory('sentry_sim_bringup')
    nav_share = get_package_share_directory('pb2025_nav_bringup')

    template_path = os.path.join(pkg_share, 'config', 'sentry_bridge.yaml.in')
    with open(template_path, 'r', encoding='utf-8') as template_file:
        bridge_config = template_file.read().replace(
            '<sentry_name>', sentry_name
        ).replace('front_mid360_lidar', 'lidar_3d').replace(
            'front_mid360_imu', 'chassis_imu'
        ).replace('front_mid360', 'lidar_link').replace(
            '/livox/lidar', f'/{sentry_name}/points_raw'
        ).replace('/livox/imu', f'/{sentry_name}/imu/data')

    bridge_file = tempfile.NamedTemporaryFile(
        mode='w',
        prefix='sentry_bridge_',
        suffix='.yaml',
        delete=False,
        encoding='utf-8',
    )
    bridge_file.write(bridge_config)
    bridge_file.flush()
    bridge_file.close()

    spawn_sentry = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'default',
            '-file',
            # The sentry lidar sits on the rotating gimbal yaw axis, unlike the
            # shared rm25 model where it is fixed to the chassis.
            os.path.join(pkg_share, 'models', 'sentry_rm25', 'model.sdf'),
            '-name', sentry_name,
            '-allow_renaming', 'false',
            '-x', _value(context, 'x'),
            '-y', _value(context, 'y'),
            '-z', _value(context, 'z'),
            '-Y', _value(context, 'yaw'),
        ],
        output='screen',
    )

    # TF invariant: every frame has exactly one parent.
    #   map -> odom -> chassis -> gimbal_yaw_odom -> gimbal_yaw -> lidar_link
    #                         -> gimbal_yaw_fake (virtual, world-aligned base)
    # fake_vel_transform publishes the virtual base transform. The gimbal
    # segment comes from sentry_gimbal_tf via /joint_states.

    sentry_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_file.name, 'use_sim_time': True}],
        output='screen',
    )

    gateway = Node(
        package='sentry_sim_bringup',
        executable='sim_serial_gateway',
        name='sim_serial_gateway',
        parameters=[{'idle_spin_speed': float(_value(context, 'spin_speed'))}],
        arguments=[
            '--ros-args',
            '-p', 'input_topic:=/cmd_vel',
            '-p', f'output_topic:=/{sentry_name}/cmd_vel',
            '-p', 'timeout_ms:=200',
        ],
        output='screen',
    )

    actions = (
        # The /world/default/create service is not always up 10 s after launch
        # on a cold Gazebo start; 25 s avoids the create request timing out.
        [TimerAction(period=25.0, actions=[spawn_sentry])]
        + [sentry_bridge, gateway]
    )

    if start_nav2:
        params_file = _value(context, 'params_file')
        localization = _value(context, 'localization')
        if not params_file:
            if localization == 'lio':
                params_file = os.path.join(
                    nav_share, 'config', 'simulation', 'nav2_params.yaml'
                )
            else:
                params_file = os.path.join(
                    pkg_share, 'config', 'nav2_params_ground_truth.yaml'
                )
        if localization == 'lio':
            nav_launch_file = os.path.join(
                nav_share,
                'launch',
                'rm_navigation_simulation_launch.py',
            )
            nav_arguments = {
                'namespace': _value(context, 'namespace'),
                'slam': _value(context, 'slam'),
                'map': _value(context, 'map'),
                'prior_pcd_file': _value(context, 'prior_pcd_file'),
                'use_sim_time': _value(context, 'use_sim_time'),
                'autostart': _value(context, 'autostart'),
                'use_composition': _value(context, 'use_composition'),
                'use_respawn': _value(context, 'use_respawn'),
                'use_rviz': _value(context, 'use_rviz'),
                'world': _value(context, 'world'),
                'params_file': params_file,
            }
        else:
            nav_launch_file = os.path.join(
                pkg_share,
                'launch',
                'nav2_ground_truth_launch.py',
            )
            nav_arguments = {
                'map': _value(context, 'map'),
                'params_file': params_file,
                'use_sim_time': _value(context, 'use_sim_time'),
                'autostart': _value(context, 'autostart'),
                'use_rviz': _value(context, 'use_rviz'),
                'use_respawn': _value(context, 'use_respawn'),
                'sentry_name': _value(context, 'sentry_name'),
                'spin_speed': _value(context, 'spin_speed'),
                'gimbal_world_lock': _value(context, 'gimbal_world_lock'),
            }
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_launch_file),
                launch_arguments=nav_arguments.items(),
            )
        )

    return actions


def generate_launch_description():
    """Declare arguments and assemble the simulation launch description."""
    pkg_share = get_package_share_directory('sentry_sim_bringup')
    pb_share = get_package_share_directory('pb2025_robot_description')
    rmu_share = get_package_share_directory('rmu_gazebo_simulator')

    vendor_models = os.path.join(pkg_share, 'models', 'vendor')
    pb_models = os.path.join(pb_share, 'resource', 'models')
    rmu_models = os.path.join(rmu_share, 'resource', 'models')
    existing_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    resource_path = os.pathsep.join(
        path
        for path in (vendor_models, existing_path, pb_models, rmu_models)
        if path
    )

    declared_arguments = [
        DeclareLaunchArgument('world', default_value='diting_3v3_field'),
        DeclareLaunchArgument('slam', default_value='False'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                pkg_share, 'maps', 'diting_3v3_field.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'prior_pcd_file',
            default_value=os.path.join(
                pkg_share, 'maps', 'diting_3v3_field.pcd'
            ),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('autostart', default_value='True'),
        DeclareLaunchArgument('use_composition', default_value='True'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument('use_rviz', default_value='False'),
        DeclareLaunchArgument('sentry_name', default_value='sentry_robot1'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.3'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('start_nav2', default_value='True'),
        DeclareLaunchArgument(
            'spin_speed',
            default_value='3.14',
            description=(
                'Small-gyro chassis rotation speed in rad/s; use 0 to disable'
            ),
        ),
        DeclareLaunchArgument(
            'gimbal_world_lock',
            default_value='True',
            description='Hold the simulated gimbal at its initial world yaw',
        ),
        DeclareLaunchArgument(
            'localization',
            default_value='ground_truth',
            description='Navigation localization mode: ground_truth or lio',
        ),
    ]

    rmu_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rmu_share, 'launch', 'bringup_sim.launch.py')
        )
    )

    ld = LaunchDescription()
    ld.add_action(
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path)
    )
    for argument in declared_arguments:
        ld.add_action(argument)
    ld.add_action(rmu_bringup)
    ld.add_action(OpaqueFunction(function=_launch_setup))
    return ld
