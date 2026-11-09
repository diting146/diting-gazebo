"""Run tunnel and referee adapters independently for both sentry robots.

The tunnel coordinates below are an explicitly labelled demonstration region;
they are not the formal competition tunnel calibration.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _robot_runtime(runtime_launch, robot_id, entry_x):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(runtime_launch),
        launch_arguments={
            'namespace': robot_id,
            'robot_id': robot_id,
            'use_sim_time': 'true',
            'frame_id': 'map',
            'base_frame': f'{robot_id}/chassis',
            'tunnel_id': 'demo_tunnel_unverified',
            'entry_x': str(entry_x),
            'entry_y': '0.0',
            'heading_rad': '0.0',
            'width_m': '0.8',
            'length_m': '2.0',
            'roof_start_m': '0.825',
            'roof_end_m': '1.175',
        }.items(),
    )


def generate_launch_description():
    package_share = get_package_share_directory('sentry_runtime')
    runtime_launch = os.path.join(package_share, 'launch',
                                  'sentry_runtime_launch.py')
    return LaunchDescription([
        Node(package='rmu_gazebo_simulator', executable='referee_hp.py',
             name='dual_sentry_referee_hp', output='screen', parameters=[{
                 'use_sim_time': True,
                 'robots': ['sentry_robot1', 'sentry_robot2'],
                 'initial_hp': 100,
                 # HP is clamped to zero: a zero threshold disables the
                 # upstream automatic go-home request in this state-only run.
                 'threshold': 0,
             }]),
        _robot_runtime(runtime_launch, 'sentry_robot1', 0.0),
        _robot_runtime(runtime_launch, 'sentry_robot2', 2.0),
    ])
