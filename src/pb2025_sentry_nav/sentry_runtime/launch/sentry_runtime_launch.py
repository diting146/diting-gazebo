"""Launch state outputs with explicit, map-calibrated tunnel geometry."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('robot_id', default_value='sentry_robot1'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('base_frame', default_value='chassis'),
        DeclareLaunchArgument('tunnel_id'),
    ]
    geometry = ['entry_x', 'entry_y', 'heading_rad', 'width_m', 'length_m',
                'roof_start_m', 'roof_end_m']
    arguments.extend(DeclareLaunchArgument(name) for name in geometry)
    common = {
        'robot_id': LaunchConfiguration('robot_id'),
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
    }
    tunnel = {
        **common,
        'frame_id': LaunchConfiguration('frame_id'),
        'base_frame': LaunchConfiguration('base_frame'),
        'tunnel_id': LaunchConfiguration('tunnel_id'),
        **{name: ParameterValue(LaunchConfiguration(name), value_type=float)
           for name in geometry},
    }
    return LaunchDescription(arguments + [
        Node(package='sentry_runtime', executable='tunnel_region_monitor',
             namespace=LaunchConfiguration('namespace'), parameters=[tunnel],
             # TF is a shared graph; robot identity is carried by the frame
             # names (for example sentry_robot1/chassis), not by a topic
             # namespace.
             remappings=[]),
        Node(package='sentry_runtime', executable='referee_state_adapter',
             namespace=LaunchConfiguration('namespace'), parameters=[common]),
    ])
