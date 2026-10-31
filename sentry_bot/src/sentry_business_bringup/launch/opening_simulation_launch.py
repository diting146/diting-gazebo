"""Business overlay: keep sensor localization and Nav2 in their owning workspace."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    sim = get_package_share_directory("sentry_sim_bringup")
    business = get_package_share_directory("sentry_business_bringup")
    return LaunchDescription(
        [
            DeclareLaunchArgument("localization", default_value="lio"),
            DeclareLaunchArgument("use_rviz", default_value="False"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sim, "launch", "bringup_sentry_simulation_launch.py")
                ),
                launch_arguments={
                    "world": "2027RMUL",
                    "localization": LaunchConfiguration("localization"),
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "spin_speed": "0.0",
                    "business_mode": "True",
                    "nav_cmd_vel_output": "/sentry/cmd_vel_nav_legacy",
                }.items(),
            ),
            Node(
                package="sentry_sim_bringup",
                executable="sim_reference_pose",
                name="business_ground_truth_reference",
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("localization"),
                            "' == 'ground_truth'",
                        ]
                    )
                ),
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="sentry_motion",
                executable="nav_velocity_adapter",
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="sentry_motion",
                executable="velocity_arbiter",
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="sentry_motion",
                executable="tunnel_controller",
                parameters=[
                    os.path.join(business, "config", "motion.yaml"),
                    {
                        "use_sim_time": True,
                        "obstacle_topic": ParameterValue(
                            PythonExpression(
                                [
                                    "'/livox/lidar' if '",
                                    LaunchConfiguration("localization"),
                                    "' == 'ground_truth' else '/terrain_map_odom'",
                                ]
                            ),
                            value_type=str,
                        ),
                    },
                ],
                output="screen",
            ),
            Node(
                package="sentry_mission",
                executable="opening_mission",
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
        ]
    )
