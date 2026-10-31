# Copyright 2025 Lihan Chen
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

"""Spawn every robot listed in config/gz_world.yaml for the selected world.

Robots are spawned directly from the rm25 SDF file (no xacro / URDF conversion,
no robot_state_publisher, no rmoss_gz_base).  The rm25 chassis is driven by the
Gazebo-native MecanumDrive2 plugin, which subscribes to <robot_name>/cmd_vel and
publishes <robot_name>/odometry, so no per-robot control node is needed here.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_world(gz_world_path):
    """Return the world selected in config/gz_world.yaml."""
    with open(gz_world_path) as file:
        config = yaml.safe_load(file)
    return config.get("world")


def _launch_setup(context, pkg_simulator, gz_world_path):
    """Create one spawn action per robot of the selected world."""
    selected_world = LaunchConfiguration("world").perform(context)
    with open(gz_world_path) as file:
        config = yaml.safe_load(file)
    robots = config["robots"].get(selected_world, [])

    robot_sdf_path = os.path.join(
        pkg_simulator, "resource", "models", "rm25_example_robot", "model.sdf"
    )

    actions = []
    for robot in robots:
        if not robot.get("spawn", True):
            continue
        actions.append(
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-file",
                    robot_sdf_path,
                    "-name",
                    str(robot["name"]),
                    "-allow_renaming",
                    "false",
                    "-x",
                    str(robot["x_pose"]),
                    "-y",
                    str(robot["y_pose"]),
                    "-z",
                    str(robot["z_pose"]),
                    "-Y",
                    str(robot["yaw"]),
                ],
                output="screen",
            )
        )
    return actions


def generate_launch_description():
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")
    gz_world_path = os.path.join(pkg_simulator, "config", "gz_world.yaml")

    declare_world = DeclareLaunchArgument(
        "world",
        default_value=_default_world(gz_world_path),
        description="World key in config/gz_world.yaml used to select robots",
    )

    ld = LaunchDescription()
    ld.add_action(declare_world)
    ld.add_action(
        OpaqueFunction(
            function=_launch_setup, args=[pkg_simulator, gz_world_path]
        )
    )

    return ld
