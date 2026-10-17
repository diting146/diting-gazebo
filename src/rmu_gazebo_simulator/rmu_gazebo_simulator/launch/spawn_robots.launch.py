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

"""Spawn every robot listed in config/gz_world.yaml.

Robots are spawned directly from the rm25 SDF file (no xacro / URDF conversion,
no robot_state_publisher, no rmoss_gz_base).  The rm25 chassis is driven by the
Gazebo-native MecanumDrive2 plugin, which subscribes to <robot_name>/cmd_vel and
publishes <robot_name>/odometry, so no per-robot control node is needed here.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")

    robot_sdf_path = os.path.join(
        pkg_simulator, "resource", "models", "rm25_example_robot", "model.sdf"
    )

    gz_world_path = os.path.join(pkg_simulator, "config", "gz_world.yaml")
    with open(gz_world_path) as file:
        config = yaml.safe_load(file)
        selected_world = config.get("world")
        robots = config["robots"].get(selected_world, [])

    ld = LaunchDescription()

    for robot in robots:
        spawn_robot = Node(
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
        ld.add_action(spawn_robot)

    return ld
