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

"""Bring up the rm25 multi-robot simulation.

Chain:
  gazebo.launch.py      -> gz server + GUI + /clock bridge
  spawn_robots.launch.py-> spawn every robot in config/gz_world.yaml
  referee_system.launch -> HP-only referee (+ optional damage injector)
  parameter_bridge      -> ONE bridge for all robots (config/ros_gz_bridge.yaml,
                           <robot_name> expanded per robot and concatenated)
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")

    gz_world_path = os.path.join(pkg_simulator, "config", "gz_world.yaml")
    with open(gz_world_path) as file:
        config = yaml.safe_load(file)
        selected_world = config.get("world")
        robots = config["robots"].get(selected_world, [])

    world_sdf_path = os.path.join(
        pkg_simulator, "resource", "worlds", f"{selected_world}_world.sdf"
    )
    ign_config_path = os.path.join(pkg_simulator, "resource", "ign", "gui.config")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulator, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world_sdf_path": world_sdf_path,
            "ign_config_path": ign_config_path,
        }.items(),
    )

    spawn_robots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulator, "launch", "spawn_robots.launch.py")
        )
    )

    referee_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulator, "launch", "referee_system.launch.py")
        )
    )

    # Expand the bridge template for every robot and concatenate into ONE config
    # so a single parameter_bridge serves all robots.
    bridge_template_path = os.path.join(pkg_simulator, "config", "ros_gz_bridge.yaml")
    with open(bridge_template_path) as file:
        template = file.read()

    combined_entries = []
    for robot in robots:
        robot_cfg = yaml.safe_load(template.replace("<robot_name>", str(robot["name"])))
        combined_entries.extend(robot_cfg)

    tmp = tempfile.NamedTemporaryFile(
        mode="w", prefix="rmu_bridge_", suffix=".yaml", delete=False
    )
    yaml.safe_dump(combined_entries, tmp, default_flow_style=False, sort_keys=False)
    tmp.flush()
    tmp.close()

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "-p", f"config_file:={tmp.name}"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_robots_launch)
    ld.add_action(referee_system_launch)
    ld.add_action(bridge)

    return ld
