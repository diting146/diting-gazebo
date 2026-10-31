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

"""Minimal HP-only referee.

Replaces the original 1v1 shoot/hit/RFID referee with:
  - referee_hp.py      : tracks per-robot HP from /referee/<robot>/damage and,
                         below the threshold, sends a Nav2 NavigateToPose goal
                         (go home).
  - damage_injector.py : test helper that publishes damage. Disabled by default
                         (enable with inject_damage:=true).
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_world(gz_world_path):
    """Return the world selected in config/gz_world.yaml."""
    with open(gz_world_path) as file:
        config = yaml.safe_load(file)
    return config.get("world")


def _launch_setup(context, gz_world_path):
    """Create the referee nodes for the robots of the selected world."""
    selected_world = LaunchConfiguration("world").perform(context)
    with open(gz_world_path) as file:
        config = yaml.safe_load(file)
    robots = [str(r["name"]) for r in config["robots"].get(selected_world, [])]

    referee_hp = Node(
        package="rmu_gazebo_simulator",
        executable="referee_hp.py",
        name="referee_hp",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "robots": robots,
                "initial_hp": 100,
                "threshold": 30,
            }
        ],
    )

    # Only started with inject_damage:=true (default off) so the plain launch
    # does not auto-damage the robots.
    damage_injector = Node(
        package="rmu_gazebo_simulator",
        executable="damage_injector.py",
        name="damage_injector",
        output="screen",
        condition=IfCondition(LaunchConfiguration("inject_damage")),
        parameters=[
            {
                "use_sim_time": True,
                "robots": robots,
                "enabled": True,
                "mode": "timer",
                "interval_sec": 5.0,
                "amount": 10,
            }
        ],
    )

    return [referee_hp, damage_injector]


def generate_launch_description():
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")
    gz_world_path = os.path.join(pkg_simulator, "config", "gz_world.yaml")

    declare_inject_damage = DeclareLaunchArgument(
        "inject_damage",
        default_value="false",
        description="Enable the test damage injector (publishes damage periodically).",
    )
    declare_world = DeclareLaunchArgument(
        "world",
        default_value=_default_world(gz_world_path),
        description="World key in config/gz_world.yaml used to select robots",
    )

    ld = LaunchDescription()
    ld.add_action(declare_inject_damage)
    ld.add_action(declare_world)
    ld.add_action(
        OpaqueFunction(function=_launch_setup, args=[gz_world_path])
    )

    return ld
