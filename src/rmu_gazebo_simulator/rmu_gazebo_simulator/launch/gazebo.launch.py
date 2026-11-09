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

"""Track the Fortress server and GUI directly so shutdown owns both processes."""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    Shutdown,
)
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def _gazebo_environment(context):
    """Keep the upstream package-exported search paths without its shell launcher."""
    upstream_path = os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
    )
    spec = importlib.util.spec_from_file_location("fortress_path_exports", upstream_path)
    upstream = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(upstream)
    model_paths, plugin_paths = upstream.GazeboRosPaths.get_paths()
    environment = context.environment
    return {
        name: os.pathsep.join(parts)
        for name, parts in {
            "GZ_SIM_SYSTEM_PLUGIN_PATH": [
                environment.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
                environment.get("LD_LIBRARY_PATH", ""), plugin_paths],
            "IGN_GAZEBO_SYSTEM_PLUGIN_PATH": [
                environment.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", ""),
                environment.get("LD_LIBRARY_PATH", ""), plugin_paths],
            "GZ_SIM_RESOURCE_PATH": [
                environment.get("GZ_SIM_RESOURCE_PATH", ""), model_paths],
            "IGN_GAZEBO_RESOURCE_PATH": [
                environment.get("IGN_GAZEBO_RESOURCE_PATH", ""), model_paths],
        }.items()
    }


def _launch_setup(context):
    """Use -s/-g modes, which avoid the combined CLI's detached fork children."""
    executable = [FindExecutable(name="ruby"), FindExecutable(name="ign"), "gazebo"]
    environment = _gazebo_environment(context)
    common = {
        "output": "screen", "shell": False, "additional_env": environment,
        "sigterm_timeout": "10.0", "sigkill_timeout": "5.0",
    }
    server = ExecuteProcess(
        cmd=executable + ["-s", "-r", "-v", "2", LaunchConfiguration("world_sdf_path"),
                          "--force-version", "6"],
        name="gazebo_server",
        on_exit=Shutdown(reason="Owned Gazebo server exited"),
        **common,
    )
    gui = ExecuteProcess(
        cmd=executable + ["-g", "-v", "2", LaunchConfiguration("world_sdf_path"),
                          "--gui-config", LaunchConfiguration("ign_config_path"),
                          "--force-version", "6"],
        name="gazebo_gui",
        on_exit=Shutdown(reason="Owned Gazebo GUI exited"),
        **common,
    )
    clock_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
    )
    return [server, gui, clock_bridge]


def generate_launch_description():
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")

    declare_world_sdf_path = DeclareLaunchArgument(
        "world_sdf_path",
        default_value=os.path.join(
            pkg_simulator, "resource", "worlds", "diting_3v3_field_world.sdf"
        ),
        description="Path to the world SDF file",
    )

    declare_ign_config_path = DeclareLaunchArgument(
        "ign_config_path",
        default_value=os.path.join(pkg_simulator, "resource", "ign", "gui.config"),
        description="Path to the Ignition Gazebo GUI configuration file",
    )

    ld = LaunchDescription()

    ld.add_action(declare_world_sdf_path)
    ld.add_action(declare_ign_config_path)
    ld.add_action(OpaqueFunction(function=_launch_setup))

    return ld
