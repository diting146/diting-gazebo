#!/usr/bin/env python3
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

"""Minimal HP referee.

Per robot:
  - subscribes  /referee/<robot>/damage  (std_msgs/Int32) and subtracts it from HP
  - publishes   /referee/<robot>/hp      (std_msgs/Int32) at 1 Hz
  - when HP first drops below `threshold`, sends a Nav2 NavigateToPose goal to
    `action_server_name` (default /<robot>/navigate_to_pose) at the robot's
    `home` pose. If no action server is available the node just logs a warning
    and keeps running (so the simulation is usable without Nav2).

home poses are per-robot, indexed by the `robots` list order, via the parallel
parameters home_frame / home_x / home_y / home_z / home_yaw.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Int32


def _at(seq, i, default):
    return seq[i] if i < len(seq) else default


class RefereeHp(Node):
    def __init__(self):
        super().__init__("referee_hp")

        self.declare_parameter("robots", [""])
        self.declare_parameter("initial_hp", 100)
        self.declare_parameter("threshold", 30)
        self.declare_parameter("action_server_name", "/{robot}/navigate_to_pose")
        self.declare_parameter("home_frame", ["map"])
        self.declare_parameter("home_x", [0.0])
        self.declare_parameter("home_y", [0.0])
        self.declare_parameter("home_z", [0.0])
        self.declare_parameter("home_yaw", [0.0])

        self.robots = [r for r in self.get_parameter("robots").value if r]
        self.initial_hp = int(self.get_parameter("initial_hp").value)
        self.threshold = int(self.get_parameter("threshold").value)
        self.action_server_name = self.get_parameter("action_server_name").value
        frames = list(self.get_parameter("home_frame").value)
        xs = list(self.get_parameter("home_x").value)
        ys = list(self.get_parameter("home_y").value)
        zs = list(self.get_parameter("home_z").value)
        yaws = list(self.get_parameter("home_yaw").value)

        self.hp = {r: self.initial_hp for r in self.robots}
        self.go_home_sent = {r: False for r in self.robots}
        self.home = {}
        self.damage_subs = []
        self.hp_pubs = {}
        self.action_clients = {}

        for i, robot in enumerate(self.robots):
            self.home[robot] = {
                "frame": _at(frames, i, "map"),
                "x": float(_at(xs, i, 0.0)),
                "y": float(_at(ys, i, 0.0)),
                "z": float(_at(zs, i, 0.0)),
                "yaw": float(_at(yaws, i, 0.0)),
            }
            self.damage_subs.append(
                self.create_subscription(
                    Int32,
                    f"/referee/{robot}/damage",
                    lambda msg, r=robot: self.damage_cb(r, msg),
                    10,
                )
            )
            self.hp_pubs[robot] = self.create_publisher(
                Int32, f"/referee/{robot}/hp", 10
            )
            self.action_clients[robot] = ActionClient(
                self, NavigateToPose, self.action_server_name.format(robot=robot)
            )

        self.create_timer(1.0, self.publish_hp)
        self.get_logger().info(
            f"referee_hp up: robots={self.robots} initial_hp={self.initial_hp} "
            f"threshold={self.threshold}"
        )

    def damage_cb(self, robot, msg):
        if robot not in self.hp:
            return
        self.hp[robot] = max(0, self.hp[robot] - int(msg.data))
        self.get_logger().info(f"{robot}: damage {msg.data} -> hp={self.hp[robot]}")
        if self.hp[robot] < self.threshold and not self.go_home_sent[robot]:
            self.send_go_home(robot)

    def publish_hp(self):
        for robot, pub in self.hp_pubs.items():
            pub.publish(Int32(data=self.hp[robot]))

    def send_go_home(self, robot):
        self.go_home_sent[robot] = True
        pose = self.home[robot]
        action_name = self.action_server_name.format(robot=robot)
        ac = self.action_clients[robot]
        if not ac.server_is_ready():
            self.get_logger().warn(
                f"{robot}: hp {self.hp[robot]} < {self.threshold} -> go-home requested, "
                f"but Nav2 action server '{action_name}' is not available; skipping "
                f"(simulation keeps running)."
            )
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = pose["frame"]
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = pose["x"]
        goal.pose.pose.position.y = pose["y"]
        goal.pose.pose.position.z = pose["z"]
        goal.pose.pose.orientation.z = math.sin(pose["yaw"] / 2.0)
        goal.pose.pose.orientation.w = math.cos(pose["yaw"] / 2.0)
        self.get_logger().info(
            f"{robot}: hp {self.hp[robot]} < {self.threshold} -> sending go-home goal to "
            f"'{action_name}' [{pose['x']}, {pose['y']}, yaw {pose['yaw']}]."
        )
        ac.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    node = RefereeHp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
