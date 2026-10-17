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

"""Test helper: publish damage to the HP referee.

Disabled by default. Enable with the launch argument `inject_damage:=true`, or
call it manually:

    ros2 topic pub /referee/rm25_robot1/damage std_msgs/Int32 "{data: 80}" --once

Parameters:
  robots       (string[])  robot names to damage
  enabled      (string)    "true" to actually publish ("false" = idle)
  mode         (string)    "timer" (default); service mode is a future TODO
  interval_sec (double)    publish period in seconds
  amount       (int)       damage per tick
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class DamageInjector(Node):
    def __init__(self):
        super().__init__("damage_injector")

        self.declare_parameter("robots", [""])
        self.declare_parameter("enabled", True)
        self.declare_parameter("mode", "timer")
        self.declare_parameter("interval_sec", 5.0)
        self.declare_parameter("amount", 10)

        self.robots = [r for r in self.get_parameter("robots").value if r]
        self.enabled = bool(self.get_parameter("enabled").value)
        self.mode = self.get_parameter("mode").value
        self.interval_sec = float(self.get_parameter("interval_sec").value)
        self.amount = int(self.get_parameter("amount").value)

        self.pubs = {
            r: self.create_publisher(Int32, f"/referee/{r}/damage", 10)
            for r in self.robots
        }

        if self.enabled and self.mode == "timer":
            self.create_timer(self.interval_sec, self.tick)
            self.get_logger().info(
                f"damage_injector ENABLED: publish {self.amount} damage every "
                f"{self.interval_sec}s to {list(self.pubs)}"
            )
        else:
            self.get_logger().info(
                "damage_injector idle (enabled="
                f"{self.enabled}, mode={self.mode}). Launch with inject_damage:=true, "
                "or test manually: ros2 topic pub /referee/<robot>/damage "
                'std_msgs/Int32 "{data: 80}" --once'
            )

    def tick(self):
        for robot, pub in self.pubs.items():
            pub.publish(Int32(data=self.amount))
            self.get_logger().info(f"inject {self.amount} damage -> {robot}")


def main(args=None):
    rclpy.init(args=args)
    node = DamageInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
