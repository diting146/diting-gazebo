#!/usr/bin/env python3
"""Isolated ROS interface checks for the business Nav2 velocity conversion."""
import json
import math
import os
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class Probe(Node):
    def __init__(self):
        super().__init__("nav_adapter_probe")
        self.stale = False
        self.raw_active = True
        self.outputs = []
        self.odom_pub = self.create_publisher(Odometry, "/odometry", 10)
        self.raw_pub = self.create_publisher(Twist, "/cmd_vel_nav2_result", 10)
        self.create_subscription(
            Twist,
            "/sentry/cmd_vel_nav",
            lambda m: self.outputs.append(
                (time.monotonic(), m.linear.x, m.linear.y, m.angular.z)
            ),
            10,
        )
        self.create_timer(0.02, self.tick)

    def tick(self):
        odom = Odometry()
        stamp = self.get_clock().now().nanoseconds - (1000000000 if self.stale else 0)
        odom.header.stamp.sec, odom.header.stamp.nanosec = divmod(stamp, 1000000000)
        odom.header.frame_id = "odom"
        odom.child_frame_id = "chassis"
        odom.pose.pose.orientation.z = math.sqrt(0.5)
        odom.pose.pose.orientation.w = math.sqrt(0.5)
        self.odom_pub.publish(odom)
        if self.raw_active:
            command = Twist()
            command.linear.x = 0.2
            command.angular.z = 0.1
            self.raw_pub.publish(command)

    def spin_for(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)


def main():
    if os.environ.get("ROS_DOMAIN_ID") != "73":
        raise RuntimeError("Use isolated ROS_DOMAIN_ID=73")
    rclpy.init()
    node = Probe()
    checks = []
    try:
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and len(node.outputs) < 10:
            node.spin_for(0.1)
        node.spin_for(0.3)
        assert node.outputs and all(
            abs(r[1]) < 1e-6 and abs(r[2] + 0.2) < 1e-6 and abs(r[3] - 0.1) < 1e-6
            for r in node.outputs[-5:]
        )
        checks.append(
            "90-degree odometry rotates virtual x to chassis negative y; angular velocity preserved"
        )
        node.raw_active = False
        node.spin_for(0.2)
        before = len(node.outputs)
        node.spin_for(0.3)
        assert (
            len(node.outputs) == before
        ), "Adapter replayed a command on an odometry update"
        checks.append("no cached command replay when input producer stops")
        node.raw_active = True
        node.stale = True
        node.spin_for(0.5)
        assert all(max(abs(v) for v in r[1:]) < 1e-9 for r in node.outputs[-5:])
        checks.append("stale odometry produces zero velocity")
        report = {
            "passed": True,
            "checks": checks,
            "ros_domain": 73,
            "no_simulator_attached": True,
        }
        (
            Path(__file__).resolve().parents[1] / "validation/nav_adapter_checks.json"
        ).write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
