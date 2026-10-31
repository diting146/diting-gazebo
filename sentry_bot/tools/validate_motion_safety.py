#!/usr/bin/env python3
"""ROS integration fault injection. Run in an isolated ROS_DOMAIN_ID (no robot)."""
import json
import os
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from sentry_interfaces.msg import MotionState
from sentry_interfaces.srv import SetMotionOwner


class Probe(Node):
    def __init__(self):
        super().__init__("motion_safety_probe")
        self.state = None
        self.outputs = []
        self.beat_enabled = True
        self.odom_enabled = True
        self.stale = False
        self.odom_pub = self.create_publisher(Odometry, "/odometry", 10)
        self.beat_pub = self.create_publisher(Empty, "/sentry/mission_heartbeat", 10)
        self.nav_pub = self.create_publisher(Twist, "/sentry/cmd_vel_nav", 10)
        self.tunnel_pub = self.create_publisher(Twist, "/sentry/cmd_vel_tunnel", 10)
        self.create_subscription(MotionState, "/sentry/motion_state", self.on_state, 10)
        self.create_subscription(
            Twist,
            "/cmd_vel",
            lambda m: self.outputs.append(
                (time.monotonic(), m.linear.x, m.linear.y, m.angular.z)
            ),
            10,
        )
        self.owner_client = self.create_client(
            SetMotionOwner, "/sentry/set_motion_owner"
        )
        self.stop_client = self.create_client(Trigger, "/sentry/stop")
        self.create_timer(0.02, self.tick)

    def on_state(self, msg):
        self.state = msg

    def tick(self):
        if self.beat_enabled:
            self.beat_pub.publish(Empty())
        if self.odom_enabled:
            msg = Odometry()
            stamp = self.get_clock().now().nanoseconds - (
                2000000000 if self.stale else 0
            )
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1000000000)
            msg.header.frame_id = "odom"
            msg.child_frame_id = "chassis"
            msg.pose.pose.orientation.w = 1.0
            self.odom_pub.publish(msg)

    def spin_for(self, duration, publish=None):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if publish:
                publish()
            rclpy.spin_once(self, timeout_sec=0.01)

    def call(self, client, request):
        assert client.wait_for_service(timeout_sec=3)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3)
        assert future.done()
        return future.result()

    def select(self, owner, generation=None):
        request = SetMotionOwner.Request()
        request.owner = owner
        request.expected_generation = (
            self.state.generation if generation is None else generation
        )
        return self.call(self.owner_client, request)

    def command(self, publisher, speed=0.1):
        msg = Twist()
        msg.linear.x = speed
        publisher.publish(msg)

    def assert_stopped(self):
        assert self.state.owner == "stop", self.state.reason
        assert all(
            max(abs(v) for v in r[1:]) < 1e-9 for r in self.outputs[-5:]
        ), self.outputs[-5:]


def main():
    if os.environ.get("ROS_DOMAIN_ID") != "73":
        raise RuntimeError("Safety probe must use isolated ROS_DOMAIN_ID=73")
    rclpy.init()
    p = Probe()
    checks = []
    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and (p.state is None or not p.state.stopped):
            p.spin_for(0.1)
        assert p.state and p.state.stopped, str(p.state)
        assert p.select("nav").success
        p.spin_for(0.15, lambda: p.command(p.nav_pub))
        assert any(r[1] > 0.09 for r in p.outputs[-5:])
        checks.append("selected source commands forwarded")
        old = p.state.generation
        assert p.call(p.stop_client, Trigger.Request()).success
        p.spin_for(0.15, lambda: p.command(p.nav_pub))
        p.assert_stopped()
        assert not p.select("tunnel", old).success
        checks.append(
            "operator stop rejects queued stale handoff and subsequent velocity"
        )
        assert p.select("tunnel").success
        p.spin_for(0.15, lambda: p.command(p.nav_pub))
        assert all(abs(r[1]) < 1e-9 for r in p.outputs[-5:])
        checks.append("wrong source and old cache cannot command new owner")
        p.spin_for(0.15, lambda: p.command(p.tunnel_pub))
        assert any(r[1] > 0.09 for r in p.outputs[-5:])
        p.spin_for(0.7)
        p.assert_stopped()
        checks.append("command silence latches stop")
        assert p.select("nav").success
        p.beat_enabled = False
        p.spin_for(0.8, lambda: p.command(p.nav_pub))
        p.assert_stopped()
        checks.append(
            "lost mission heartbeat latches stop despite live command producer"
        )
        p.beat_enabled = True
        p.spin_for(0.2)
        assert p.select("nav").success
        p.stale = True
        p.spin_for(0.8, lambda: p.command(p.nav_pub))
        p.assert_stopped()
        assert not p.select("tunnel").success
        checks.append("replayed stale odometry stops and blocks reactivation")
        p.stale = False
        p.spin_for(0.5)
        assert p.select("nav").success
        p.odom_enabled = False
        p.spin_for(0.8, lambda: p.command(p.nav_pub))
        assert p.state.owner == "nav"
        assert all(max(abs(v) for v in row[1:]) < 1e-9 for row in p.outputs[-5:])
        p.odom_enabled = True
        p.spin_for(0.5, lambda: p.command(p.nav_pub))
        assert p.state.owner == "nav"
        assert any(row[1] > 0.09 for row in p.outputs[-5:])
        checks.append("brief missing odometry holds zero until fresh data returns")
        p.odom_enabled = False
        p.spin_for(2.8, lambda: p.command(p.nav_pub))
        p.assert_stopped()
        checks.append(
            "persistent odometry loss latches stop even with live heartbeat and commands"
        )
        result = {
            "passed": True,
            "checks": checks,
            "ros_domain": 73,
            "note": "Isolated ROS graph, no physical robot or simulator attached.",
        }
        path = Path(__file__).resolve().parents[1] / "validation/motion_safety.json"
        path.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result))
    finally:
        p.call(p.stop_client, Trigger.Request())
        p.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
