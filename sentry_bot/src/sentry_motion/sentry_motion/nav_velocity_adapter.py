"""Convert each fresh Nav2 command directly from the odom-aligned virtual base.

The upstream fake_vel_transform continues to own the virtual TF. Its cached,
path-synchronized velocity output is not selected in business mode.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from sentry_motion.line_geometry import body_velocity


class NavVelocityAdapter(Node):
    def __init__(self):
        super().__init__("nav_velocity_adapter")
        self.odom = None
        self.pub = self.create_publisher(Twist, "/sentry/cmd_vel_nav", 10)
        self.create_subscription(
            Odometry, "/odometry", self.on_odom, qos_profile_sensor_data
        )
        self.create_subscription(Twist, "/cmd_vel_nav2_result", self.on_command, 10)

    def on_odom(self, msg):
        self.odom = msg

    def on_command(self, command):
        msg = Twist()
        values = (command.linear.x, command.linear.y, command.angular.z)
        if not all(math.isfinite(v) for v in values):
            # Let the arbiter latch the selected source off, rather than hide
            # an invalid producer behind apparently healthy zero commands.
            msg.linear.x = math.nan
            self.pub.publish(msg)
            return
        odom = self.odom
        if odom is None:
            self.pub.publish(msg)
            return
        age = (
            self.get_clock().now() - Time.from_msg(odom.header.stamp)
        ).nanoseconds * 1e-9
        if (
            odom.header.frame_id != "odom"
            or odom.child_frame_id != "chassis"
            or not -0.1 <= age <= 0.3
        ):
            self.pub.publish(msg)
            return
        q = odom.pose.pose.orientation
        if (
            not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w))
            or abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1) > 0.01
        ):
            msg.linear.x = math.nan
            self.pub.publish(msg)
            return
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        vx, vy = body_velocity(
            (0.0, 0.0), (1.0, 0.0), yaw, command.linear.x, command.linear.y
        )
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, command.angular.z
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavVelocityAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
