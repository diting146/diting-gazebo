# Copyright 2026 diting
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

"""
Simulation-only ground-truth localization for the sentry robot.

The upstream point_lio stack does not converge with this Gazebo sentry model:
the simulated IMU reports a level orientation while its accelerometer is
tilted, so the LIO odometry drifts away from the true pose.  Instead of trying
to fix the LIO estimator, this node consumes the odometry published by the
Gazebo ``MecanumDrive2`` plugin (bridged from ignition transport to
``nav_msgs/msg/Odometry``) and republishes it as the planar ``odom -> base``
pose expected by Nav2.

The node can optionally relay ``cmd_vel`` for older launch paths. The small-
gyro bringup disables that relay and inserts ``fake_vel_transform`` between
Nav2 and the chassis gateway instead.

This module is intended for simulation only and must not be used on the real
robot, where the MCU provides the odometry and tf chain.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from tf2_ros import TransformBroadcaster


def _yaw_from_quaternion(quaternion):
    """Return the planar yaw angle encoded by a quaternion."""
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


class SimGroundTruthOdometry(Node):
    """Publish planar ground-truth odometry and relay velocity commands."""

    def __init__(self):
        super().__init__('sim_ground_truth_odometry')

        odom_input = self.declare_parameter(
            'odom_input', '/sentry_robot1/odometry_raw'
        ).value
        odom_output = self.declare_parameter(
            'odom_output', '/odometry'
        ).value
        cmd_vel_input = self.declare_parameter(
            'cmd_vel_input', '/cmd_vel_nav2_result'
        ).value
        cmd_vel_output = self.declare_parameter(
            'cmd_vel_output', '/cmd_vel'
        ).value
        self._parent_frame = self.declare_parameter(
            'parent_frame', 'odom'
        ).value
        self._child_frame = self.declare_parameter(
            'child_frame', 'chassis'
        ).value
        self._relay_cmd_vel = bool(
            self.declare_parameter('relay_cmd_vel', True).value
        )
        self._publish_tf = bool(
            self.declare_parameter('publish_tf', True).value
        )
        self._max_rate_hz = float(
            self.declare_parameter('max_rate_hz', 50.0).value
        )
        if self._max_rate_hz <= 0.0:
            raise ValueError(
                'max_rate_hz must be > 0.0, got '
                f'{self._max_rate_hz}'
            )
        self._last_publish_ns = None

        self._odom_publisher = self.create_publisher(
            Odometry, odom_output, 10
        )
        self._cmd_vel_publisher = None
        self._cmd_vel_subscription = None
        if self._relay_cmd_vel:
            self._cmd_vel_publisher = self.create_publisher(
                Twist, cmd_vel_output, 10
            )
            self._cmd_vel_subscription = self.create_subscription(
                Twist, cmd_vel_input, self._on_cmd_vel, 10
            )
        self._tf_broadcaster = (
            TransformBroadcaster(self) if self._publish_tf else None
        )

        self._odom_subscription = self.create_subscription(
            Odometry, odom_input, self._on_odometry, 10
        )
        self.get_logger().info(
            f'sim_ground_truth_odometry: {odom_input} -> {odom_output}, '
            f'child_frame={self._child_frame}, '
            f'cmd_vel relay={self._relay_cmd_vel} '
            f'(max {self._max_rate_hz} Hz)'
        )

    def _on_odometry(self, msg):
        """Flatten the ground-truth pose and republish it for Nav2."""
        now = self.get_clock().now()
        if self._last_publish_ns is not None:
            elapsed_s = (
                now - self._last_publish_ns
            ).nanoseconds * 1e-9
            if elapsed_s < 1.0 / self._max_rate_hz:
                return
        self._last_publish_ns = now

        yaw = _yaw_from_quaternion(msg.pose.pose.orientation)

        odometry = Odometry()
        odometry.header.stamp = msg.header.stamp
        odometry.header.frame_id = self._parent_frame
        odometry.child_frame_id = self._child_frame
        odometry.pose.pose.position.x = msg.pose.pose.position.x
        odometry.pose.pose.position.y = msg.pose.pose.position.y
        odometry.pose.pose.position.z = 0.0
        odometry.pose.pose.orientation.x = 0.0
        odometry.pose.pose.orientation.y = 0.0
        odometry.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odometry.pose.pose.orientation.w = math.cos(yaw / 2.0)
        odometry.pose.covariance = list(msg.pose.covariance)
        odometry.twist = msg.twist
        self._odom_publisher.publish(odometry)

        if self._tf_broadcaster is None:
            return

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self._parent_frame
        transform.child_frame_id = self._child_frame
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)
        self._tf_broadcaster.sendTransform(transform)

    def _on_cmd_vel(self, msg):
        """Forward Nav2 velocity commands to the gateway unchanged."""
        if self._cmd_vel_publisher is not None:
            self._cmd_vel_publisher.publish(msg)


def main(args=None):
    """Run the node until shutdown."""
    # Keep rclpy from shutting the context down inside its signal handler;
    # Ctrl+C then raises KeyboardInterrupt and the node is torn down cleanly.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SimGroundTruthOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
