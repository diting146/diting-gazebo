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

"""Hold the simulated gimbal's initial world yaw while the chassis rotates."""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64

from sentry_sim_bringup.gimbal_world_lock_control import WorldYawHold


def _wrap_angle(angle):
    """Return an angle in [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_quaternion(quaternion):
    """Extract planar yaw only from a finite, nonzero orientation."""
    values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('nonfinite orientation')
    norm = math.sqrt(sum(value * value for value in values))
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError('invalid orientation norm')
    x, y, z, w = (value / norm for value in values)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class GimbalWorldLock(Node):
    """Command the gimbal joint to counter-rotate with the chassis."""

    def __init__(self):
        super().__init__('gimbal_world_lock')

        odom_topic = self.declare_parameter(
            'odom_topic', '/sentry_robot1/odometry_raw'
        ).value
        joint_states_topic = self.declare_parameter(
            'joint_states_topic', '/joint_states'
        ).value
        command_topic = self.declare_parameter(
            'command_topic', '/sentry_robot1/gimbal_yaw_cmd'
        ).value
        self._joint_name = self.declare_parameter(
            'joint_name', 'gimbal_yaw_joint'
        ).value

        self._hold = WorldYawHold(
            feedforward_time_sec=float(self.declare_parameter(
                'yaw_feedforward_time_sec', 0.17).value),
            integral_gain=float(self.declare_parameter('yaw_integral_gain', 2.0).value),
            integral_limit=float(self.declare_parameter('yaw_integral_limit', 0.35).value),
            feedback_timeout_sec=float(self.declare_parameter(
                'feedback_timeout_sec', 0.25).value),
        )
        self._logged_target = None

        self._command_publisher = self.create_publisher(Float64, command_topic, 10)
        self._feedback_source = self.declare_parameter('feedback_source', 'imu').value
        imu_topic = self.declare_parameter('imu_topic', '/livox/imu').value
        if self._feedback_source == 'imu':
            self.create_subscription(Imu, imu_topic, self._on_imu, 10)
        elif self._feedback_source == 'odometry':
            self.create_subscription(Odometry, odom_topic, self._on_odometry, 10)
        else:
            raise ValueError('feedback_source must be imu or odometry')
        self.create_subscription(
            JointState, joint_states_topic, self._on_joint_states, 10
        )
        self.create_timer(0.02, self._publish_command)

        self.get_logger().info(
            f'holding initial gimbal world yaw using '
            f'{imu_topic if self._feedback_source == "imu" else odom_topic}, '
            f'{joint_states_topic}, and {command_topic}'
        )

    def _measurement_time(self, msg, now_ns):
        stamp = msg.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        return stamp_ns if stamp_ns else now_ns

    def _on_imu(self, msg):
        try:
            if msg.orientation_covariance[0] == -1.0:
                raise ValueError('orientation unavailable')
            yaw = _yaw_from_quaternion(msg.orientation)
        except ValueError:
            self._hold.invalidate_feedback()
            return
        now_ns = self.get_clock().now().nanoseconds
        self._hold.update_feedback(
            yaw, msg.angular_velocity.z, self._measurement_time(msg, now_ns), now_ns)

    def _on_odometry(self, msg):
        try:
            yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
        except ValueError:
            self._hold.invalidate_feedback()
            return
        now_ns = self.get_clock().now().nanoseconds
        self._hold.update_feedback(
            yaw, msg.twist.twist.angular.z, self._measurement_time(msg, now_ns), now_ns)

    def _on_joint_states(self, msg):
        for index, name in enumerate(msg.name):
            if (
                name == self._joint_name
                or name.endswith('::' + self._joint_name)
                or name.endswith('/' + self._joint_name)
            ) and index < len(msg.position):
                now_ns = self.get_clock().now().nanoseconds
                self._hold.update_joint(
                    float(msg.position[index]), self._measurement_time(msg, now_ns), now_ns)
                return
        self._hold.invalidate_joint()

    def _publish_command(self):
        value = self._hold.command(self.get_clock().now().nanoseconds)
        if self._hold.target_yaw is None:
            self._logged_target = None
        if value is None:
            return
        if self._logged_target != self._hold.target_yaw:
            self._logged_target = self._hold.target_yaw
            self.get_logger().info(
                f'latched gimbal world yaw at {self._logged_target:.3f} rad')
        command = Float64()
        command.data = value
        self._command_publisher.publish(command)


def main(args=None):
    """Run the simulated gimbal world-yaw hold controller."""
    rclpy.init(args=args)
    node = GimbalWorldLock()
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
