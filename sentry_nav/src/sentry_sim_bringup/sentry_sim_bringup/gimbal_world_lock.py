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
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


def _wrap_angle(angle):
    """Return an angle in [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_quaternion(quaternion):
    """Extract planar yaw from a quaternion."""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


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

        self._chassis_yaw = None
        self._joint_yaw = None
        self._world_yaw_target = None

        self._command_publisher = self.create_publisher(Float64, command_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odometry, 10)
        self.create_subscription(
            JointState, joint_states_topic, self._on_joint_states, 10
        )
        self.create_timer(0.02, self._publish_command)

        self.get_logger().info(
            f'holding initial gimbal world yaw using {odom_topic}, '
            f'{joint_states_topic}, and {command_topic}'
        )

    def _try_latch_world_yaw(self):
        if (
            self._world_yaw_target is None
            and self._chassis_yaw is not None
            and self._joint_yaw is not None
        ):
            self._world_yaw_target = _wrap_angle(
                self._chassis_yaw + self._joint_yaw
            )
            self.get_logger().info(
                f'latched gimbal world yaw at '
                f'{self._world_yaw_target:.3f} rad'
            )

    def _on_odometry(self, msg):
        self._chassis_yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
        self._try_latch_world_yaw()

    def _on_joint_states(self, msg):
        for index, name in enumerate(msg.name):
            if (
                name == self._joint_name
                or name.endswith('::' + self._joint_name)
                or name.endswith('/' + self._joint_name)
            ) and index < len(msg.position):
                self._joint_yaw = float(msg.position[index])
                self._try_latch_world_yaw()
                return

    def _publish_command(self):
        if self._world_yaw_target is None or self._chassis_yaw is None:
            return
        command = Float64()
        command.data = self._joint_yaw + _wrap_angle(
            self._world_yaw_target - self._chassis_yaw - self._joint_yaw
        )
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
