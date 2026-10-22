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
Publish the sentry gimbal yaw chain as ROS transforms.

The sentry 3D lidar is mounted on the rotating ``gimbal_yaw`` link, so there is
no fixed transform from the chassis to ``lidar_link``.  This node turns the
``gimbal_yaw_joint`` angle carried by ``/joint_states`` into the transform chain

    chassis -> gimbal_yaw_odom -> gimbal_yaw -> lidar_link

where ``chassis -> gimbal_yaw_odom`` and ``gimbal_yaw -> lidar_link`` are static
and only ``gimbal_yaw_odom -> gimbal_yaw`` changes with the turret angle.

Without this node the lidar frame is disconnected from ``map``/``odom`` and
Nav2 silently drops every point cloud observation.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

WARN_PERIOD_NS = 2_000_000_000


def _quaternion_from_yaw(yaw):
    """Return the quaternion components for a rotation about z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _make_transform(parent, child, stamp, translation, yaw=0.0, pitch=0.0):
    """Build a TransformStamped with yaw and mounting pitch (zero roll)."""
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.translation.x = float(translation[0])
    transform.transform.translation.y = float(translation[1])
    transform.transform.translation.z = float(translation[2])
    _, _, sin_yaw, cos_yaw = _quaternion_from_yaw(yaw)
    sin_pitch = math.sin(pitch / 2.0)
    cos_pitch = math.cos(pitch / 2.0)
    x = -sin_yaw * sin_pitch
    y = cos_yaw * sin_pitch
    z = sin_yaw * cos_pitch
    w = cos_yaw * cos_pitch
    transform.transform.rotation.x = x
    transform.transform.rotation.y = y
    transform.transform.rotation.z = z
    transform.transform.rotation.w = w
    return transform


class SentryGimbalTf(Node):
    """Broadcast the static gimbal mount and the rotating yaw joint."""

    def __init__(self):
        """Read parameters and start the static and dynamic broadcasters."""
        super().__init__('sentry_gimbal_tf')

        self._joint_states_topic = self.declare_parameter(
            'joint_states_topic', '/joint_states'
        ).value
        self._joint_name = self.declare_parameter(
            'joint_name', 'gimbal_yaw_joint'
        ).value
        self._chassis_frame = self.declare_parameter(
            'chassis_frame', 'chassis'
        ).value
        self._gimbal_odom_frame = self.declare_parameter(
            'gimbal_yaw_odom_frame', 'gimbal_yaw_odom'
        ).value
        self._gimbal_frame = self.declare_parameter(
            'gimbal_yaw_frame', 'gimbal_yaw'
        ).value
        self._lidar_frame = self.declare_parameter(
            'lidar_frame', 'lidar_link'
        ).value
        self._gimbal_odom_z = float(
            self.declare_parameter('gimbal_yaw_odom_z', 0.1336).value
        )
        lidar_offset = self.declare_parameter(
            'lidar_offset', [0.15, -0.12, 0.0306]
        ).value
        if len(lidar_offset) != 3:
            raise ValueError(
                'lidar_offset must have 3 elements, got '
                f'{len(lidar_offset)}'
            )
        self._lidar_offset = [float(value) for value in lidar_offset]
        self._lidar_pitch = float(
            self.declare_parameter('lidar_pitch', 0.209440).value
        )

        self._last_warn_ns = None

        self._static_broadcaster = StaticTransformBroadcaster(self)
        stamp = self.get_clock().now().to_msg()
        self._static_broadcaster.sendTransform([
            _make_transform(
                self._chassis_frame,
                self._gimbal_odom_frame,
                stamp,
                (0.0, 0.0, self._gimbal_odom_z),
            ),
            _make_transform(
                self._gimbal_frame,
                self._lidar_frame,
                stamp,
                self._lidar_offset,
                pitch=self._lidar_pitch,
            ),
        ])

        self._dynamic_broadcaster = TransformBroadcaster(self)
        self._subscription = self.create_subscription(
            JointState,
            self._joint_states_topic,
            self._on_joint_states,
            10,
        )

        self.get_logger().info(
            f'sentry_gimbal_tf: {self._joint_states_topic} '
            f'[{self._joint_name}] -> '
            f'{self._gimbal_odom_frame} -> {self._gimbal_frame} -> '
            f'{self._lidar_frame}'
        )

    def _find_joint_position(self, msg):
        """Return the gimbal joint position, tolerating scoped joint names."""
        for index, name in enumerate(msg.name):
            if (
                name == self._joint_name
                or name.endswith('::' + self._joint_name)
                or name.endswith('/' + self._joint_name)
            ):
                if index < len(msg.position):
                    return msg.position[index]
        return None

    def _on_joint_states(self, msg):
        """Publish the yaw transform for the current gimbal angle."""
        position = self._find_joint_position(msg)
        if position is None:
            now_ns = self.get_clock().now().nanoseconds
            if (
                self._last_warn_ns is None
                or now_ns - self._last_warn_ns > WARN_PERIOD_NS
            ):
                self._last_warn_ns = now_ns
                self.get_logger().warning(
                    f'joint {self._joint_name!r} not present in '
                    f'{self._joint_states_topic}; saw {list(msg.name)}'
                )
            return

        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()
        self._dynamic_broadcaster.sendTransform(
            _make_transform(
                self._gimbal_odom_frame,
                self._gimbal_frame,
                stamp,
                (0.0, 0.0, 0.0),
                position,
            )
        )


def main(args=None):
    """Run the node until shutdown."""
    # Keep rclpy's signal handler from shutting the context down mid-spin so
    # Ctrl+C raises KeyboardInterrupt and the node is torn down cleanly.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SentryGimbalTf()
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
