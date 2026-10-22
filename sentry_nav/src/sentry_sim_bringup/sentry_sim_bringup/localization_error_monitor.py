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
Measure the localization error against the read-only reference pose.

The operational pose comes from TF (``map -> chassis`` by default).  The
reference pose comes from ``/reference/pose``, which is fed by Gazebo ground
truth and is never consumed by the operational stack.  The error is published
as ``geometry_msgs/Vector3Stamped`` on ``/localization_error`` with
``x = dx``, ``y = dy`` and ``z = dyaw`` (all in the map frame).
"""

import math
from collections import deque

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time

MAX_STAMP_SKEW_S = 0.1


def _yaw_from_quaternion(quaternion):
    """Return the planar yaw encoded by a quaternion."""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z
               + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y
                     + quaternion.z * quaternion.z),
    )


def _wrap_angle(angle):
    """Wrap an angle into [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class LocalizationErrorMonitor(Node):
    """Compare the TF pose with the reference pose at a fixed rate."""

    def __init__(self):
        """Read parameters and start the TF listener."""
        super().__init__('localization_error_monitor')

        self._map_frame = self.declare_parameter('map_frame', 'map').value
        self._robot_frame = self.declare_parameter(
            'robot_frame', 'chassis'
        ).value
        self._reference_topic = self.declare_parameter(
            'reference_topic', '/reference/pose'
        ).value
        self._error_topic = self.declare_parameter(
            'error_topic', '/localization_error'
        ).value
        self._rate_hz = float(
            self.declare_parameter('rate_hz', 5.0).value
        )
        if self._rate_hz <= 0.0:
            raise ValueError('rate_hz must be positive')

        self._reference = None
        self._references = deque(maxlen=5000)
        self._dropped = 0

        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer, self)
        self._publisher = self.create_publisher(
            Vector3Stamped, self._error_topic, 10
        )
        self._subscription = self.create_subscription(
            PoseStamped, self._reference_topic, self._on_reference, 10
        )
        self._timer = self.create_timer(1.0 / self._rate_hz, self._on_timer)

        self.get_logger().info(
            f'localization_error_monitor: {self._map_frame} -> '
            f'{self._robot_frame} vs {self._reference_topic}'
        )

    def _on_reference(self, msg):
        """Remember the latest reference pose."""
        self._reference = msg
        self._references.append(msg)

    def _on_timer(self):
        """Publish the current error if both poses are available."""
        reference = self._reference
        if reference is None:
            return

        try:
            transform = self._buffer.lookup_transform(
                self._map_frame,
                self._robot_frame,
                Time(),
            )
        except tf2_ros.TransformException:
            return

        # Choose a reference sample covered by the TF buffer, then query TF
        # at that exact sample time. Nearest-time comparison can manufacture
        # a large yaw error during fast spin when reference delivery is late.
        transform_time = Time.from_msg(transform.header.stamp).nanoseconds
        candidates = [m for m in self._references if
                      Time.from_msg(m.header.stamp).nanoseconds <= transform_time]
        if not candidates:
            return
        reference = max(candidates, key=lambda m:
                        Time.from_msg(m.header.stamp).nanoseconds)
        age = (transform_time - Time.from_msg(
            reference.header.stamp).nanoseconds) * 1e-9
        if age > MAX_STAMP_SKEW_S:
            self._dropped += 1
            return
        try:
            transform = self._buffer.lookup_transform(
                self._map_frame, self._robot_frame,
                Time.from_msg(reference.header.stamp),
            )
        except tf2_ros.TransformException:
            return

        dx = transform.transform.translation.x - reference.pose.position.x
        dy = transform.transform.translation.y - reference.pose.position.y
        dyaw = _wrap_angle(
            _yaw_from_quaternion(transform.transform.rotation)
            - _yaw_from_quaternion(reference.pose.orientation)
        )

        error = Vector3Stamped()
        error.header.stamp = reference.header.stamp
        error.header.frame_id = self._map_frame
        error.vector.x = dx
        error.vector.y = dy
        error.vector.z = dyaw
        self._publisher.publish(error)


def main(args=None):
    """Run the node until shutdown."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = LocalizationErrorMonitor()
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
