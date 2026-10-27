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
Publish Gazebo ground truth as a read-only reference channel.

This node exists so that acceptance runs can measure localization error
without letting ground truth leak into the operational stack: it publishes
only ``/reference/odometry`` and ``/reference/pose``.  It deliberately does
not broadcast TF, does not publish ``/odometry``, and does not relay
``cmd_vel``.
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


class SimReferencePose(Node):
    """Mirror the simulator pose onto the read-only reference topics."""

    def __init__(self, **kwargs):
        """Read parameters and create the reference publishers."""
        super().__init__('sim_reference_pose', **kwargs)

        odom_input = self.declare_parameter(
            'odom_input', '/sentry_robot1/odometry_raw'
        ).value
        odom_output = self.declare_parameter(
            'odom_output', '/reference/odometry'
        ).value
        pose_output = self.declare_parameter(
            'pose_output', '/reference/pose'
        ).value
        self._frame_id = self.declare_parameter(
            'frame_id', 'map'
        ).value

        self._odom_publisher = self.create_publisher(
            Odometry, odom_output, 10
        )
        self._pose_publisher = self.create_publisher(
            PoseStamped, pose_output, 10
        )
        self._subscription = self.create_subscription(
            Odometry, odom_input, self._on_odometry, 10
        )

        self.get_logger().info(
            f'sim_reference_pose: {odom_input} -> {odom_output}, '
            f'{pose_output} (frame {self._frame_id})'
        )

    def _on_odometry(self, msg):
        """Republish the pose on the reference topics only."""
        odometry = Odometry()
        odometry.header.stamp = msg.header.stamp
        odometry.header.frame_id = self._frame_id
        odometry.child_frame_id = 'reference_base'
        odometry.pose = msg.pose
        odometry.twist = msg.twist
        self._odom_publisher.publish(odometry)

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self._frame_id
        pose.pose = msg.pose.pose
        self._pose_publisher.publish(pose)


def main(args=None):
    """Run the node until shutdown."""
    # Keep rclpy's signal handler from shutting the context down mid-spin so
    # Ctrl+C raises KeyboardInterrupt and the node is torn down cleanly.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SimReferencePose()
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
