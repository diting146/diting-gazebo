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
Fake MCU gateway between Nav2 and the simulated sentry chassis.

The real sentry receives ``cmd_vel`` over a serial link from its MCU.  In
simulation the chassis is driven by the Gazebo-native ``MecanumDrive2`` plugin,
which subscribes to the Gazebo transport topic ``/<entity_name>/cmd_vel``
(``ignition.msgs.Twist``).  Python has no Gazebo transport bindings, so this
node republishes incoming ROS ``geometry_msgs/msg/Twist`` messages on a ROS
topic and ``ros_gz_bridge`` maps that topic onto the Gazebo transport topic.

When no command arrives for ``timeout_ms`` milliseconds the node behaves like
a watchdog and clears translation at ``publish_rate_hz``. Angular velocity
returns to ``idle_spin_speed`` (zero by default), allowing continuous small-gyro
rotation in simulation even when Nav2 has no active goal.
"""

import math

import rclpy
from example_interfaces.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


class SimSerialGateway(Node):
    """Republish ROS commands and use the configured idle spin on input timeout."""

    def __init__(self, **kwargs):
        super().__init__('sim_serial_gateway', **kwargs)

        input_topic = self.declare_parameter('input_topic', 'cmd_vel').value
        output_topic = self.declare_parameter(
            'output_topic', 'sentry_robot1/cmd_vel'
        ).value
        timeout_ms = int(self.declare_parameter('timeout_ms', 200).value)
        publish_rate_hz = float(
            self.declare_parameter('publish_rate_hz', 50.0).value
        )

        if timeout_ms < 0:
            raise ValueError('timeout_ms must be non-negative')
        if publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be positive')

        self._idle_spin_speed = float(
            self.declare_parameter('idle_spin_speed', 0.0).value
        )
        self._require_localization = bool(self.declare_parameter(
            'require_localization', False).value)
        self._localization_since_ns = None
        self._localization_last_ns = None
        self._angular_acceleration_limit = float(self.declare_parameter(
            'angular_acceleration_limit', 0.0).value)
        self._last_output_angular = 0.0
        self._last_output_ns = None
        self._timeout_seconds = timeout_ms / 1000.0
        self._last_received_ns = None
        self._publisher = self.create_publisher(Twist, output_topic, 10)
        self._subscription = self.create_subscription(
            Twist, input_topic, self._on_twist, 10
        )
        spin_topic = self.declare_parameter('spin_topic', '/cmd_spin').value
        self._spin_subscription = (
            self.create_subscription(Float32, spin_topic, self._on_spin, 10)
            if spin_topic else None
        )
        # Command expiry is elapsed real time, including while /clock is paused.
        self._watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        if self._require_localization:
            self.create_subscription(
                Odometry, self.declare_parameter('localization_topic', 'odometry').value,
                self._on_localization, 10)
        self._timer = self.create_timer(
            1.0 / publish_rate_hz, self._on_timer, clock=self._watchdog_clock
        )

        self.get_logger().info(
            f'sim_serial_gateway: {input_topic} -> {output_topic} '
            f'(timeout {timeout_ms} ms)'
        )

    def _on_localization(self, msg):
        """Use local odometry arrival only, never the Gazebo reference channel."""
        now = self._watchdog_clock.now().nanoseconds
        if self._localization_last_ns is None or now - self._localization_last_ns >= 1_000_000_000:
            self._localization_since_ns = now
        self._localization_last_ns = now

    def _localization_ready(self):
        """Require two seconds of odometry and a fresh sample in LIO mode."""
        if not self._require_localization:
            return True
        if self._localization_since_ns is None:
            return False
        now = self._watchdog_clock.now().nanoseconds
        return (now - self._localization_since_ns > 2_000_000_000
                and now - self._localization_last_ns < 1_000_000_000)

    def _publish_command(self, msg):
        """Ramp nonzero spin requests; an explicit stop remains immediate."""
        now = self._watchdog_clock.now().nanoseconds
        dt = 0.02 if self._last_output_ns is None else max(
            0.0, (now - self._last_output_ns) * 1e-9)
        self._last_output_ns = now
        output = Twist()
        output.linear = msg.linear
        output.angular.x = msg.angular.x
        output.angular.y = msg.angular.y
        target = msg.angular.z
        if target != 0.0 and self._angular_acceleration_limit > 0.0:
            step = self._angular_acceleration_limit * min(dt, 0.2)
            target = self._last_output_angular + max(
                -step, min(step, target - self._last_output_angular))
        self._last_output_angular = target
        output.angular.z = target
        self._publisher.publish(output)

    def _on_spin(self, msg):
        """Use the same spin request as fake_vel_transform during idle."""
        if not math.isfinite(msg.data):
            self.get_logger().warning('Ignoring non-finite spin speed')
            return
        self._idle_spin_speed = float(msg.data)
        self.get_logger().info(f'Idle spin speed: {self._idle_spin_speed:.3f} rad/s')
        self._on_timer()

    def _on_twist(self, msg):
        """Forward an incoming command and remember when it arrived."""
        self._last_received_ns = self._watchdog_clock.now().nanoseconds
        self._publish_command(msg if self._localization_ready() else Twist())

    def _on_timer(self):
        """Clear translation and apply the configured spin while input is silent."""
        idle_command = Twist()
        idle_command.angular.z = (
            self._idle_spin_speed if self._localization_ready() else 0.0)
        now_ns = self._watchdog_clock.now().nanoseconds
        if self._last_received_ns is None:
            self._publish_command(idle_command)
            return

        age_seconds = (now_ns - self._last_received_ns) * 1e-9
        if age_seconds >= self._timeout_seconds:
            self._publish_command(idle_command)


def main(args=None):
    """Run the gateway until shutdown."""
    # Keep rclpy from shutting the context down inside its signal handler;
    # Ctrl+C then raises KeyboardInterrupt and the node is torn down cleanly.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SimSerialGateway()
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
