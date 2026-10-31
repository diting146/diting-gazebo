"""Single chassis command writer for business-mode operation."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from sentry_interfaces.msg import MotionState
from sentry_interfaces.srv import SetMotionOwner
from sentry_motion.arbiter_core import ArbiterCore


class VelocityArbiter(Node):
    def __init__(self):
        super().__init__("velocity_arbiter")
        self.core = ArbiterCore(
            command_timeout=float(self.declare_parameter("command_timeout", 0.5).value)
        )
        self.last_odom = -math.inf
        self.last_odom_stamp = None
        self.data_wait_since = None
        self.stationary_since = None
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.state_pub = self.create_publisher(MotionState, "/sentry/motion_state", 10)
        for owner in ("nav", "tunnel"):
            self.create_subscription(
                Twist,
                "/sentry/cmd_vel_" + owner,
                lambda msg, o=owner: self.command(o, msg),
                10,
            )
        self.create_subscription(Empty, "/sentry/mission_heartbeat", self.beat, 10)
        self.create_subscription(
            Odometry, "/odometry", self.odom, qos_profile_sensor_data
        )
        self.create_service(SetMotionOwner, "/sentry/set_motion_owner", self.select)
        self.create_service(Trigger, "/sentry/stop", self.stop)
        self.create_timer(
            0.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    def beat(self, msg):
        self.core.heartbeat = time.monotonic()

    def odom(self, msg):
        v = msg.twist.twist
        age = (
            self.get_clock().now().nanoseconds
            - (msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec)
        ) * 1e-9
        valid_values = all(
            math.isfinite(x) for x in (v.linear.x, v.linear.y, v.angular.z)
        )
        if (
            not valid_values
            or age < -0.1
            or age > 2.0
            or msg.header.frame_id != "odom"
            or msg.child_frame_id != "chassis"
        ):
            self.last_odom = -math.inf
            self.last_odom_stamp = None
            self.core.stop("Invalid or replayed odometry")
            return
        # Brief delayed samples do not refresh the accepted timestamp. The
        # timer holds zero output while waiting for genuinely fresh data.
        if age > 0.5:
            return
        valid = True
        now = time.monotonic()
        self.last_odom = now
        self.last_odom_stamp = msg.header.stamp
        stationary = (
            valid
            and math.hypot(v.linear.x, v.linear.y) < 0.02
            and abs(v.angular.z) < 0.03
        )
        if not stationary:
            self.stationary_since = None
        elif self.stationary_since is None:
            self.stationary_since = now
        self.core.stopped = (
            self.stationary_since is not None and now - self.stationary_since >= 0.3
        )

    def odometry_fresh(self):
        if self.last_odom_stamp is None or time.monotonic() - self.last_odom > 1.0:
            return False
        stamp = self.last_odom_stamp.sec * 1000000000 + self.last_odom_stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) * 1e-9
        return -0.1 <= age <= 0.5

    def command(self, owner, msg):
        self.core.receive(
            owner, (msg.linear.x, msg.linear.y, msg.angular.z), time.monotonic()
        )

    def select(self, request, response):
        now = time.monotonic()
        if not self.odometry_fresh():
            self.core.stopped = False
        response.success, response.message = self.core.select(
            request.owner, now, request.expected_generation
        )
        response.generation = self.core.generation
        return response

    def stop(self, request, response):
        self.core.stop("Operator stop")
        response.success = True
        response.message = self.core.reason
        return response

    def tick(self):
        now = time.monotonic()
        fresh = self.odometry_fresh()
        if not fresh:
            self.core.stopped = False
            self.stationary_since = None
            if self.data_wait_since is None:
                self.data_wait_since = now
            if self.core.owner != "stop" and now - self.data_wait_since > 2.0:
                self.core.stop("Odometry expired")
        else:
            self.data_wait_since = None
        vx, vy, wz = self.core.output(now)
        if not fresh:
            vx = vy = wz = 0.0
        # Absolute chassis limits independent of the selected producer.
        scale = min(1.0, 0.6 / max(math.hypot(vx, vy), 1e-9))
        msg = Twist()
        msg.linear.x, msg.linear.y = vx * scale, vy * scale
        msg.angular.z = max(-0.5, min(0.5, wz))
        self.pub.publish(msg)
        state = MotionState()
        state.stamp = self.get_clock().now().to_msg()
        state.generation = self.core.generation
        state.owner, state.stopped, state.reason = (
            self.core.owner,
            self.core.stopped,
            self.core.reason,
        )
        if not fresh and self.core.owner != "stop":
            state.reason = "Waiting for fresh odometry; output held at zero"
        self.state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
