"""Closed-loop, fixed-odom-line tunnel traversal for an omnidirectional chassis."""
import math
import threading
import time
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sentry_motion.pointcloud import xyz_points
from tf2_ros import Buffer, TransformListener
from sentry_interfaces.action import TraverseTunnel
from sentry_interfaces.msg import MotionState
from sentry_motion.line_geometry import (
    line_error,
    body_velocity,
    validate_tunnel_segment,
)
from sentry_motion.obstacle_geometry import blocked_by_points


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix(tf):
    return rotation(tf.transform.rotation)


class TunnelController(Node):
    def __init__(self):
        super().__init__("tunnel_controller")
        self.declare_parameter("speed", 0.2)
        self.declare_parameter("max_lateral_error", 0.08)
        self.declare_parameter("timeout", 60.0)
        self.declare_parameter("lateral_kp", 3.0)
        self.declare_parameter("lateral_min_speed", 0.10)
        self.declare_parameter("lateral_max_speed", 0.12)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.group = ReentrantCallbackGroup()
        self.pub = self.create_publisher(Twist, "/sentry/cmd_vel_tunnel", 10)
        self.cloud = None
        self.odom = None
        self.cloud_at = self.odom_at = self.motion_at = -math.inf
        self.velocity = (0.0, 0.0)
        self.owner = "stop"
        self.lock = threading.Lock()
        topic = self.declare_parameter("obstacle_topic", "/terrain_map_odom").value
        self.create_subscription(
            PointCloud2, topic, self.on_cloud, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, "/odometry", self.on_odom, qos_profile_sensor_data
        )
        self.create_subscription(
            MotionState, "/sentry/motion_state", self.on_motion, 10
        )
        self.server = ActionServer(
            self,
            TraverseTunnel,
            "/sentry/traverse_tunnel",
            self.execute,
            goal_callback=self.goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.group,
        )

    def on_cloud(self, msg):
        self.cloud = msg
        self.cloud_at = time.monotonic() if msg.width * msg.height > 0 else -math.inf

    def on_odom(self, msg):
        age = (
            self.get_clock().now() - Time.from_msg(msg.header.stamp)
        ).nanoseconds * 1e-9
        self.odom_at = time.monotonic() if -0.1 <= age <= 0.5 else -math.inf
        v = msg.twist.twist
        if msg.header.frame_id != "odom" or msg.child_frame_id != "chassis":
            self.odom_at = -math.inf
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if (
            not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
            or abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1.0) > 0.01
        ):
            self.odom_at = -math.inf
        self.odom = msg
        self.velocity = (math.hypot(v.linear.x, v.linear.y), abs(v.angular.z))
        if not all(math.isfinite(x) for x in self.velocity):
            self.odom_at = -math.inf

    def on_motion(self, msg):
        self.owner, self.motion_at = msg.owner, time.monotonic()

    def goal(self, request):
        if not self.lock.acquire(blocking=False):
            return GoalResponse.REJECT
        try:
            a, b = request.entry.pose.position, request.exit.pose.position
            validate_tunnel_segment((a.x, a.y), (b.x, b.y))
            valid = (
                request.entry.header.frame_id == request.exit.header.frame_id == "map"
                and math.isfinite(a.z)
                and math.isfinite(b.z)
                and abs(a.z) < 0.01
                and abs(b.z) < 0.01
            )
        except ValueError:
            valid = False
        if not valid:
            self.lock.release()
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def transform_point(self, pose, tf):
        p = pose.pose.position
        v = matrix(tf) @ np.array([p.x, p.y, p.z])
        t = tf.transform.translation
        return (float(v[0] + t.x), float(v[1] + t.y))

    def blocked(self, stage, odom, cloud):
        age = (
            self.get_clock().now() - Time.from_msg(cloud.header.stamp)
        ).nanoseconds * 1e-9
        if not cloud.header.frame_id or not -0.1 <= age <= 0.5:
            raise RuntimeError("Obstacle frame or timestamp invalid")
        pts = xyz_points(cloud)
        if pts.size == 0:
            raise RuntimeError("No finite obstacle observations")
        if cloud.header.frame_id != "odom":
            cloud_tf = self.buffer.lookup_transform(
                "odom", cloud.header.frame_id, Time.from_msg(cloud.header.stamp)
            )
            ct = cloud_tf.transform.translation
            pts = np.einsum(
                "ij,nj->ni", matrix(cloud_tf), pts, optimize=False
            ) + np.array([ct.x, ct.y, ct.z])
        p = odom.pose.pose.position
        pts -= np.array([p.x, p.y, p.z])
        pts = np.einsum(
            "ij,nj->ni", rotation(odom.pose.pose.orientation).T, pts, optimize=False
        )
        return blocked_by_points(pts, aligning=stage == "align")

    def execute(self, handle):
        result = TraverseTunnel.Result()
        previous = np.zeros(3)
        stable = None
        stage = "align"
        data_wait_started = None
        started = time.monotonic()
        last_error = None
        best_remaining = math.inf
        progress_at = started
        try:
            while not (
                self.buffer.can_transform("odom", "map", Time())
                and self.buffer.can_transform("odom", "chassis", Time())
            ):
                if handle.is_cancel_requested:
                    handle.canceled()
                    result.message = "Canceled"
                    return result
                if time.monotonic() - started > 3.0:
                    raise RuntimeError(
                        "Initial map/odom/chassis transforms unavailable"
                    )
                self.pub.publish(Twist())
                time.sleep(0.05)
            tf = self.buffer.lookup_transform("odom", "map", Time())
            start = self.transform_point(handle.request.entry, tf)
            end = self.transform_point(handle.request.exit, tf)
            while rclpy.ok():
                now = time.monotonic()
                if handle.is_cancel_requested:
                    handle.canceled()
                    result.message = "Canceled"
                    return result
                if now - started > float(self.get_parameter("timeout").value):
                    raise RuntimeError("Tunnel timeout")
                if now - self.motion_at > 0.5:
                    raise RuntimeError("Motion state expired")
                if self.owner != "tunnel":
                    raise RuntimeError("Tunnel ownership lost")
                odom = self.odom
                if odom is None:
                    raise RuntimeError("No chassis odometry")
                age = (
                    self.get_clock().now() - Time.from_msg(odom.header.stamp)
                ).nanoseconds * 1e-9
                if age < -0.1:
                    raise RuntimeError("Chassis transform is ahead of simulation clock")
                cloud = self.cloud
                cloud_at = self.cloud_at
                cloud_age = (
                    (
                        self.get_clock().now() - Time.from_msg(cloud.header.stamp)
                    ).nanoseconds
                    * 1e-9
                    if cloud
                    else math.inf
                )
                fresh = (
                    age <= 0.3
                    and now - self.odom_at <= 0.5
                    and now - cloud_at <= 0.5
                    and -0.1 <= cloud_age <= 0.5
                )
                if fresh and cloud.header.frame_id != "odom":
                    fresh = self.buffer.can_transform(
                        "odom",
                        cloud.header.frame_id,
                        Time.from_msg(cloud.header.stamp),
                    )
                if not fresh:
                    self.pub.publish(Twist())
                    previous[:] = 0.0
                    if data_wait_started is None:
                        data_wait_started = now
                    if now - data_wait_started > 2.0:
                        raise RuntimeError(
                            f"Pose or obstacle data unavailable for 2s (odometry age {age:.3f}s)"
                        )
                    feedback = TraverseTunnel.Feedback()
                    feedback.stage = "waiting_data"
                    if last_error is not None:
                        feedback.lateral_error = last_error.lateral
                        feedback.heading_error = last_error.heading
                        feedback.remaining = last_error.remaining
                    handle.publish_feedback(feedback)
                    time.sleep(0.05)
                    continue
                if data_wait_started is not None:
                    progress_at += now - data_wait_started
                    data_wait_started = None
                p, q = odom.pose.pose.position, odom.pose.pose.orientation
                heading = yaw(q)
                e = line_error(start, end, (p.x, p.y, heading))
                last_error = e
                if abs(e.lateral) > (
                    0.2
                    if stage == "align"
                    else float(self.get_parameter("max_lateral_error").value)
                ):
                    raise RuntimeError("Lateral deviation exceeds passage allowance")
                if stage == "follow" and abs(e.heading) > math.radians(10):
                    raise RuntimeError("Heading deviation exceeds passage allowance")
                if self.blocked(stage, odom, cloud):
                    raise RuntimeError("Obstacle inside tunnel swept area")
                if e.remaining < best_remaining - 0.03:
                    best_remaining, progress_at = e.remaining, now
                if stage == "follow" and e.remaining > 0.05 and now - progress_at > 5:
                    raise RuntimeError("No forward progress")
                max_lateral = float(self.get_parameter("lateral_max_speed").value)
                lateral = max(
                    -max_lateral,
                    min(
                        max_lateral,
                        -float(self.get_parameter("lateral_kp").value) * e.lateral,
                    ),
                )
                if abs(e.lateral) > 0.01:
                    minimum = float(self.get_parameter("lateral_min_speed").value)
                    lateral = math.copysign(max(abs(lateral), minimum), lateral)
                else:
                    lateral = 0.0
                angular = max(-0.2, min(0.2, -1.2 * e.heading))
                if abs(e.heading) > math.radians(1) and abs(angular) < 0.06:
                    angular = math.copysign(0.06, angular)
                forward = 0.0
                if stage == "align":
                    forward = max(-0.05, min(0.05, -e.along))
                    ready = (
                        abs(e.along) < 0.03
                        and abs(e.lateral) < 0.02
                        and abs(e.heading) < math.radians(2)
                    )
                    stable = (stable if stable is not None else now) if ready else None
                    if stable is not None and now - stable >= 0.5:
                        stage = "follow"
                        progress_at = now
                elif stage == "follow":
                    forward = max(
                        0.0,
                        min(
                            float(self.get_parameter("speed").value), 0.7 * e.remaining
                        ),
                    )
                    if e.remaining < -0.05:
                        raise RuntimeError("Exit overshoot")
                    if (
                        abs(e.remaining) < 0.03
                        and abs(e.lateral) < 0.02
                        and abs(e.heading) < math.radians(2)
                    ):
                        stage = "settle"
                        stable = None
                else:
                    lateral = angular = 0.0
                    stopped = self.velocity[0] < 0.02 and self.velocity[1] < 0.03
                    stable = (
                        (stable if stable is not None else now) if stopped else None
                    )
                    if stable is not None and now - stable >= 0.3:
                        result.success = True
                        result.message = "Complete chassis outside tunnel"
                        handle.succeed()
                        return result
                vx, vy = body_velocity(start, end, heading, forward, lateral)
                target = np.array([vx, vy, angular])
                previous += np.clip(target - previous, -0.02, 0.02)
                msg = Twist()
                msg.linear.x, msg.linear.y, msg.angular.z = map(float, previous)
                self.pub.publish(msg)
                feedback = TraverseTunnel.Feedback()
                (
                    feedback.stage,
                    feedback.lateral_error,
                    feedback.heading_error,
                    feedback.remaining,
                ) = (stage, e.lateral, e.heading, e.remaining)
                handle.publish_feedback(feedback)
                time.sleep(0.05)
        except Exception as error:
            result.message = str(error)
            handle.abort()
            return result
        finally:
            if rclpy.ok():
                self.pub.publish(Twist())
            self.lock.release()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TunnelController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
