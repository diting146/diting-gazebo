"""Operator-started Nav2 / tunnel / Nav2 mission with exclusive ownership."""
import math
import threading
import time
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from example_interfaces.msg import Float32
from sentry_interfaces.action import OpeningMission, TraverseTunnel
from sentry_interfaces.msg import MotionState
from sentry_interfaces.srv import SetMotionOwner
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy
from sentry_mission.route_validation import validate_route, validate_map_points


class OpeningMissionNode(Node):
    def __init__(self):
        super().__init__("opening_mission")
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.running = False
        self.generation = None
        self.child = None
        self.child_result = None
        self.pending_goal = None
        self.motion = None
        self.grid = None
        self.create_subscription(
            OccupancyGrid,
            "/map",
            self.on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.motion_at = -math.inf
        self.heartbeat = self.create_publisher(Empty, "/sentry/mission_heartbeat", 10)
        self.spin_pub = self.create_publisher(Float32, "/cmd_spin", 10)
        self.nav = ActionClient(
            self, NavigateToPose, "/navigate_to_pose", callback_group=self.group
        )
        self.tunnel = ActionClient(
            self, TraverseTunnel, "/sentry/traverse_tunnel", callback_group=self.group
        )
        self.owner = self.create_client(
            SetMotionOwner, "/sentry/set_motion_owner", callback_group=self.group
        )
        self.nav_ready = self.create_client(
            Trigger,
            "/lifecycle_manager_navigation/is_active",
            callback_group=self.group,
        )
        self.stop_client = self.create_client(
            Trigger, "/sentry/stop", callback_group=self.group
        )
        self.create_subscription(
            MotionState, "/sentry/motion_state", self.on_motion, 10
        )
        self.create_timer(0.05, self.tick)
        self.server = ActionServer(
            self,
            OpeningMission,
            "/sentry/opening_mission",
            self.execute,
            goal_callback=self.goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.group,
        )

    def on_map(self, msg):
        self.grid = msg

    def on_motion(self, msg):
        self.motion, self.motion_at = msg, time.monotonic()

    def tick(self):
        if self.running:
            self.heartbeat.publish(Empty())
            self.spin_pub.publish(Float32(data=0.0))

    def goal(self, request):
        if self.pending_goal is not None or (
            self.child_result is not None and not self.child_result.done()
        ):
            self.get_logger().warning("Previous child action has not terminated")
            return GoalResponse.REJECT
        try:
            validate_route(request.entry, request.exit, request.center)
            validate_map_points(
                (request.entry, request.exit, request.center), self.grid
            )
        except ValueError as error:
            self.get_logger().warning(str(error))
            return GoalResponse.REJECT
        return (
            GoalResponse.ACCEPT
            if self.lock.acquire(blocking=False)
            else GoalResponse.REJECT
        )

    def wait(self, future, timeout=5.0):
        limit = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() > limit:
                raise RuntimeError("ROS request timeout")
            time.sleep(0.02)
        return future.result()

    def select(self, owner):
        if not self.owner.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("Motion arbiter unavailable")
        request = SetMotionOwner.Request()
        request.owner = owner
        request.expected_generation = self.generation
        response = self.wait(self.owner.call_async(request))
        if not response.success:
            raise RuntimeError(response.message)
        self.generation = response.generation
        limit = time.monotonic() + 1
        while time.monotonic() < limit:
            if (
                self.motion
                and self.motion.generation == self.generation
                and self.motion.owner == owner
            ):
                return
            time.sleep(0.02)
        raise RuntimeError("Ownership state not confirmed")

    def stop_and_wait(self):
        self.select("stop")
        limit = time.monotonic() + 6
        while time.monotonic() < limit:
            if (
                self.motion
                and time.monotonic() - self.motion_at < 0.3
                and self.motion.owner == "stop"
                and self.motion.stopped
            ):
                return
            time.sleep(0.05)
        raise RuntimeError("Chassis did not confirm stationary")

    def feedback(self, handle, stage):
        msg = OpeningMission.Feedback()
        msg.stage = stage
        handle.publish_feedback(msg)

    def run_child(self, handle, client, goal, stage, timeout):
        if handle.is_cancel_requested:
            raise RuntimeError("Mission canceled")
        if not client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError(stage + " action unavailable")
        future = client.send_goal_async(
            goal, feedback_callback=lambda m: self.child_feedback(handle, stage, m)
        )
        self.pending_goal = future
        # Late acceptance after timeout must not leave an unowned Nav2 action alive.
        try:
            self.child = self.wait(future)
        except Exception:
            future.add_done_callback(self.cancel_late_goal)
            raise
        if not self.child.accepted:
            self.pending_goal = None
            raise RuntimeError(stage + " goal rejected")
        result = self.child.get_result_async()
        self.child_result = result
        self.pending_goal = None
        limit = time.monotonic() + timeout
        while not result.done():
            if handle.is_cancel_requested or time.monotonic() > limit:
                raise RuntimeError(
                    "Mission canceled"
                    if handle.is_cancel_requested
                    else stage + " timeout"
                )
            if (
                self.motion is None
                or time.monotonic() - self.motion_at > 0.5
                or self.motion.owner == "stop"
                or self.motion.generation != self.generation
            ):
                raise RuntimeError(
                    "Motion stopped: "
                    + (self.motion.reason if self.motion else "no arbiter state")
                )
            time.sleep(0.05)
        response = result.result()
        self.child = None
        if response.status != 4:
            raise RuntimeError(
                stage
                + " failed: "
                + getattr(response.result, "message", str(response.status))
            )

    def cancel_late_goal(self, future):
        try:
            child = future.result()
            if child and child.accepted:
                self.child_result = child.get_result_async()
                child.cancel_goal_async()
                self.child_result.add_done_callback(
                    lambda _: setattr(self, "pending_goal", None)
                )
            else:
                self.pending_goal = None
        except Exception as error:
            # Keep the pending latch on transport failure: a new mission must
            # not reuse a potentially live old Nav2 producer.
            self.get_logger().error("Late goal cleanup failed: " + str(error))

    def child_feedback(self, handle, stage, message):
        msg = OpeningMission.Feedback()
        msg.stage = stage
        f = message.feedback
        if stage == "tunnel":
            msg.stage += ":" + f.stage
            msg.lateral_error, msg.heading_error, msg.remaining = (
                f.lateral_error,
                f.heading_error,
                f.remaining,
            )
        else:
            msg.remaining = f.distance_remaining
        handle.publish_feedback(msg)

    def wait_navigation_ready(self, handle):
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            if handle.is_cancel_requested:
                raise RuntimeError("Mission canceled")
            if self.motion is None or self.motion.generation != self.generation:
                raise RuntimeError("Motion stopped while waiting for navigation")
            if self.nav_ready.service_is_ready():
                response = self.wait(self.nav_ready.call_async(Trigger.Request()), 2)
                if response.success:
                    return
            self.feedback(handle, "waiting_navigation")
            time.sleep(0.1)
        raise RuntimeError("Navigation is not active")

    def execute(self, handle):
        result = OpeningMission.Result()
        self.running = True
        try:
            if self.motion is None or time.monotonic() - self.motion_at > 0.5:
                raise RuntimeError("Fresh motion state required")
            self.generation = self.motion.generation
            self.tick()
            self.stop_and_wait()
            self.wait_navigation_ready(handle)
            self.select("nav")
            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = handle.request.entry
            self.run_child(handle, self.nav, nav_goal, "entry", 120)
            self.stop_and_wait()
            self.select("tunnel")
            tunnel_goal = TraverseTunnel.Goal()
            tunnel_goal.entry, tunnel_goal.exit = (
                handle.request.entry,
                handle.request.exit,
            )
            self.run_child(handle, self.tunnel, tunnel_goal, "tunnel", 65)
            self.stop_and_wait()
            self.wait_navigation_ready(handle)
            self.select("nav")
            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = handle.request.center
            self.run_child(handle, self.nav, nav_goal, "center", 120)
            self.stop_and_wait()
            result.success = True
            result.message = "Center reached"
            handle.succeed()
        except Exception as error:
            result.message = str(error)
            try:
                if self.stop_client.service_is_ready():
                    self.wait(self.stop_client.call_async(Trigger.Request()), 3)
            except Exception as stop_error:
                self.get_logger().error(str(stop_error))
            try:
                if self.child and self.child.accepted:
                    self.wait(self.child.cancel_goal_async(), 3)
                    if self.child_result is not None:
                        self.wait(self.child_result, 3)
            except Exception as cancel_error:
                self.get_logger().error("Child cleanup pending: " + str(cancel_error))
            if handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
        finally:
            self.running = False
            self.spin_pub.publish(Float32(data=0.0))
            self.lock.release()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = OpeningMissionNode()
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
