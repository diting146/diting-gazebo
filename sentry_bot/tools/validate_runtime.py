#!/usr/bin/env python3
"""Repeat real ROS actions against one already-running UL business simulation.

Reference pose is read ONLY for acceptance measurements; it never drives motion.
Any failed run stops the suite and latches chassis stop. No retries hide failures.
"""
import argparse
import json
import math
import threading
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from sentry_interfaces.action import OpeningMission, TraverseTunnel
from sentry_interfaces.msg import MotionState
from sentry_interfaces.srv import SetMotionOwner

ROOT = Path(__file__).resolve().parents[1]


class RuntimeProbe(Node):
    def __init__(self):
        super().__init__(
            "sentry_runtime_acceptance",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.motion = None
        self.motion_at = 0.0
        self.direct_active = False
        self.generation = None
        self.current = None
        self.reference = None
        self.reference_at = 0.0
        self.owner_events = []
        self.heartbeat = self.create_publisher(Empty, "/sentry/mission_heartbeat", 10)
        self.create_subscription(
            MotionState, "/sentry/motion_state", self.on_motion, 10
        )
        self.create_subscription(
            PoseStamped, "/reference/pose", self.on_reference, qos_profile_sensor_data
        )
        self.create_timer(0.05, self.tick)
        self.select_client = self.create_client(
            SetMotionOwner, "/sentry/set_motion_owner"
        )
        self.stop_client = self.create_client(Trigger, "/sentry/stop")
        self.opening = ActionClient(self, OpeningMission, "/sentry/opening_mission")
        self.tunnel = ActionClient(self, TraverseTunnel, "/sentry/traverse_tunnel")
        self.nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.active_child = None

    def on_motion(self, msg):
        if self.motion is None or (self.motion.owner, self.motion.generation) != (
            msg.owner,
            msg.generation,
        ):
            self.owner_events.append(
                {
                    "time": time.monotonic(),
                    "owner": msg.owner,
                    "generation": msg.generation,
                    "reason": msg.reason,
                }
            )
        self.motion, self.motion_at = msg, time.monotonic()

    def on_reference(self, msg):
        self.reference, self.reference_at = msg, time.monotonic()
        if self.current is None:
            return
        p, q = msg.pose.position, msg.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
            self.current["reference_invalid"] = True
            return
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        hx = 0.288 * abs(math.cos(yaw)) + 0.266 * abs(math.sin(yaw))
        hy = 0.288 * abs(math.sin(yaw)) + 0.266 * abs(math.cos(yaw))
        if p.x + hx >= -2.675 and p.x - hx <= -2.325 and p.y > 3.0:
            margin = min(p.y - 3.06, 3.86 - p.y) - hy
            self.current["min_reference_wall_clearance"] = min(
                self.current["min_reference_wall_clearance"], margin
            )
            self.current["reference_samples_in_tunnel"] += 1
        self.current["last_reference"] = [p.x, p.y, yaw]

    def tick(self):
        if self.direct_active:
            self.heartbeat.publish(Empty())

    @staticmethod
    def wait(future, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise RuntimeError("ROS response deadline exceeded")
            time.sleep(0.02)
        return future.result()

    def select(self, owner):
        request = SetMotionOwner.Request()
        request.owner = owner
        request.expected_generation = self.generation
        response = self.wait(self.select_client.call_async(request))
        if not response.success:
            raise RuntimeError(response.message)
        self.generation = response.generation
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            if (
                self.motion
                and self.motion.generation == self.generation
                and self.motion.owner == owner
            ):
                return
            time.sleep(0.02)
        raise RuntimeError("No ownership acknowledgement")

    def stop_and_wait(self):
        self.select("stop")
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            if (
                self.motion
                and self.motion.stopped
                and self.motion.owner == "stop"
                and time.monotonic() - self.motion_at < 0.5
            ):
                return
            time.sleep(0.05)
        raise RuntimeError("Chassis failed to stop")

    def feedback(self, message):
        if self.current is None:
            return
        f = message.feedback
        stage = getattr(f, "stage", "nav")
        if isinstance(stage, str) and stage.endswith("follow"):
            self.current["follow_samples"] += 1
            self.current["max_follow_lateral_error"] = max(
                self.current["max_follow_lateral_error"], abs(f.lateral_error)
            )
            self.current["max_follow_heading_error"] = max(
                self.current["max_follow_heading_error"], abs(f.heading_error)
            )
        if self.current.get("last_stage") != stage:
            self.current["stages"].append(stage)
            self.current["last_stage"] = stage
            print(self.current["kind"], self.current["index"], stage, flush=True)

    def action(self, client, goal, timeout):
        if not client.wait_for_server(timeout_sec=5):
            raise RuntimeError("Action server unavailable")
        sent = client.send_goal_async(goal, feedback_callback=self.feedback)
        try:
            child = self.wait(sent)
        except Exception:
            sent.add_done_callback(
                lambda f: f.result().cancel_goal_async()
                if f.result() and f.result().accepted
                else None
            )
            raise
        if not child.accepted:
            raise RuntimeError("Action rejected")
        self.active_child = child
        response = self.wait(child.get_result_async(), timeout)
        self.active_child = None
        if response.status != 4 or not getattr(response.result, "success", True):
            raise RuntimeError(
                getattr(response.result, "message", str(response.status))
            )

    def emergency_stop(self):
        if self.stop_client.service_is_ready():
            try:
                self.wait(self.stop_client.call_async(Trigger.Request()), 3)
            except Exception:
                pass
        if self.active_child:
            try:
                self.wait(self.active_child.cancel_goal_async(), 3)
            except Exception:
                pass


def pose(point):
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.pose.position.x = float(point["x"])
    msg.pose.position.y = float(point["y"])
    yaw = float(point.get("yaw", 0.0))
    msg.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.orientation.w = math.cos(yaw / 2)
    return msg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--tunnel-pairs", type=int, default=10)
    parser.add_argument("--opening-runs", type=int, default=10)
    parser.add_argument("--output", default="runtime_acceptance.json")
    parser.add_argument(
        "--localization", choices=("lio", "ground_truth"), default="lio"
    )
    args = parser.parse_args()
    if min(args.tunnel_pairs, args.opening_runs) < 0:
        parser.error("Run counts must be nonnegative")
    route = yaml.safe_load(
        (ROOT / "src/sentry_business_bringup/config/opening_2027rmul.yaml").read_text()
    )
    output = ROOT / "validation" / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = RuntimeProbe()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    report = {
        "passed": False,
        "route": route,
        "model_changed": False,
        "localization": args.localization,
        "runs": [],
        "error": None,
    }
    try:
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline and (
            node.motion is None or not node.motion.stopped or node.reference is None
        ):
            time.sleep(0.1)
        if node.motion is None or not node.motion.stopped or node.reference is None:
            raise RuntimeError("Motion or reference diagnostics not ready")
        deadline = time.monotonic() + 10
        graph_valid = False
        while time.monotonic() < deadline:
            publishers = node.get_publishers_info_by_topic("/cmd_vel")
            motion_publishers = node.get_publishers_info_by_topic(
                "/sentry/motion_state"
            )
            report["cmd_vel_publishers"] = [p.node_name for p in publishers]
            matches = (
                len(publishers) == len(motion_publishers) == 1
                and bytes(publishers[0].endpoint_gid)[:12]
                == bytes(motion_publishers[0].endpoint_gid)[:12]
                and any(publishers[0].endpoint_gid)
            )
            report["single_writer_matches_motion_state_participant"] = bool(matches)
            known_name_valid = len(publishers) == 1 and publishers[0].node_name in (
                "velocity_arbiter",
                "_NODE_NAME_UNKNOWN_",
            )
            if matches and known_name_valid:
                graph_valid = True
                break
            time.sleep(0.1)
        if not graph_valid:
            raise RuntimeError(
                "Chassis writer must be the sole motion-arbiter participant"
            )
        for kind, count in [
            ("opening", args.opening_runs),
            ("tunnel", args.tunnel_pairs * 2),
        ]:
            for i in range(count):
                started = time.monotonic()
                run = {
                    "kind": kind,
                    "index": i + 1,
                    "success": False,
                    "stages": [],
                    "max_follow_lateral_error": 0.0,
                    "max_follow_heading_error": 0.0,
                    "follow_samples": 0,
                    "min_reference_wall_clearance": math.inf,
                    "reference_samples_in_tunnel": 0,
                }
                node.current = run
                report["runs"].append(run)
                try:
                    if kind == "opening":
                        node.direct_active = False
                        goal = OpeningMission.Goal()
                        goal.entry = pose(route["entry"])
                        goal.exit = pose(route["exit"])
                        goal.center = pose(route["center"])
                        node.action(node.opening, goal, 320)
                    else:
                        node.direct_active = True
                        node.generation = node.motion.generation
                        node.tick()
                        node.stop_and_wait()
                        a, b = (
                            (route["entry"], route["exit"])
                            if i % 2 == 0
                            else (route["exit"], route["entry"])
                        )
                        run["direction"] = "forward" if i % 2 == 0 else "reverse"
                        if i == 0:
                            node.select("nav")
                            goal = NavigateToPose.Goal()
                            goal.pose = pose(a)
                            node.action(node.nav, goal, 140)
                            node.stop_and_wait()
                        node.select("tunnel")
                        goal = TraverseTunnel.Goal()
                        goal.entry = pose(a)
                        goal.exit = pose(b)
                        node.action(node.tunnel, goal, 70)
                        node.stop_and_wait()
                    if (
                        run["follow_samples"] == 0
                        or run["reference_samples_in_tunnel"] == 0
                    ):
                        raise RuntimeError("Missing traversal/reference measurements")
                    if (
                        run["max_follow_lateral_error"] > 0.08
                        or run["min_reference_wall_clearance"] <= 0
                    ):
                        raise RuntimeError(
                            "Measured tracking or physical wall margin failed"
                        )
                    if run.get("reference_invalid"):
                        raise RuntimeError("Invalid reference measurements")
                    run["success"] = True
                    print(
                        "PASS",
                        kind,
                        i + 1,
                        "lateral",
                        run["max_follow_lateral_error"],
                        "wall_clearance",
                        run["min_reference_wall_clearance"],
                        flush=True,
                    )
                finally:
                    run["elapsed_wall"] = time.monotonic() - started
                    if not math.isfinite(run["min_reference_wall_clearance"]):
                        run["min_reference_wall_clearance"] = None
                    output.write_text(json.dumps(report, indent=2) + "\n")
                    node.current = None
                node.direct_active = False
                time.sleep(0.5)
        report["passed"] = True
    except Exception as error:
        report["error"] = str(error)
        node.emergency_stop()
        print("FAILED", error, flush=True)
    finally:
        report["owner_events"] = node.owner_events
        output.write_text(json.dumps(report, indent=2) + "\n")
        node.direct_active = False
        executor.shutdown()
        worker.join(timeout=3)
        node.destroy_node()
        rclpy.try_shutdown()
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
