#!/usr/bin/env python3
"""Live cancellation and physical obstacle-stop checks on the existing simulation."""
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
import yaml
from sentry_interfaces.action import OpeningMission
from validate_runtime import ROOT, RuntimeProbe, pose


def new_case(kind):
    return {
        "kind": kind,
        "index": 1,
        "stages": [],
        "max_follow_lateral_error": 0.0,
        "max_follow_heading_error": 0.0,
        "follow_samples": 0,
        "min_reference_wall_clearance": math.inf,
        "reference_samples_in_tunnel": 0,
    }


def stopped(node):
    deadline = time.monotonic() + 6
    while time.monotonic() < deadline:
        if node.motion and node.motion.owner == "stop" and node.motion.stopped:
            return True
        time.sleep(0.05)
    return False


def main():
    route = yaml.safe_load(
        (ROOT / "src/sentry_business_bringup/config/opening_2027rmul.yaml").read_text()
    )
    rclpy.init()
    node = RuntimeProbe()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    report = {"passed": False, "cases": [], "error": None, "model_changed": False}
    fixture = "sentry_validation_blocker_" + str(os.getpid())
    spawned = False

    def goal():
        g = OpeningMission.Goal()
        g.entry = pose(route["entry"])
        g.exit = pose(route["exit"])
        g.center = pose(route["center"])
        return g

    try:
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline and not (node.motion and node.motion.stopped):
            time.sleep(0.1)
        assert (
            node.motion and node.motion.stopped
        ), "Localization and stationary chassis not ready"
        assert node.opening.wait_for_server(
            timeout_sec=10
        ), "Opening action unavailable"
        case = new_case("operator_cancel")
        node.current = case
        report["cases"].append(case)

        def cancel_while_moving():
            deadline = time.monotonic() + 140
            follow_start = None
            while time.monotonic() < deadline and node.current is case:
                if "tunnel:follow" in case["stages"] and node.reference:
                    p = node.reference.pose.position
                    if follow_start is None:
                        follow_start = (p.x, p.y)
                    traveled = math.hypot(p.x - follow_start[0], p.y - follow_start[1])
                    if traveled >= 0.05 and node.active_child:
                        case["measured_travel_before_cancel"] = traveled
                        case["cancel_requested_at"] = time.monotonic()
                        node.wait(node.active_child.cancel_goal_async(), 3)
                        return
                time.sleep(0.02)

        canceller = threading.Thread(target=cancel_while_moving, daemon=True)
        canceller.start()
        try:
            node.action(node.opening, goal(), 160)
            raise AssertionError("Mission unexpectedly completed after cancellation")
        except RuntimeError as error:
            case["result"] = str(error)
        assert "cancel_requested_at" in case, "Cancel trigger never reached"
        assert stopped(node), "Chassis not stopped after cancellation"
        case["stop_confirmed_after"] = time.monotonic() - case["cancel_requested_at"]
        case["passed"] = True
        node.current = None
        canceller.join(timeout=1)
        create = subprocess.run(
            [
                "ros2",
                "run",
                "ros_gz_sim",
                "create",
                "-world",
                "default",
                "-file",
                str(ROOT / "tools/fixtures/tunnel_blocker.sdf"),
                "-name",
                fixture,
                "-allow_renaming",
                "false",
                "-x",
                "-2.5",
                "-y",
                "3.46",
                "-z",
                "0.20",
            ],
            capture_output=True,
            text=True,
            timeout=20,
        )
        report["fixture_create_output"] = create.stdout + create.stderr
        assert create.returncode == 0, "Fixture creation failed"
        spawned = True
        observed = subprocess.run(
            [
                "ign",
                "topic",
                "-e",
                "-t",
                "/world/default/pose/info",
                "-n",
                "1",
                "--json-output",
            ],
            capture_output=True,
            text=True,
            timeout=10,
        )
        entities = json.loads(observed.stdout)
        placed = next(
            (p for p in entities.get("pose", []) if p.get("name") == fixture), None
        )
        assert placed is not None, "Fixture missing from Gazebo pose inventory"
        report["fixture_observed_pose"] = placed
        actual = placed.get("position", {})
        assert all(
            abs(actual.get(k, 0) - v) < 0.001
            for k, v in [("x", -2.5), ("y", 3.46), ("z", 0.2)]
        ), "Fixture is not inside the intended tunnel"
        case = new_case("physical_tunnel_obstacle")
        node.current = case
        report["cases"].append(case)
        try:
            node.action(node.opening, goal(), 160)
            raise AssertionError("Mission passed through the physical blocker")
        except RuntimeError as error:
            case["result"] = str(error)
        assert "Obstacle inside tunnel swept area" in case["result"], case["result"]
        assert case["follow_samples"] > 0, "Did not exercise tunnel controller"
        assert stopped(node), "Chassis not stopped at physical blocker"
        p = node.reference.pose.position
        assert p.x + 0.288 < -2.65, "Chassis reached blocker before stopping"
        case["reference_stop_position"] = [p.x, p.y]
        case["passed"] = True
        report["passed"] = True
    except Exception as error:
        report["error"] = str(error)
        node.emergency_stop()
    finally:
        node.current = None
        if spawned:
            remove = subprocess.run(
                [
                    "ign",
                    "service",
                    "-s",
                    "/world/default/remove",
                    "--reqtype",
                    "ignition.msgs.Entity",
                    "--reptype",
                    "ignition.msgs.Boolean",
                    "--timeout",
                    "5000",
                    "--req",
                    f'name:"{fixture}" type:MODEL',
                ],
                capture_output=True,
                text=True,
                timeout=8,
            )
            report["fixture_remove_output"] = remove.stdout + remove.stderr
            if remove.returncode != 0 or "true" not in remove.stdout:
                report["passed"] = False
                report["error"] = "Fixture removal not confirmed"
        for case in report["cases"]:
            if not math.isfinite(case["min_reference_wall_clearance"]):
                case["min_reference_wall_clearance"] = None
        (ROOT / "validation/live_stop_checks.json").write_text(
            json.dumps(report, indent=2) + "\n"
        )
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
