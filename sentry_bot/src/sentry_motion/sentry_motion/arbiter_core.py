"""Fail-closed ownership and freshness rules; time uses monotonic seconds."""
import math


class ArbiterCore:
    def __init__(self, command_timeout=0.2, heartbeat_timeout=0.5):
        self.owner = "stop"
        self.reason = "Startup stop"
        self.command_timeout = command_timeout
        self.heartbeat_timeout = heartbeat_timeout
        self.commands = {}
        self.heartbeat = -math.inf
        self.stopped = False
        self.generation = 0

    def stop(self, reason):
        if self.owner != "stop" or self.reason != reason or reason == "Operator stop":
            self.generation += 1
        self.owner = "stop"
        self.commands.clear()
        self.reason = reason

    def select(self, owner, now, expected_generation=None):
        if expected_generation is not None and expected_generation != self.generation:
            return False, "Motion generation changed; start a new mission"
        if owner not in ("stop", "nav", "tunnel"):
            return False, "Unknown owner"
        if owner == "stop":
            self.stop("Requested stop")
            return True, self.reason
        if self.owner != "stop" or not self.stopped:
            return False, "Stop and confirm stationary before selecting owner"
        if now - self.heartbeat > self.heartbeat_timeout:
            return False, "Mission heartbeat missing"
        self.commands.clear()
        self.owner = owner
        self.selected_at = now
        self.reason = "Selected " + owner
        return True, self.reason

    def receive(self, owner, velocity, now):
        if owner != self.owner:
            return
        if not all(math.isfinite(v) for v in velocity):
            self.stop("Invalid velocity")
            return
        self.commands[owner] = (now, tuple(velocity))

    def output(self, now):
        if self.owner == "stop":
            return (0.0, 0.0, 0.0)
        if now - self.heartbeat > self.heartbeat_timeout:
            self.stop("Mission heartbeat expired")
            return (0.0, 0.0, 0.0)
        stamp, command = self.commands.get(
            self.owner, (self.selected_at, (0.0, 0.0, 0.0))
        )
        if now - stamp > (self.command_timeout if self.owner in self.commands else 3.0):
            self.stop("Velocity command expired")
            return (0.0, 0.0, 0.0)
        return command
