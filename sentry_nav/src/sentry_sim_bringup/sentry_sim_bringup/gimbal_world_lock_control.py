# Copyright 2026 diting
# Licensed under the Apache License, Version 2.0 (the "License").

"""World-yaw hold math, independent of ROS and simulator transport."""

import math


def wrap_angle(angle):
    """Return the signed shortest angular difference."""
    return math.atan2(math.sin(angle), math.cos(angle))


class WorldYawHold:
    """Track one world-yaw reference using fresh chassis and joint feedback.

    ``feedforward_time_sec`` compensates the simulated position servo's measured
    tracking lag. It is a simulator calibration, not a hardware controller gain.
    The bounded integral removes the remaining position/friction error.
    """

    def __init__(
        self, feedforward_time_sec=0.17, integral_gain=2.0,
        integral_limit=0.35, feedback_timeout_sec=0.25,
    ):
        values = (feedforward_time_sec, integral_gain, integral_limit,
                  feedback_timeout_sec)
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError('world-yaw hold parameters must be finite and nonnegative')
        if feedback_timeout_sec <= 0.0:
            raise ValueError('feedback_timeout_sec must be positive')
        self.feedforward_time_sec = feedforward_time_sec
        self.integral_gain = integral_gain
        self.integral_limit = integral_limit
        self.feedback_timeout_ns = int(feedback_timeout_sec * 1e9)
        self.target_yaw = None
        self.integral = 0.0
        self._feedback = None
        self._joint = None
        self._last_command_ns = None

    def update_feedback(self, yaw, angular_velocity, stamp_ns, received_ns):
        """Replace the chassis measurement; bad input invalidates old feedback."""
        if not all(math.isfinite(value) for value in (yaw, angular_velocity)):
            self._feedback = None
            return False
        self._feedback = (yaw, angular_velocity, stamp_ns, received_ns)
        return True

    def update_joint(self, position, stamp_ns, received_ns):
        """Retain the measured continuous joint angle, never wrap its turns."""
        if not math.isfinite(position):
            self._joint = None
            return False
        self._joint = (position, stamp_ns, received_ns)
        return True

    def invalidate_feedback(self):
        """Stop commanding until another valid chassis measurement arrives."""
        self._feedback = None

    def invalidate_joint(self):
        """Stop commanding until another valid joint measurement arrives."""
        self._joint = None

    def _fresh(self, sample, now_ns):
        if sample is None:
            return False
        stamp_ns, received_ns = sample[-2:]
        # /clock and sensor callbacks can be delivered in either order. Permit
        # a small source lead, but never accept old pre-reset clock samples.
        return (
            -50_000_000 <= now_ns - stamp_ns <= self.feedback_timeout_ns
            and 0 <= now_ns - received_ns <= self.feedback_timeout_ns
        )

    def command(self, now_ns):
        """Return a continuous joint position, or None for missing/stale input."""
        previous_ns = self._last_command_ns
        self._last_command_ns = now_ns
        if previous_ns is not None and now_ns < previous_ns:
            self.integral = 0.0
            self.target_yaw = None
            self._feedback = None
            self._joint = None
            return None
        if not self._fresh(self._feedback, now_ns) or not self._fresh(self._joint, now_ns):
            self.integral = 0.0
            return None
        yaw, angular_velocity = self._feedback[:2]
        joint = self._joint[0]
        if self.target_yaw is None:
            self.target_yaw = wrap_angle(yaw + joint)
        error = wrap_angle(self.target_yaw - yaw - joint)
        dt = 0.0 if previous_ns is None else (now_ns - previous_ns) / 1e9
        # A missed timer period must not integrate an unobserved old error.
        if 0.0 < dt <= min(0.1, self.feedback_timeout_ns / 1e9):
            self.integral = max(-self.integral_limit, min(
                self.integral_limit,
                self.integral + self.integral_gain * error * dt,
            ))
        command = (
            joint + error
            - angular_velocity * self.feedforward_time_sec
            + self.integral
        )
        return command if math.isfinite(command) else None
