"""Frame-independent geometry for a directed tunnel segment (metres/radians)."""
import math
from dataclasses import dataclass


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class LineError:
    along: float
    lateral: float
    heading: float
    remaining: float


def line_error(start, end, pose):
    values = (*start, *end, *pose)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("Non-finite route or pose")
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dx, dy)
    if length < 0.1:
        raise ValueError("Tunnel segment must be at least 0.1 m")
    ux, uy = dx / length, dy / length
    px, py = pose[0] - start[0], pose[1] - start[1]
    along = px * ux + py * uy
    return LineError(
        along, -px * uy + py * ux, wrap(pose[2] - math.atan2(dy, dx)), length - along
    )


def body_velocity(start, end, yaw, forward, lateral):
    angle = math.atan2(end[1] - start[1], end[0] - start[0]) - yaw
    return (
        forward * math.cos(angle) - lateral * math.sin(angle),
        forward * math.sin(angle) + lateral * math.cos(angle),
    )


def validate_tunnel_segment(start, end):
    """Only the mesh-audited UL top-left passage, in either direction."""
    if not all(math.isfinite(v) for v in (*start, *end)):
        raise ValueError("Non-finite tunnel coordinates")
    a, b = sorted((start, end), key=lambda p: p[0])
    if not (-4.4 <= a[0] <= -4.0 and -1.05 <= b[0] <= -0.7):
        raise ValueError("Entry/exit must stay outside the measured tunnel")
    for x in (-2.675, -2.325):
        y = a[1] + (b[1] - a[1]) * (x - a[0]) / (b[0] - a[0])
        if not 3.44 <= y <= 3.48:
            raise ValueError("Line leaves the audited tunnel center corridor")
    if abs(math.atan2(b[1] - a[1], b[0] - a[0])) > math.radians(3):
        raise ValueError("Tunnel axis deviation must be at most 3 degrees")
