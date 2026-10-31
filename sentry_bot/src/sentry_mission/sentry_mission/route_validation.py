"""Constrain v1 route editing to the measured left-upper UL passage."""
import math


def validate_route(entry, exit_point, center):
    for point in (entry, exit_point, center):
        if point.header.frame_id != "map":
            raise ValueError("Route frame must be map")
        p = point.pose.position
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            raise ValueError("Non-finite route")
        if abs(p.z) > 0.01:
            raise ValueError("Route poses must use map-plane z=0")
        if not (-6 < p.x < 6 and -4 < p.y < 4):
            raise ValueError("Point outside UL field")
    a, b = entry.pose.position, exit_point.pose.position
    from sentry_motion.line_geometry import validate_tunnel_segment

    validate_tunnel_segment((a.x, a.y), (b.x, b.y))
    return True


def validate_map_points(poses, grid, radius=0.4):
    """Conservative full-circle static clearance, not just free goal pixels."""
    import numpy as np

    if grid is None or grid.header.frame_id != "map":
        raise ValueError("Static map unavailable")
    resolution = grid.info.resolution
    if resolution <= 0 or len(grid.data) != grid.info.width * grid.info.height:
        raise ValueError("Invalid static map")
    q = grid.info.origin.orientation
    angle = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    data = np.asarray(grid.data).reshape(grid.info.height, grid.info.width)
    rows, cols = np.nonzero((data < 0) | (data >= 50))
    for pose in poses:
        dx = pose.pose.position.x - grid.info.origin.position.x
        dy = pose.pose.position.y - grid.info.origin.position.y
        x, y = dx * math.cos(angle) + dy * math.sin(angle), -dx * math.sin(
            angle
        ) + dy * math.cos(angle)
        if (
            min(
                x,
                y,
                grid.info.width * resolution - x,
                grid.info.height * resolution - y,
            )
            < radius
        ):
            raise ValueError("Goal footprint crosses map boundary")
        distances = np.hypot(
            (cols + 0.5) * resolution - x, (rows + 0.5) * resolution - y
        )
        if np.any(distances < radius + resolution * math.sqrt(2) / 2):
            raise ValueError("Goal has insufficient static footprint clearance")
    return True
