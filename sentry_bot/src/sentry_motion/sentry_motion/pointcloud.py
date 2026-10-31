"""Read XYZ from ROS clouds without assuming intensity/ring share XYZ dtype."""
import numpy as np
from sensor_msgs_py import point_cloud2


def xyz_points(cloud):
    points = point_cloud2.read_points(
        cloud, field_names=("x", "y", "z"), skip_nans=True
    )
    if isinstance(points, np.ndarray) and points.dtype.names:
        xyz = np.column_stack([points[name] for name in ("x", "y", "z")]).astype(float)
    else:
        xyz = np.asarray(list(points), dtype=float).reshape(-1, 3)
    return xyz[np.isfinite(xyz).all(axis=1)]
