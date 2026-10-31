import numpy as np
from sensor_msgs.msg import PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header
from sentry_motion.pointcloud import xyz_points


def test_raw_cloud_mixed_ring_datatype_and_nan_points():
    fields = [
        PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
        for i, name in enumerate(("x", "y", "z"))
    ]
    fields.append(
        PointField(name="ring", offset=12, datatype=PointField.UINT16, count=1)
    )
    cloud = create_cloud(
        Header(frame_id="lidar_link"),
        fields,
        [(1.0, 2.0, 3.0, 7), (np.nan, 2.0, 3.0, 8)],
    )
    np.testing.assert_allclose(xyz_points(cloud), [[1.0, 2.0, 3.0]])
