"""Check sensor conversion and time-aligned reference diagnostics."""

import math
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2, PointField
from sentry_sim_bringup.livox_cloud_adapter import LivoxCloudAdapter
from sentry_sim_bringup.localization_error_monitor import LocalizationErrorMonitor
from sentry_sim_bringup.sim_lio_gateway import SimSerialGateway


@pytest.fixture
def context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_cloud_transform_preserves_stamp_and_instantaneous_scan_time(context):
    node = LivoxCloudAdapter(parameter_overrides=[Parameter('max_rate_hz', value=0.0)])
    captured = []
    node._publisher = SimpleNamespace(publish=captured.append)
    msg = PointCloud2()
    msg.header.stamp.sec = 5
    msg.height, msg.width = 1, 3
    msg.point_step, msg.row_step = 12, 36
    msg.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(('x', 'y', 'z'))]
    msg.data = np.array([[1, 0, 0], [0, 1, 0], [np.nan, 0, 0]], dtype='<f4').tobytes()
    rotation = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    try:
        node._publish_cloud(msg, (rotation, np.array([1, 2, 3])))
        assert len(captured) == 1
        output = captured[0]
        assert output.header.stamp == msg.header.stamp
        assert output.header.frame_id == 'chassis'
        assert output.width == 2
        dtype = np.dtype({'names': ['x', 'y', 'z', 'time'],
                          'formats': ['<f4'] * 4, 'offsets': [0, 4, 8, 28], 'itemsize': 32})
        points = np.frombuffer(output.data, dtype=dtype)
        np.testing.assert_allclose(points['x'], [1, 0])
        np.testing.assert_allclose(points['y'], [3, 2])
        np.testing.assert_allclose(points['z'], [3, 3])
        np.testing.assert_array_equal(points['time'], [1, 1])
    finally:
        node.destroy_node()


def test_error_monitor_uses_same_timestamp_and_wraps_yaw(context):
    node = LocalizationErrorMonitor()
    captured, queries = [], []
    node._publisher = SimpleNamespace(publish=captured.append)
    transform = TransformStamped()
    transform.header.stamp.sec = 5
    transform.header.stamp.nanosec = 50_000_000
    transform.transform.translation.x = 1.2
    transform.transform.translation.y = 1.7
    transform.transform.rotation.z = math.sin(math.radians(-179) / 2)
    transform.transform.rotation.w = math.cos(math.radians(-179) / 2)

    def lookup(map_frame, robot_frame, stamp):
        queries.append(stamp.nanoseconds)
        return transform

    node._buffer = SimpleNamespace(lookup_transform=lookup)
    try:
        for second in (4, 5, 6):
            reference = PoseStamped()
            reference.header.stamp.sec = second
            reference.pose.position.x = 1.0
            reference.pose.position.y = 2.0
            reference.pose.orientation.z = math.sin(math.radians(179) / 2)
            reference.pose.orientation.w = math.cos(math.radians(179) / 2)
            node._on_reference(reference)
        node._on_timer()
        assert queries == [0, 5_000_000_000]
        assert len(captured) == 1
        result = captured[0]
        assert result.header.stamp.sec == 5
        assert result.vector.x == pytest.approx(0.2)
        assert result.vector.y == pytest.approx(-0.3)
        assert result.vector.z == pytest.approx(math.radians(2))
        # A reference older than the allowed skew must not produce a new error.
        transform.header.stamp.nanosec = 200_000_000
        node._on_timer()
        assert len(captured) == 1
    finally:
        node.destroy_node()


def test_sensor_gateway_requires_fresh_operational_odometry(context):
    node = SimSerialGateway(parameter_overrides=[
        Parameter('require_localization', value=True),
        Parameter('localization_topic', value='/lio_unit/odometry'),
        Parameter('spin_topic', value=''),
    ])
    now = [0]
    node._watchdog_clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now[0]))
    try:
        assert not node._localization_ready()
        node._on_localization(None)
        assert not node._localization_ready()
        for stamp in (500_000_000, 1_000_000_000, 1_500_000_000, 2_100_000_000):
            now[0] = stamp
            node._on_localization(None)
        assert node._localization_ready()
        now[0] += 1_000_000_000
        assert not node._localization_ready()
        node._on_localization(None)
        assert not node._localization_ready()
        subscriptions = node.get_subscriptions_info_by_topic('/lio_unit/odometry')
        assert any(item.node_name == node.get_name() for item in subscriptions)
        assert not node.get_subscriptions_info_by_topic('/reference/odometry')
    finally:
        node.destroy_node()
