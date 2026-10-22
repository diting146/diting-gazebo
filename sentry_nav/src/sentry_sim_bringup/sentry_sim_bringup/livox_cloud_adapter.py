# Copyright 2026 diting
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Turn the raw simulated LiDAR cloud into a deskew-ready body-frame cloud.

The simulated sensor publishes a rigid cloud in ``lidar_link``.  The LiDAR is
mounted on the rotating gimbal, so feeding it directly to point_lio would make
the estimator see a time-varying lidar-to-IMU extrinsic.  This node transforms
every point into the ``chassis`` frame (the IMU is at the chassis origin), and
adds the ``ring`` and ``time`` fields that point_lio's Velodyne preprocessor
expects.

``time`` defaults to a constant non-zero value in microseconds.  This matters:
point_lio treats an all-zero per-point time as "no timestamps supplied" and
then synthesises a 0.1 s sweep from the azimuth, which injects a spurious
rotation when the chassis spins.  A constant non-zero value makes the
preprocessor take the supplied (constant) offsets instead, i.e. no differential
deskew.
"""

import math
from collections import deque

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import PointCloud2, PointField

OUTPUT_POINT_STEP = 32
OUTPUT_LAYOUT = {
    'x': 0,
    'y': 4,
    'z': 8,
    'intensity': 16,
    'ring': 24,
    'time': 28,
}


def _quaternion_to_matrix(x, y, z, w):
    """Return the 3x3 rotation matrix for a quaternion."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class LivoxCloudAdapter(Node):
    """Republish the simulated LiDAR cloud in the chassis frame."""

    def __init__(self):
        """Read parameters and create the cloud pipeline."""
        super().__init__('livox_cloud_adapter')

        input_topic = self.declare_parameter(
            'input_topic', '/livox/lidar'
        ).value
        output_topic = self.declare_parameter(
            'output_topic', '/velodyne_points'
        ).value
        self._target_frame = self.declare_parameter(
            'target_frame', 'chassis'
        ).value
        self._n_scan = int(self.declare_parameter('n_scan', 32).value)
        self._angle_min_deg = float(
            self.declare_parameter('angle_min_deg', -7.0).value
        )
        self._angle_max_deg = float(
            self.declare_parameter('angle_max_deg', 52.0).value
        )
        self._time_mode = self.declare_parameter(
            'time_mode', 'constant_nonzero'
        ).value
        self._time_epsilon_us = float(
            self.declare_parameter('time_epsilon_us', 1.0).value
        )
        self._sweep_time_us = float(
            self.declare_parameter('sweep_time_us', 0.0).value
        )
        self._max_rate_hz = float(
            self.declare_parameter('max_rate_hz', 10.0).value
        )
        if self._time_mode not in (
            'constant_nonzero', 'linear', 'zero'
        ):
            raise ValueError(
                'time_mode must be constant_nonzero, linear or zero'
            )
        if self._n_scan <= 0:
            raise ValueError('n_scan must be positive')

        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer, self)
        self._publisher = self.create_publisher(
            PointCloud2, output_topic, QoSProfile(
                depth=5, reliability=ReliabilityPolicy.BEST_EFFORT
            )
        )
        self._subscription = self.create_subscription(
            PointCloud2, input_topic, self._on_cloud, QoSProfile(
                depth=5, reliability=ReliabilityPolicy.BEST_EFFORT
            )
        )

        self._pending = deque(maxlen=5)
        self.create_timer(0.01, self._drain_pending)
        self._frames = 0
        self._drops = 0
        self._last_stats_ns = None
        self._last_cloud_ns = None
        self._first_cloud_ns = None
        self._startup_delay = float(self.declare_parameter(
            "startup_delay", 3.0).value)

        self.get_logger().info(
            f'livox_cloud_adapter: {input_topic} -> {output_topic} '
            f'in {self._target_frame} (time_mode={self._time_mode})'
        )

    def _transform_ok(self, msg):
        """Return the transform into the target frame, or None."""
        if msg.header.frame_id == self._target_frame:
            return (np.eye(3), np.zeros(3))
        try:
            transform = self._buffer.lookup_transform(
                self._target_frame,
                msg.header.frame_id,
                rclpy.time.Time.from_msg(msg.header.stamp),
            )
        except tf2_ros.TransformException as error:
            self.get_logger().debug(f'TF lookup failed: {error}')
            return None
        rotation = transform.transform.rotation
        translation = transform.transform.translation
        return (
            _quaternion_to_matrix(
                rotation.x, rotation.y, rotation.z, rotation.w
            ),
            np.array([translation.x, translation.y, translation.z]),
        )

    def _input_dtype(self, msg):
        """Build a structured dtype matching the incoming cloud layout."""
        offsets = {field.name: field.offset for field in msg.fields}
        if not {'x', 'y', 'z'}.issubset(offsets):
            return None, None
        names = ['x', 'y', 'z']
        formats = ['<f4', '<f4', '<f4']
        used = [offsets['x'], offsets['y'], offsets['z']]
        for name, fmt in (('intensity', '<f4'), ('ring', '<u2')):
            if name in offsets:
                names.append(name)
                formats.append(fmt)
                used.append(offsets[name])
        dtype = np.dtype({
            'names': names,
            'formats': formats,
            'offsets': used,
            'itemsize': msg.point_step,
        })
        return dtype, offsets

    def _ring_for_points(self, cloud):
        """Return ring indices, from the input or from the vertical angle."""
        if 'ring' in cloud.dtype.names:
            return np.asarray(cloud['ring'], dtype=np.uint16)
        distance = np.hypot(
            np.asarray(cloud['x'], dtype=np.float64),
            np.asarray(cloud['y'], dtype=np.float64),
        )
        vertical_deg = np.degrees(
            np.arctan2(np.asarray(cloud['z'], dtype=np.float64), distance)
        )
        span = self._angle_max_deg - self._angle_min_deg
        ring = np.floor(
            (vertical_deg - self._angle_min_deg) / (span / self._n_scan)
        )
        return np.clip(ring, 0, self._n_scan - 1).astype(np.uint16)

    def _time_for_points(self, count):
        """Return the per-point time column in microseconds."""
        if self._time_mode == 'zero':
            return np.zeros(count, dtype='<f4')
        if self._time_mode == 'linear':
            return np.linspace(
                0.0, self._sweep_time_us, count, dtype='<f4'
            )
        return np.full(count, self._time_epsilon_us, dtype='<f4')

    def _on_cloud(self, msg):
        """Wait for joint transforms at the sensor timestamp without blocking TF."""
        if len(self._pending) == self._pending.maxlen:
            self._drops += 1
        stamp_ns = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds
        if self._first_cloud_ns is None:
            self._first_cloud_ns = stamp_ns
        if (stamp_ns - self._first_cloud_ns) * 1e-9 < self._startup_delay:
            return
        self._pending.append(msg)

    def _drain_pending(self):
        while self._pending:
            msg = self._pending[0]
            transform = self._transform_ok(msg)
            if transform is None:
                return
            self._pending.popleft()
            self._publish_cloud(msg, transform)

    def _publish_cloud(self, msg, transform):
        """Transform a cloud using the transform at its acquisition time."""
        now_ns = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds
        if self._last_cloud_ns is not None and self._max_rate_hz > 0.0:
            if (now_ns - self._last_cloud_ns) * 1e-9 < 1.0 / self._max_rate_hz:
                return
        self._last_cloud_ns = now_ns

        dtype, offsets = self._input_dtype(msg)
        if dtype is None:
            self.get_logger().warning('cloud has no xyz fields')
            return

        count = msg.width * msg.height
        if count == 0:
            return
        cloud = np.ndarray(
            (msg.height, msg.width), dtype=dtype, buffer=bytes(msg.data),
            strides=(msg.row_step, msg.point_step),
        ).reshape(-1)

        points = np.empty((count, 3), dtype=np.float64)
        points[:, 0] = cloud['x']
        points[:, 1] = cloud['y']
        points[:, 2] = cloud['z']
        finite = np.isfinite(points).all(axis=1)
        cloud = cloud[finite]
        points = points[finite]
        count = len(points)
        if count == 0:
            return
        rotation, translation = transform
        body = points @ rotation.T + translation

        intensity = (
            np.asarray(cloud['intensity'], dtype='<f4')
            if 'intensity' in cloud.dtype.names
            else np.zeros(count, dtype='<f4')
        )

        output = np.zeros(count, dtype=np.dtype({
            'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
            'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
            'offsets': [
                OUTPUT_LAYOUT['x'], OUTPUT_LAYOUT['y'],
                OUTPUT_LAYOUT['z'], OUTPUT_LAYOUT['intensity'],
                OUTPUT_LAYOUT['ring'], OUTPUT_LAYOUT['time'],
            ],
            'itemsize': OUTPUT_POINT_STEP,
        }))
        output['x'] = body[:, 0]
        output['y'] = body[:, 1]
        output['z'] = body[:, 2]
        output['intensity'] = intensity
        output['ring'] = self._ring_for_points(cloud)
        output['time'] = self._time_for_points(count)

        adapted = PointCloud2()
        adapted.header.stamp = msg.header.stamp
        adapted.header.frame_id = self._target_frame
        adapted.height = 1
        adapted.width = count
        adapted.fields = [
            PointField(
                name='x', offset=OUTPUT_LAYOUT['x'],
                datatype=PointField.FLOAT32, count=1,
            ),
            PointField(
                name='y', offset=OUTPUT_LAYOUT['y'],
                datatype=PointField.FLOAT32, count=1,
            ),
            PointField(
                name='z', offset=OUTPUT_LAYOUT['z'],
                datatype=PointField.FLOAT32, count=1,
            ),
            PointField(
                name='intensity', offset=OUTPUT_LAYOUT['intensity'],
                datatype=PointField.FLOAT32, count=1,
            ),
            PointField(
                name='ring', offset=OUTPUT_LAYOUT['ring'],
                datatype=PointField.UINT16, count=1,
            ),
            PointField(
                name='time', offset=OUTPUT_LAYOUT['time'],
                datatype=PointField.FLOAT32, count=1,
            ),
        ]
        adapted.is_bigendian = False
        adapted.point_step = OUTPUT_POINT_STEP
        adapted.row_step = OUTPUT_POINT_STEP * count
        adapted.is_dense = False
        adapted.data = output.tobytes()
        self._publisher.publish(adapted)

        self._frames += 1
        if (
            self._last_stats_ns is None
            or now_ns - self._last_stats_ns > 5_000_000_000
        ):
            self._last_stats_ns = now_ns
            self.get_logger().info(
                f'published {self._frames} frames, '
                f'{self._drops} dropped'
            )


def main(args=None):
    """Run the node until shutdown."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = LivoxCloudAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
