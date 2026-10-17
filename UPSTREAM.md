# Upstream source provenance

This workspace contains source snapshots, including local simulation changes. Upstream Git metadata is not included; the source files and existing license notices are retained.

| Local path | Original repository | Base commit |
| --- | --- | --- |
| `src/pb2025_robot_description` | https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_robot_description.git | `9afbce352e01c567f2454ee4f92bee38d5f015d1` |
| `src/pb2025_sentry_nav` | https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav.git | `9c124ebb0c0b6b64714390ea3c92f27490f63f9d` |
| `src/pb2025_sentry_nav/livox_ros_driver2` | https://github.com/SMBU-PolarBear-Robotics-Team/livox_ros_driver2.git | `524e14a1ec714e427bde729a8f869bb7acd8507d` |
| `src/pb2025_sentry_nav/pb_nav2_plugins` | https://github.com/SMBU-PolarBear-Robotics-Team/pb_nav2_plugins.git | `e6778ca44d9e88aa612a3c6c87d536a8cfa21705` |
| `src/pb2025_sentry_nav/pb_omni_pid_pursuit_controller` | https://github.com/SMBU-PolarBear-Robotics-Team/pb_omni_pid_pursuit_controller.git | `b542c5794b91fba2bb3fe2cbdc72fcf854fedb1a` |
| `src/pb2025_sentry_nav/pb_teleop_twist_joy` | https://github.com/SMBU-PolarBear-Robotics-Team/pb_teleop_twist_joy.git | `becf0f61b8453ef9b089cbad82e5cb2efe00c31e` |
| `src/pb2025_sentry_nav/point_lio` | https://github.com/SMBU-PolarBear-Robotics-Team/point_lio.git | `e85e79558cf746f6699888a54285fe48b3b0ac71` |
| `src/pb2025_sentry_nav/pointcloud_to_laserscan` | https://github.com/SMBU-PolarBear-Robotics-Team/pointcloud_to_laserscan.git | `5efaa8ba391bf4450a02cc5bb794b586c9f7d267` |
| `src/pb2025_sentry_nav/small_gicp_relocalization` | https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization.git | `1df063ee6f67b592e1da47e2a21ded00321c0906` |
| `src/rmoss_gazebo` | https://github.com/SMBU-PolarBear-Robotics-Team/rmoss_gazebo.git | `7443ff06c345752d0e23172c479d99b27207e2c7` |
| `src/rmu_gazebo_simulator` | https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator.git | `61c49f5f0a9fd1ab9d867fb30c49520f5d5a0ee7` |

## Local simulation changes

- Custom 12.6 × 8.6 m arena and bundled robot/model resources.
- Two rm25 robots spawn at (-5.3, 3.0) and (5.3, 3.0), facing into the arena.
- Per-robot velocity, odometry, point cloud, IMU, and joint-state bridges.
- HP referee with an optional damage injector; both scripts are executable.
- Navigation goal support exists in the referee; end-to-end Nav2 return-home behavior has not been validated.

## Licenses

Consult each vendored project and third-party directory for its license. This snapshot does not replace those licenses or claim ownership of upstream source or models. The robot-description package declares Apache-2.0 in its package manifest.
