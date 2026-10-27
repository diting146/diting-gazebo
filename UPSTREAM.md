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

## Navigation overlay

`sentry_nav/src/pb2025_sentry_nav` is a source snapshot from https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav, including populated dependency submodules as ordinary directories. Original package licenses are retained. `sentry_nav/src/sentry_sim_bringup` contains the local Gazebo/Nav2 integration, lidar scan conversion configuration, gimbal TF and world-yaw stabilization, and runtime spin control.

Snapshot revisions:

- `pb2025_sentry_nav`: `9c124ebb0c0b6b64714390ea3c92f27490f63f9d`
- `pb_omni_pid_pursuit_controller`: `b542c5794b91fba2bb3fe2cbdc72fcf854fedb1a`
- `pointcloud_to_laserscan`: `5efaa8ba391bf4450a02cc5bb794b586c9f7d267`
- `livox_ros_driver2`: `524e14a1ec714e427bde729a8f869bb7acd8507d`
- `small_gicp_relocalization`: `1df063ee6f67b592e1da47e2a21ded00321c0906`
- `pb_teleop_twist_joy`: `becf0f61b8453ef9b089cbad82e5cb2efe00c31e`
- `point_lio`: `e85e79558cf746f6699888a54285fe48b3b0ac71`
- `pb_nav2_plugins`: `e6778ca44d9e88aa612a3c6c87d536a8cfa21705`

## 2026-10-22 integration

The overlay now defaults to sensor-only Point-LIO and small_gicp localization.
Local additions include instantaneous cloud adaptation into the chassis frame,
IMU-based gimbal stabilization, terrain-cloud obstacle layers, mapping bringup,
and reference-only error monitoring. Upstream C++ estimator source is retained;
the simulation parameter snapshot and custom integration package carry the
configuration changes.

The tracked `diting_3v3_field_mapped.pcd` prior contains 242,239 downsampled
sensor-mapping points in the original mapping coordinates. The 5,483,668-point
raw capture is kept locally and is not distributed in Git. The STL-derived
`diting_3v3_field.pcd` remains a separate geometric reference.

Model geometry follows collaborator commit `ca4d876`, including wider wheel
positions and the raised, pitched lidar. Local costmap radii are 0.40 m to cover
the widened wheel collision envelope. Earlier dynamic acceptance measurements
precede that geometry update; see `docs/validation/2026-10-22.md` for the scope.

## 2026-10-27 dual-robot YAstar integration

`src/pb2025_sentry_nav/tdt_yastar_planner` imports the two-dimensional YAstar
implementation from the audited TDT Nav Kit source snapshot (`src/YAstar`).
The original upstream commit was not recorded with that snapshot. The imported
algorithm is copyright 2026 Rongxuan Ye and BoLin Li under MIT; the Nav2 adapter
is project code under Apache-2.0. The complete MIT notice is retained in
`tdt_yastar_planner/TDT_LICENSE_NOTICE.md`. Local adaptations add a Nav2 costmap
snapshot, bounded search, path headers/headings, and regression coverage; the
kinodynamic planner and MinimumSnap are not imported.

The custom simulation integration now uses one autonomous sentry and one
manual obstacle robot, namespaced sensor/control wiring, simulation command
watchdogs, and shared wait/replan behavior trees. Only the integration package
and the new planner are published in this change; existing estimator, model,
and `fake_vel_transform` source is retained from the remote baseline. Validation
scope and remaining differences from the development workspace are recorded in
`docs/validation/2026-10-27.md`.
