# PolarBear Omni PID Pursuit Controller

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build](https://github.com/SMBU-PolarBear-Robotics-Team/pb_omni_pid_pursuit_controller/actions/workflows/build_and_test.yml/badge.svg)](https://github.com/SMBU-PolarBear-Robotics-Team/pb_omni_pid_pursuit_controller/actions/workflows/build_and_test.yml)

## Configuration

| Parameter | Description |
|-----|----|
| `translation_kp` | The proportional gain for the translation PID controller. Controls how strongly the robot reacts to translation errors. |
| `translation_ki` | The integral gain for the translation PID controller. Helps eliminate steady-state errors over time. |
| `translation_kd` | The derivative gain for the translation PID controller. Damps oscillations by reacting to the rate of change of translation errors. |
| `enable_rotation` | Whether to enable the rotation PID controller. If disabled, the robot will not rotate to face the path's direction. `twist.angular.z` always remains zero. |
| `rotation_kp` | The proportional gain for the rotation PID controller. Controls how strongly the robot reacts to rotational errors. |
| `rotation_ki` | The integral gain for the rotation PID controller. Helps eliminate steady-state rotational errors. |
| `rotation_kd` | The derivative gain for the rotation PID controller. Damps oscillations by reacting to the rate of change of rotational errors. |
| `transform_tolerance` | The tolerance for transforming between frames. A higher value may allow for more flexibility in handling small delays in the transformation. |
| `min_max_sum_error` | The minimum threshold for the maximum sum of errors used to limit the accumulated error in the PID controller to prevent windup. |
| `lookahead_dist` | The fixed lookahead distance used to find the lookahead point for path following. |
| `use_velocity_scaled_lookahead_dist` | Whether to scale the lookahead distance based on the current velocity, instead of using a constant distance. |
| `min_lookahead_dist` | The minimum allowable lookahead distance when using velocity scaling for the lookahead point. |
| `max_lookahead_dist` | The maximum allowable lookahead distance when using velocity scaling for the lookahead point. |
| `lookahead_time` | The time used to project the robot's velocity to calculate the velocity-scaled lookahead distance. |
| `use_interpolation` | Enables interpolation between poses along the path when selecting the lookahead point, improving smoothness but potentially increasing computational cost. |
| `use_rotate_to_heading` | Whether to rotate the robot to face the path's direction before moving forward, useful in holonomic robots. |
| `use_rotate_to_heading_treshold` | The angular threshold at which the robot should rotate in place to align with the desired heading. |
| `min_approach_linear_velocity` | The minimum linear velocity when approaching the goal to ensure the robot moves slowly when close to its target. |
| `approach_velocity_scaling_dist` | The distance from the goal where velocity scaling starts when approaching, slowing the robot down as it nears the target. |
| `v_linear_min` | The minimum translation speed the robot can command, allowing for reverse or slow-forward movement. |
| `v_linear_max` | The maximum translation speed the robot can command, setting the upper limit for forward movement. |
| `v_angular_min` | The minimum rotation speed the robot can command, allowing for counter-clockwise rotation. |
| `v_angular_max` | The maximum rotation speed the robot can command, setting the upper limit for clockwise rotation. |
| `curvature_min` | The minimum curvature threshold below which no speed reduction is applied. |
| `curvature_max` | The maximum curvature threshold above which significant speed reduction is applied. |
| `reduction_ratio_at_high_curvature` | The speed reduction ratio at high curvature. 0.5 means a 50% reduction. |
| `curvature_forward_dist` | The forward distance used for curvature calculation. |
| `curvature_backward_dist` | The backward distance used for curvature calculation. |
| `max_velocity_scaling_factor_rate` | The maximum rate of change for the velocity scaling factor. |
| `max_robot_pose_search_dist` | The maximum distance along the path to search for the robot's closest pose, used to keep the robot on the planned path. |

Example fully-described XML with default parameter values:

```yaml
controller_server:
  ros__parameters:
    odom_topic: odometry
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
    FollowPath:
      plugin: "pb_omni_pid_pursuit_controller::OmniPidPursuitController"
      translation_kp: 3.0
      translation_ki: 0.1
      translation_kd: 0.3
      enable_rotation: true
      rotation_kp: 3.0
      rotation_ki: 0.1
      rotation_kd: 0.3
      transform_tolerance: 0.1
      min_max_sum_error: 1.0
      lookahead_dist: 2.0
      use_velocity_scaled_lookahead_dist: true
      lookahead_time: 1.0
      min_lookahead_dist: 0.5
      max_lookahead_dist: 1.0
      use_interpolation: false
      use_rotate_to_heading: false
      use_rotate_to_heading_treshold: 0.1
      min_approach_linear_velocity: 0.5
      approach_velocity_scaling_dist: 1.0
      v_linear_min: -2.5
      v_linear_max: 2.5
      v_angular_min: -3.0
      v_angular_max: 3.0
      curvature_min: 0.4
      curvature_max: 0.7
      reduction_ratio_at_high_curvature: 0.5
      curvature_forward_dist: 0.7
      curvature_backward_dist: 0.3
      max_velocity_scaling_factor_rate: 0.9
```


## 仿真控制器碰撞诊断（2026-11-04）

首次碰撞拒绝会在相对话题 `controller_collision` 发布可靠、transient-local 的 JSON String。记录 `plugin`、`branch`（current/command/planned）、命中格及原始代价值、完整带padding足迹、判定扫掠多边形、候选速度和同一次地图锁内复制的完整局部地图。连续拒绝仅输出一次；真正返回安全速度后才重新允许记录，重规划不会重置。

```bash
ros2 topic echo /sentry_robot1/controller_collision std_msgs/msg/String \
  --qos-durability transient_local --qos-reliability reliable --once
```

`decision_snapshot_stamp_ns` 是决策快照时间，`map_update_stamp_available:false` 表示没有地图源更新时间；不能当作传感器采样时间。TF变换结果保留原PoseStamped时间，但没有新增同步TF采样。数据复制发生在原costmap锁内，JSON序列化及发布在解锁后；正常与连续重复拒绝不复制整张地图。不改变254/255拒绝、完整车身扫掠或停止策略。旧七参数几何接口继续保留，供隧道插件链接。

本轮受控动态障碍已验证命中格与完整快照一致、独立SAT可重放，并实际停车；这不能倒推出历史无快照失败的具体原因。
