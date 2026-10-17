# Diting Gazebo Simulation

ROS 2 Humble / Gazebo Fortress 双机器人仿真工作区，包含自定义 3v3 场地、rm25 机器人、传感器桥接和简化血量裁判。

## 当前功能

- 12.6 × 8.6 米场地，两台机器人分别出生于俯视图左上角和右上角。
- `rm25_robot1`：`(-5.3, 3.0, 0.3)`；`rm25_robot2`：`(5.3, 3.0, 0.3)`；初始 yaw 均为 `-pi/2`。
- 各机器人独立的速度控制、里程计、三维点云、IMU 和关节状态话题。
- 初始血量 100，低于 30 时尝试请求 Nav2 回家；完整导航闭环仍需接入和验证。

## 环境与构建

使用 Ubuntu 22.04、ROS 2 Humble 和 Gazebo Fortress。先按各软件的安装说明配置环境；C++ 插件需要 Fortress 的开发库（包括 `libignition-gazebo6-dev`）。

在仓库根目录运行：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/rmoss_gazebo/rmoss_gz_plugins src/rmu_gazebo_simulator/rmu_gazebo_simulator --ignore-src --rosdistro humble -r -y
colcon build --packages-select rmoss_gz_plugins rmu_gazebo_simulator --symlink-install
source install/setup.bash
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
```

这里先构建已用于仿真测试的两个包。`src/pb2025_sentry_nav/` 和机器人描述源码也已保留；全量导航构建需要各上游 README 中列出的额外依赖。

## 配置与检查

- 出生点：`src/rmu_gazebo_simulator/rmu_gazebo_simulator/config/gz_world.yaml`
- 传感器桥接：同目录下的 `ros_gz_bridge.yaml`
- 场地与模型：`src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/`
- 裁判实现：`src/rmu_gazebo_simulator/rmu_gazebo_simulator/scripts/referee_system/`

```bash
ros2 topic echo /rm25_robot1/odometry --once
ros2 topic echo /referee/rm25_robot1/hp --once
ros2 topic hz /rm25_robot1/points_raw
```

本机重启测试已确认两台机器人出生点、里程计、点云、IMU 和血量数据正常，`/clock` 只有一个发布源。这不等同于已验证所有导航功能。

## 代码来源

源码以普通目录快照收录，克隆本仓库不需要初始化 Git 子模块。上游地址、基线提交和许可说明见 [UPSTREAM.md](UPSTREAM.md)。本地修改已包含在此快照中。构建产物、运行日志和旧模型备份未收录。
