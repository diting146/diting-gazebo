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

## 哨兵导航工作区（sentry_nav）

`sentry_nav/src/` 包含完整的导航源码快照和自定义 `sentry_sim_bringup`。当前仿真支持 3D 点云转 LaserScan 避障、Gazebo 真值定位、底盘自旋及云台对地稳向。两个工作区均包含上游导航包，因此构建时必须指定 `--base-paths src`，分别构建：

```bash
# 仓库根目录：先构建 Gazebo 工作区
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 再构建导航 overlay
cd sentry_nav
colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch sentry_sim_bringup bringup_sentry_simulation_launch.py x:=-5.3 y:=3.0 yaw:=-1.57079632679 spin_speed:=3.14
```

以上全量构建需先安装上游包的依赖；仅构建 `sentry_sim_bringup` 要求其依赖已经安装。新终端应依次 source ROS、仓库根目录 `install/setup.bash`、`sentry_nav/install/setup.bash`。

运行中开启或停止自旋（云台稳向继续工作）：

```bash
ros2 topic pub --once -w 2 --keep-alive 1 /cmd_spin example_interfaces/msg/Float32 "{data: 3.14}"
ros2 topic pub --once -w 2 --keep-alive 1 /cmd_spin example_interfaces/msg/Float32 "{data: 0.0}"
```

默认自旋速度由 `spin_speed` 指定；`spin_speed:=0` 关闭启动时自旋，`gimbal_world_lock:=False` 关闭云台稳向。当前 TF 高度表达仍有简化，实车部署需另外接入真实定位、电控和传感器标定。
