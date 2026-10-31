# Diting Gazebo Simulation

ROS 2 Humble / Gazebo Fortress 仿真与哨兵导航工程，包含自定义 3v3 场地、RM25 模型、传感器定位、Nav2 避障和自旋稳向。

## 仿真场地与可选机器人

- 12.6 × 8.6 米场地，配置中保留左上角与右上角两台可选机器人，当前均为 `spawn: false`。
- `rm25_robot1`：`(-5.3, 3.0, 0.3)`；`rm25_robot2`：`(5.3, 3.0, 0.3)`；初始 yaw 均为 `-pi/2`。
- 各机器人独立的速度控制、里程计、三维点云、IMU 和关节状态话题。
- 初始血量 100，低于 30 时尝试请求 Nav2 回家；完整导航闭环仍需接入和验证。

## 环境与构建

使用 Ubuntu 22.04、ROS 2 Humble 和 Gazebo Fortress。先按各软件的安装说明配置环境；C++ 插件需要 Fortress 的开发库（包括 `libignition-gazebo6-dev`）。

在仓库根目录运行：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/rmoss_gazebo/rmoss_gz_plugins src/rmu_gazebo_simulator/rmu_gazebo_simulator --ignore-src --rosdistro humble -r -y
colcon build --base-paths src --packages-select rmoss_gz_plugins rmu_gazebo_simulator --symlink-install
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

默认使用 **Point-LIO + small_gicp 传感器定位**，左上角出生，只生成哨兵，自旋关闭。Gazebo 雷达和 IMU 提供传感器输入；真值仅发布到 `/reference/*` 供误差观测，不提供定位、TF 或运动控制。`localization:=ground_truth` 仅用于显式调试。

### 构建与启动

根目录 `src/` 为仿真工作区，`sentry_nav/src/` 为导航 overlay。两处包含同名上游包，构建时均指定 `--base-paths src`，按以下顺序操作：

```bash
# 在仓库根目录
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

cd sentry_nav
colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
cd ..
source source_all.bash
ros2 launch sentry_sim_bringup bringup_sentry_simulation_launch.py \
  localization:=lio spin_speed:=0.0 use_rviz:=False
```

全量构建需要安装上游导航包依赖。如果 small_gicp 的 FetchContent 无法访问 GitHub，可在相应构建命令中追加 `-DFETCHCONTENT_SOURCE_DIR_SMALL_GICP=/path/to/local/small_gicp`，替换为已有源码的绝对路径。新终端从仓库根目录执行 `source source_all.bash`；该脚本也兼容本机 `gazebo/` 与 `sentry_nav/` 并列的布局。

另外两台 RM25 默认关闭；要启用时，修改 `src/rmu_gazebo_simulator/rmu_gazebo_simulator/config/gz_world.yaml` 中相应机器人的 `spawn: true`，并重建仿真包。哨兵默认出生点为 `x=-5.3, y=3.0, yaw=-pi/2`。

### 建图、地图与坐标

纯 3D 建图时不启动 Nav2 或 small_gicp：

```bash
ros2 launch sentry_sim_bringup bringup_sentry_simulation_launch.py \
  localization:=lio slam:=True spin_speed:=0.0 x:=0 y:=0 yaw:=0
```

等待落地和 LIO 初始化后，用 `/cmd_vel` 手动采集。正常 Ctrl+C 时 Point-LIO 尝试保存 `sentry_nav/src/pb2025_sentry_nav/point_lio/PCD/scans.pcd`。该目录属于本地采集输出；保存成功不代表地图质量合格。

运行先验为 `sentry_nav/src/sentry_sim_bringup/maps/diting_3v3_field_mapped.pcd`，含 242,239 个点，来自传感器建图原始结果的体素降采样，保留原始 camera_init 坐标。约 167 MiB 的 `diting_3v3_field_mapped_raw.pcd` 保留在采集机器上，不随仓库分发。原有 STL 采样 `diting_3v3_field.pcd` 仅作几何对照，二维导航地图仍为同目录的 PGM/YAML。

```text
map → odom → base_footprint → chassis
                                ├── gimbal_yaw_fake
                                └── gimbal_yaw_odom → gimbal_yaw → lidar_link
```

雷达 `/livox/lidar` 经采样时刻的完整 TF 转到 `chassis`，再供 Point-LIO 使用；IMU 为 `/livox/imu`。Gazebo 整帧点云统一使用 `time=1 µs`，关闭体素时间混合；当前 Point-LIO 大批量测量分支使用逆方差权重，配置 `lidar_meas_cov: 1000.0`，`acc_norm: 9.8`。这些参数针对当前仿真输入，不直接作为实物雷达标定。

局部、全局 costmap 使用经坐标转换的实时地形点云和标准 `ObstacleLayer`，按实际高度筛除地面；`/obstacle_scan` 同时保留。更新后的轮心为 `(±0.21152, ±0.24318, 0)`，轮胎碰撞包络约 0.391 m，故两种定位配置的 costmap 半径均为 **0.40 m**。

雷达仍装在哨兵云台 yaw 上，安装俯仰约 12°；对应 TF 平移为 `(0.15, -0.12, 0.0306)`。修改共用 RM25 后重新生成派生模型：

```bash
python3 sentry_nav/src/sentry_sim_bringup/models/generate_sentry_model.py
```

### 自旋控制

```bash
ros2 topic pub --once -w 2 --keep-alive 1 /cmd_spin example_interfaces/msg/Float32 "{data: 3.14}"
ros2 topic pub --once -w 2 --keep-alive 1 /cmd_spin example_interfaces/msg/Float32 "{data: 0.0}"
```

启动参数 `spin_speed:=3.14` 可开启自旋。网关等待 LIO 输出后允许执行，角加减速度限制为 1 rad/s²，从 3.14 rad/s 停转约需 3 秒。云台对地稳向读取 IMU 和关节反馈；`gimbal_world_lock:=False` 可关闭。

### Foxglove 与运行状态

```bash
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix pb2025_nav_bringup)/share/pb2025_nav_bringup/rviz/nav2_default_view.rviz" --ros-args -p use_sim_time:=true
ros2 launch foxglove_bridge foxglove_bridge_launch.xml use_sim_time:=true
ros2 lifecycle get /controller_server
ros2 topic hz /odometry
ros2 run tf2_ros tf2_echo map chassis
```

Foxglove 连接 `ws://localhost:8765`。完整的本机动态验收结论、最新模型更新后的复验范围，见 [2026-10-22 联调记录](docs/validation/2026-10-22.md)。重启前退出旧启动实例，避免重复节点和仿真时钟回跳。


## UL 开局与隧道业务

本分支基于已完成 LIO 验证的 `5a1888d`，未合并主线后续的双机 YAstar 方案。新增 `sentry_bot/` 业务工作空间和 `operator_console/` 操控台，配套 2027 场地、地图和启动选择。Nav2 负责开阔区域，专用闭环直线控制负责隧道，定位保持 Point-LIO + small_gicp，未接入 z_nav。机器人和雷达安装保持不变。

按前文构建仿真与导航工作空间后，构建业务层：

```bash
source source_all.bash
cd sentry_bot
colcon build --base-paths src --symlink-install
cd ..
source source_all.bash
ros2 launch sentry_business_bringup opening_simulation_launch.py
```

另一个终端执行 `bash operator_console/start.bash`，打开 http://127.0.0.1:8989/ ，载入默认三点、编辑坐标、启动开局任务。不要并行启动第二个仿真实例。

原运行工作区已连续通过 10 次完整开局、隧道正反方向各 10 次；最大横向误差约 1.96 cm，测得侧壁最小余量约 9.64 cm。运动中取消、实体障碍物停车和数据失效保护测试通过。移入本仓库时仅适配地图工具的两种目录布局；记录不代表在新的工作区又运行了一遍完整仿真。

见 [业务说明](sentry_bot/README.md)、[验证记录](sentry_bot/validation/implementation_progress.md) 和 [操控台说明](operator_console/README.md)。只提交源码、地图、场地和验证资料，不包含构建产物或原始建图录包。实物验证和裁判占领计时不在本次范围内。
