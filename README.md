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

仅保留双车完整仿真入口：自动哨兵 `sentry_robot1` 使用 **Point-LIO + small_gicp 传感器定位和二维 YAstar 全局规划**，手动车 `rm25_robot1` 用于制造动态障碍，不运行导航或车间通信；不生成 robot2。Gazebo 雷达和 IMU 提供定位输入；真值仅供误差观测。`localization:=ground_truth` 保留为显式调试选项，Theta* 保留为显式比较选项，无自动回退。

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
ros2 launch sentry_sim_bringup dual_sentry_simulation_launch.py \
  spin_speed:=0.0 use_rviz:=True
```

全量构建需要安装上游导航包依赖。如果 small_gicp 的 FetchContent 无法访问 GitHub，可在相应构建命令中追加 `-DFETCHCONTENT_SOURCE_DIR_SMALL_GICP=/path/to/local/small_gicp`，替换为已有源码的绝对路径。新终端从仓库根目录执行 `source source_all.bash`；该脚本也兼容本机 `gazebo/` 与 `sentry_nav/` 并列的布局。

该入口自行生成两车，不使用 `gz_world.yaml` 的机器人列表。哨兵出生点为 `(-5.3, 3.0, 0.3), yaw=-pi/2`，手动车为 `(5.3, -3.0, 0.3), yaw=0`。不要同时运行原始 `rmu_gazebo_simulator bringup_sim.launch.py` 或叠加第二个场景。默认生成延迟为 25 秒，等待模型落地和定位、Nav2 激活后再发送目标。

另开终端，从仓库根目录加载环境后控制手动车：

```bash
source source_all.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/rm25_robot1/cmd_vel
```

`k` 停车；键盘指令断流默认 1 秒后由仿真网关输出零速度。该命令不控制自动哨兵。若未安装键盘工具，安装 `ros-humble-teleop-twist-keyboard`。

显式调试或对比时，先退出当前场景，再用同一入口增加 `localization:=ground_truth` 或 `planner_plugin:=nav2_theta_star_planner/ThetaStarPlanner`。YAstar 源码在根工作区 `src/pb2025_sentry_nav/tdt_yastar_planner/`，必须先构建并加载根工作区，再构建 overlay。插件基于 Nav2 已膨胀的二维 costmap，未知区域默认不可通行；不包含 TDT 的 Kinodynamic A* 或 MinimumSnap。导航行为树以 3 Hz 重规划，失败后停车、等待 1 秒并有界重试，不执行自动旋转或倒车。

### 建图、地图与坐标

纯 3D 建图时不启动 Nav2 或 small_gicp：

```bash
ros2 launch sentry_sim_bringup dual_sentry_simulation_launch.py \
  localization:=lio slam:=True spin_speed:=0.0
```

等待落地和 LIO 初始化后，用 `/sentry_robot1/cmd_vel_chassis` 手动采集，例如将键盘命令重映射到该话题。正常 Ctrl+C 时 Point-LIO 尝试保存 `sentry_nav/src/pb2025_sentry_nav/point_lio/PCD/scans.pcd`。该目录属于本地采集输出；保存成功不代表地图质量合格。

运行先验为 `sentry_nav/src/sentry_sim_bringup/maps/diting_3v3_field_mapped.pcd`，含 242,239 个点，来自传感器建图原始结果的体素降采样，保留原始 camera_init 坐标。约 167 MiB 的 `diting_3v3_field_mapped_raw.pcd` 保留在采集机器上，不随仓库分发。原有 STL 采样 `diting_3v3_field.pcd` 仅作几何对照，二维导航地图仍为同目录的 PGM/YAML。

```text
map → sentry_robot1/odom → sentry_robot1/base_footprint → sentry_robot1/chassis
                                                        ├── sentry_robot1/gimbal_yaw_fake
                                                        └── sentry_robot1/gimbal_yaw_odom
                                                              → sentry_robot1/gimbal_yaw
                                                              → sentry_robot1/lidar_link
```

Gazebo 原始点云 `/sentry_robot1/points_raw` 经适配生成 `/sentry_robot1/velodyne_points`；IMU 输入为 `/sentry_robot1/imu/data`。点云经采样时刻的完整 TF 转到 `sentry_robot1/chassis`，再供 Point-LIO 使用。Gazebo 整帧点云统一使用 `time=1 µs`，关闭体素时间混合；当前 Point-LIO 大批量测量分支使用逆方差权重，配置 `lidar_meas_cov: 1000.0`，`acc_norm: 9.8`。这些参数针对当前仿真输入，不直接作为实物雷达标定。

局部、全局 costmap 使用从实时点云生成的 `/sentry_robot1/obstacle_scan` 和标准 `ObstacleLayer`。更新后的轮心为 `(±0.21152, ±0.24318, 0)`，轮胎碰撞包络约 0.391 m，故两种定位配置的 costmap 半径均为 **0.40 m**。这是导航包络，不修改模型碰撞几何。

雷达仍装在哨兵云台 yaw 上，安装俯仰约 12°；对应 TF 平移为 `(0.15, -0.12, 0.0306)`。修改共用 RM25 后重新生成派生模型：

```bash
python3 sentry_nav/src/sentry_sim_bringup/models/generate_sentry_model.py
```

### 自旋控制

```bash
ros2 topic pub --once -w 2 --keep-alive 1 /sentry_robot1/cmd_spin example_interfaces/msg/Float32 "{data: 3.14}"
ros2 topic pub --once -w 2 --keep-alive 1 /sentry_robot1/cmd_spin example_interfaces/msg/Float32 "{data: 0.0}"
```

启动参数 `spin_speed` 默认 3.14 rad/s；基础导航和动态障碍验收建议显式传入 `spin_speed:=0.0`。LIO 网关等待里程计后允许执行，非零角速度变化限制为 1 rad/s²，收到零角速度则立即置零。云台对地稳向读取 IMU 和关节反馈；`gimbal_world_lock:=False` 可关闭。该就绪检查仅检查里程计到达，不代表 GICP 匹配质量保护；手动初值及定位保护逻辑本次未修改。

### Foxglove 与运行状态

```bash
# use_rviz:=True 已加载相同仿真专用 RViz 配置；手动打开时使用：
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix pb2025_nav_bringup)/share/pb2025_nav_bringup/rviz/nav2_default_view.rviz" --ros-args -r __ns:=/sentry_robot1 -p use_sim_time:=true
ros2 launch foxglove_bridge foxglove_bridge_launch.xml use_sim_time:=true
ros2 lifecycle get /sentry_robot1/controller_server
ros2 topic hz /sentry_robot1/odometry
ros2 run tf2_ros tf2_echo map sentry_robot1/chassis
```

Foxglove 连接 `ws://localhost:8765`。完整的本机动态验收结论、最新模型更新后的复验范围，见 [2026-10-22 联调记录](docs/validation/2026-10-22.md)。重启前退出旧启动实例，避免重复节点和仿真时钟回跳。
