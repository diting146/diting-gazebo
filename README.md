# Diting Gazebo Simulation

## 当前入口与状态（2026-11-06）

唯一推荐完整入口为“自动哨兵 + 键盘手动车”双车仿真，默认使用 **Point-LIO + small_gicp 定位、二维 YAstar 全局规划、原全向控制器与隧道 BT**。默认就绪后执行固定三点路线；模拟血量和低血返出生点已启用，不使用实体串口。默认自旋为0；显式启用自旋时，固定三点任务在整车出洞且安全后启旋，到最终目标后停止平移并保留自旋，取消或失败关闭自旋。

```bash
source /home/hui/diting-gazebo-feat-sentry-nav-simulation/source_all.bash
ros2 launch sentry_sim_bringup dual_sentry_simulation_launch.py use_rviz:=True
```

首次部署、依赖安装和两层构建请以[当前构建与依赖说明](docs/当前构建与依赖说明_2026-11-06.md)为准；血量操作见[sentry_nav当前说明](sentry_nav/README.md)。下方按日期记录的试验参数、通过次数和结论只对应当时版本。当前仍有洞内碰撞检查失败样本，未声称长期稳定；前置途径点试验没有接入默认任务。

本轮逐项处理进度与验证边界见[结构审计整改待办](docs/结构审计整改待办_2026-11-06.md)。

正常入口的隧道几何、软代价ROI和默认三点/yaw统一来自 `sentry_nav/src/sentry_sim_bringup/config/scene_geometry.yaml`；可用 `scene_file:=/绝对路径/scene_geometry.yaml` 替换。显式路线及自定义参数/BT的优先级见[场地几何配置说明](docs/场地几何配置_2026-11-06.md)。旧模板中的坐标仅兼容回退，单改旧数字不会改变正常入口生成结果。

构建优先使用 `bash "$WS/tools/build_current.bash" all --check` 检查，再按需执行 `root`、`overlay` 或 `all`；实际构建前先安装依赖。该入口固定包范围并隔离两层环境，详见[包归属与受控构建](docs/包归属与受控构建_2026-11-06.md)。下方详细手动构建命令保留。

## 2026-11-01 YAstar 几何优化与附近定位（历史记录）

当前仍是 Nav2 + 二维 TDT YAstar + 原控制器。YAstar 后增加 TDT 闭式 minimum-jerk 几何路径优化，LIO/GT 仿真显式启用；失败回退已验证原路径，取消则终止规划。洞内保持原专用轴线。它不是按时间跟踪的速度/加速度优化，尚未证明整条路线提速或长期成功率提高。

仿真 `spawn_x/y/z/yaw` 与地图定位 `initial_pose_x/y/z/yaw` 独立。默认值保持原出生区域；LIO 固定路线等可信配准及原全部条件连续稳定3秒后出发，记录实际 `route_status.home_pose`，重复任务不覆盖，时钟回退清空并立即发布。只处理出生区域附近摆放误差，不做全场重定位、返航或运行中定位异常停车；RViz直接目标不受这项启动门控限制。

52项 C++测试、最终140项Python测试通过。默认/重复三点路线和一个偏移0.25m、朝向偏9.79°的出生样本成功；配准不合格时18秒未发目标，行进转向取消后实际停车。GT开启优化原配置复测成功。仍保留一次洞内偏差失败和一次GT规划拒绝，复测通过不能代表根因修复。当前为功能集成测试版。

```bash
source /home/hui/diting-gazebo-feat-sentry-nav-simulation/source_all.bash
ros2 launch sentry_sim_bringup dual_sentry_simulation_launch.py use_rviz:=True
```

使用新终端并先关闭旧完整仿真实例。固定路线、速度、地图、车身轮廓和扫描守卫沿用现有版本。此前 DDS 接收缓存与独立 odom/TF 回调修复保持；下方日期较早的章节是历史记录。

详见 [验收说明](docs/轨迹优化与附近定位验收_2026-11-01.md)、[算法边界](docs/YAstar轨迹优化与定位边界_2026-11-01.md)、[操作与回退](docs/轨迹优化与附近定位使用_2026-11-01.md)。

## 2026-10-31 路线与迁移准备（历史记录）

当前为**提速测试版**。最后单世界复核首轮25.853仿真秒、重复41.000秒均成功；此前存在扫描空档和定位输入超时失败样本，尚不能称稳定版。受测UDP接收路径已确认有丢包，具体与两条断流链的对应关系待集中排查，未改系统网络参数。

固定三点仍为 **(-4.0,3.475) → (-2.5,3.475) → (0,0)**，默认导航就绪后自动开始一次，结束后可调用原有 start_route 再次执行；取消使用 cancel_route。普通最高1.0m/s；距洞外等待点1.0m内锁存0.45m/s，洞内仍0.8m/s，完整出洞恢复普通速度。

入口外有完整旋转圆盘能通过的短直线空间时，哨兵边走边将真实底盘车头转向隧道，再停车完成原有精调/对中/入洞验收。条件不满足继续原停车对齐流程。控制器、地图、模型、完整footprint和0.8秒数据时效保护保持；不会全局开启自旋或旋转恢复。

扫描链路现显式只积压3帧，BT防止旧扫描覆盖新缓存，云台动态TF发布上限100Hz并保留采集时间戳。**仍记录到一次转换器等待导致扫描空档，尚不能宣称偶发扫描超时根治。**性能、最终复测次数及限制见本轮验收记录，旧章节数字只表示当时版本。

```bash
source /home/hui/diting-gazebo-feat-sentry-nav-simulation/source_all.bash
ros2 launch sentry_sim_bringup dual_sentry_simulation_launch.py use_rviz:=True
```

请使用新终端，并先关闭已有完整仿真实例。仅手工发目标时加 auto_nav_to_origin:=False；原有三点任务服务和状态话题不变。

小电脑移植前先阅读部署说明：当前仿真网关不是真实底盘驱动，旧reality入口也不是当前YAstar+隧道框架的已验收硬件版本；先确定ROS速度坐标/单位、真实反馈、TF与超时停车责任，具体线缆协议待定。

部署文档：[迁移清单](docs/小电脑迁移准备清单_2026-10-31.md)、[底盘通信与真机边界](docs/底盘通信与真机边界审计_2026-10-31.md)、[本轮修改与验收](docs/路线提速与扫描修正_2026-10-31.md)。

ROS 2 Humble / Gazebo Fortress 导航仿真工作区。当前只保留“自动哨兵 + 键盘手动车”的完整仿真入口。

## 2026-10-30 三点任务初版（历史记录）

当前自动任务通过 Nav2 `NavigateThroughPoses` 按顺序执行 map 坐标 **(-4.0,3.475) → (-2.5,3.475) → (0,0)**，最后到原点。旧参数名 `auto_nav_to_origin` 保留兼容，默认 True；其含义现为启用这条三点任务并按默认自动开始，不再只发送一个原点目标。任务完成、失败或取消后不自动循环。建图模式不执行该任务。

默认启动（一个 Gazebo 世界、自动哨兵、键盘手动车及 RViz）：

```bash
source /home/hui/diting-gazebo-feat-sentry-nav-simulation/source_all.bash
ros2 launch sentry_sim_bringup dual_sentry_simulation_launch.py use_rviz:=True normal_speed:=1.0
```

默认自动启动模式也保留开始与取消服务，任务结束后可直接再次调用，无需重启仿真。若只想手动开始，关闭旧实例后使用以下服务模式启动：

```bash
source /home/hui/diting-gazebo-feat-sentry-nav-simulation/source_all.bash
ros2 launch sentry_sim_bringup dual_sentry_simulation_launch.py use_rviz:=True normal_speed:=1.0 auto_nav_to_origin:=False enable_route_task:=True route_auto_start:=False
```

新终端先加载同一环境，再按需要调用：

```bash
source /home/hui/diting-gazebo-feat-sentry-nav-simulation/source_all.bash
ros2 service call /sentry_robot1/start_route std_srvs/srv/Trigger '{}'
ros2 topic echo /sentry_robot1/route_status std_msgs/msg/String --qos-durability transient_local --qos-reliability reliable --once
ros2 service call /sentry_robot1/cancel_route std_srvs/srv/Trigger '{}'
```

`route_status` 为 JSON 字符串，包含 `run_id`、`state`、`last_result`。忙碌时再次 start 会拒绝，不排队；cancel 响应表示请求已发出，要等同一 `run_id` 的终态 `state=idle` 和 `last_result.outcome=canceled` 后再开始。正常完成为 `succeeded`，失败为 `failed`；以实际状态和日志为准。

取消任务使用 `cancel_route`；单独对任务管理节点按 Ctrl+C 不会替代 Nav2 目标取消。本轮主动取消实测覆盖洞外对齐阶段，其他阶段仍需另行验收。

当前普通目标检查器容差 **0.5 m**，独立隧道目标检查器容差 **0.15 m**；普通控制器的接近目标减速距离 **0.9 m**，隧道控制器 **0.6 m**。距洞外等待点大于 **1.5 m** 时保留普通 **1.0 m/s** 上限；进入 **1.5 m** 范围后锁存入口 **0.3 m/s** 限速，避免远处就全程慢速接近。参数为 `approach_slowdown_distance`，距离针对洞外等待点计算。洞内上限 **0.8 m/s**，整车出洞后恢复普通速度上限。目标容差不替代完整车身碰撞检查，途经点也不等于每个点精确停车。

本轮已验证默认自动首轮、结束后手动再执行一轮、忙碌时拒绝重复开始，以及准备阶段主动取消并实际停车；最终构建、24 项 GTest 与87项 pytest 通过。首轮发送到成功为31.967仿真秒，详细时间口径与限制见[过洞流程优化记录](/home/hui/文档/ChatGPT/导航仿真/过洞流程优化_2026-10-30/修改与验收说明.md)。本次未扩展返航、隧道增益或禁行区域功能。明确标注为历史记录的章节保留当时参数与验收数字；当前功能和操作段以本轮同步后的说明为准。

## 当前功能

- 自动哨兵 `sentry_robot1`：出生 (-4.6,3.475,0.3)，yaw=0，车头正对北侧隧道口（距原出生点约0.846m）；默认地图初猜同步，使用 Point-LIO + small_gicp 定位与 YAstar 全局规划。
- 手动车 `rm25_robot1`：出生 (5.3,-3.0,0.3)，yaw=0，用于人工干扰哨兵路线，不运行导航或协同通信。
- 不生成 robot2；local/global costmap 使用底盘坐标系下 0.576 × 0.534 m 的矩形 footprint，每侧 padding 0.005 m（2026-10-29 试验值）；模型和雷达安装不变。
- 默认导航就绪后，通过一次 NavigateThroughPoses 执行 map 坐标 (-4.0,3.475) → (-2.5,3.475) → (0,0)，路线 yaw=0；auto_nav_to_origin 默认 True 兼容保留。任务终态后可再次 start_route；建图模式不触发。
- 普通速度上限默认 1.0 m/s（normal_speed:=1.0）；距洞外等待点超过1.0 m时解除区域限速，进入后锁存入口0.45 m/s，洞内上限0.8 m/s，整车出洞后恢复普通上限。前视0.2–0.4 m、前视时间0.5 s；普通/隧道接近目标减速距离0.9/0.6 m，普通到点容差0.5 m，隧道checker容差0.15 m。PID与完整车身碰撞参数保持。
- 路线完整穿过已标定隧道时，BT执行洞外安全短直线行进转向、减速停车、必要的车头精调、连续闭环居中与停稳确认、重规划穿越，整车出洞后恢复原目标；tunnel_passage:=False 可关闭此阶段。
- 正常 3 Hz 重规划，失败等待 1 秒，最多恢复重试 3 次，持续失败返回 ABORTED；不自动旋转、倒车或清空整个代价地图。
- 手动车按住键持续驾驶，1 秒断流后下发零指令；按 k/空格或正常退出可主动停车，底盘另有物理减速时间。

## 环境与构建

使用 Ubuntu 22.04、ROS 2 Humble 和 Gazebo Fortress。先按[当前构建与依赖说明](docs/当前构建与依赖说明_2026-11-06.md)安装限定范围的依赖，再使用新终端依次构建。根层只选当前入口所需的6包，overlay按依赖闭包构建17包；两层都必须指定 `--base-paths src`，避免递归发现同名包。不要在已经加载overlay的终端中重建根层。

```bash
source /opt/ros/humble/setup.bash
export WS=/home/hui/diting-gazebo-feat-sentry-nav-simulation
cd "$WS"
colcon build --base-paths src --symlink-install \
  --packages-select rmoss_gz_plugins rmu_gazebo_simulator pb2025_robot_description \
  pb_omni_pid_pursuit_controller tdt_yastar_planner sentry_planner_server \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source "$WS/install/setup.bash"
cd "$WS/sentry_nav"
colcon build --base-paths src --symlink-install \
  --packages-up-to sentry_sim_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
```

既有碰撞修复需构建控制器；本轮三点任务和过洞优化只增量构建 overlay 中的 sentry_tunnel_bt、sentry_sim_bringup 两包，本机已完成。

## 哨兵导航工作区（sentry_nav）

唯一完整仿真入口：

```bash
source /opt/ros/humble/setup.bash
export WS=/home/hui/diting-gazebo-feat-sentry-nav-simulation
source "$WS/install/setup.bash"
source "$WS/sentry_nav/install/setup.bash"
ros2 launch sentry_sim_bringup dual_sentry_simulation_launch.py use_rviz:=True normal_speed:=1.0
```

旧的单车仿真启动文件已删除。双车入口直接生成两台机器人并按 localization 参数加载定位导航组件；组件不负责创建单车仿真世界。不要叠加启动多个世界。

- 默认自旋关闭（spin_speed=0）；显式传 `spin_speed:=3.14` 时，固定三点任务先关闭自旋完成接近、对齐和穿越，整车出洞且旋转包络安全后启旋，到最终目标后停止平移并保留自旋，取消或失败关闭自旋。关闭固定任务时保持立即自旋语义；RViz单目标不自动继承固定路线的出洞启旋设置。
- `use_rviz:=True` 请求启动 `/sentry_robot1` 命名空间的 RViz，Fixed Frame 保持 `map`。新启动日志会打印 `Sentry simulation settings`，先核对实际 `use_rviz` 和 `normal_speed`。
- normal_speed 必须为有限正数，启动时覆盖普通 FollowPath 的 v_linear_min/max。隧道 FollowPathTunnel 使用独立0.45/0.8 m/s区域上限，不随普通速度提高；SpeedLimit=0 表示解除区域限速，停车使用零Twist。
- 默认25秒后生成模型，任务每次开始等待Nav2、里程计/扫描/TF连续可用3秒后发送；没有自动循环，任务终态后可显式start_route再次执行。
- 固定三点任务action为 /sentry_robot1/navigate_through_poses，RViz单目标仍可使用 /sentry_robot1/navigate_to_pose。默认LIO按固定出生点给出初始猜测，small_gicp支持 /sentry_robot1/initialpose 修正。
- Theta* 代码与显式规划器切换入口保留；当前默认 YAstar，不自动回退。
- 当前入口默认启动 LIO；通行流程由 Nav2 内的 BT 插件执行。固定任务默认启动模拟血量节点，由任务节点负责低血返航；建图模式、关闭固定任务或 `enable_sim_referee:=False` 时不启用该血量入口。独立隧道监测、真实裁判串口和机器人间协同未启用。上游算法和实验模块源码保留，未作为新的仿真入口启用。

## 键盘控制

```bash
source /home/hui/diting-gazebo-feat-sentry-nav-simulation/source_all.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/rm25_robot1/cmd_vel -p speed:=0.2 -p turn:=0.5
```

焦点放在键盘终端。`i` 前进，`,` 后退，`j/l` 旋转，大写 `J/L` 平移，`k` 或空格停车。出生点靠近边界，首次可先后退进入空旷区域。

新终端可使用 `source "$WS/source_all.bash"` 一次加载 ROS、根工作区和 overlay。`localization:=ground_truth` 显式选择真值调试，`localization:=lio slam:=True` 仅采集三维点云。详细差异见 [定位同步说明](docs/定位同步说明_2026-10-26.md)。

碰撞轮廓随真实 chassis 旋转；控制器继续在 gimbal_yaw_fake 坐标系输出原有速度，避免重复转换。仿真显式启用完整凸包扫掠检查，覆盖车体内部、四角、规划路径段及 0.5 秒指令投影；未知区域、越界和 TF 失败停车。观察到转动时使用保守的整圈旋转包络。二维 YAstar 保留，当前不保证任意底盘朝向均能自动对齐通过窄道。

当前场地的两个隧道启用 YAstar 局部软代价补偿：仅在配置的 `map` 矩形区域内，把原权重 `w=max(1,c/10)` 改为 `1+0.1*(w-1)`。区域包含隧道和沿轴线两端各 0.5 m 的入口延伸。墙体、未知格及完整车身碰撞仍然禁止；区域外和 local/global costmap 原始值不变，因此 RViz 膨胀层颜色不随此补偿改变。它降低绕行倾向，不保证任何起终点都必须穿洞，也不使碰撞目标合法。

参数位于 LIO / Ground Truth YAML 的 `planner_server.ros__parameters.GridBased`：`tunnel_cost_enabled` 开关、`tunnel_cost_scale`（大于 0 且不大于 1，1 恢复原权重）、`tunnel_cost_regions`（每四个数为全局坐标系下 `[min_x,min_y,max_x,max_y]`，米）。这些参数在规划器 configure 时读取，修改后需要重新加载；只适用于已标定的当前场地地图，换图/建图坐标系变化时须关闭或重新标定。独立自旋仍须关闭；区域补偿不包含入口对齐、自动限速或任务管理。

完整操作、检查命令及已知问题见 [ACCEPTANCE.md](ACCEPTANCE.md)。其中旧单车验证属于历史记录，不代表当前仍保留该入口。

## 代码与配置来源

- 场地与普通机器人模型：`src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/`。
- 当前双车生成、桥接和固定出生参数：`sentry_nav/src/sentry_sim_bringup/`。
- 规划器：`src/pb2025_sentry_nav/tdt_yastar_planner/`。
- 上游来源和许可证说明见 [UPSTREAM.md](UPSTREAM.md)。旧 RM25 裁判/模型演示源码保留作为上游资源，不是当前推荐启动入口。

构建产物、运行日志及临时点云不应上传仓库。

键盘终端也需加载 `source_all.bash`，使 ROS 域及 DDS 传输配置与仿真一致。环境脚本现提供显式 UDP 配置，保留用户已有 DDS profile；不要直接把键盘改发到 `cmd_vel_safe`，否则绕过断流停车保护。

## 2026-10-29 自主过隧道测试版（历史记录）

本节保留当时全场 0.4 m/s 的配置、命令和验收证据；已被 2026-10-30 的普通／隧道速度分离配置取代。当前请使用上方 `normal_speed:=1.0` 完整命令，手动验收另加 `auto_nav_to_origin:=False`；若复现旧速度，应显式传 `normal_speed:=0.4`。历史成功记录不等于当前普通 1.0 m/s 配置的验收结果。

默认 LIO + YAstar 的正式安装版本已完成两条实际隧道的双向穿越；出生点直接下达北侧另一端目标、洞内目标、普通原点导航等 8 个目标均 SUCCEEDED，已有启动隔离和速度网关回归 24 项通过。记录了连续 Gazebo 真值，确认经过中央顶棚下方，没有绕行。这是当前固定底盘朝向、关闭自旋、空隧道与指定目标的测试结果，不代表任意目标、姿态、动态阻挡均已验收。Ground Truth 配置同步参数，本轮完整通行实测使用 LIO。

控制器只改两份仿真 YAML 的 FollowPath 参数：

| 参数 | 修改前 | 测试版 |
| --- | --- | --- |
| `min_lookahead_dist` | 0.5 m | 0.2 m |
| `max_lookahead_dist` | 1.0 m | 0.4 m |
| `lookahead_time` | 1.0 s | 0.5 s |
| `v_linear_min / max` | -2.5 / 2.5 m/s | -0.4 / 0.4 m/s |
| `min_approach_linear_velocity` | 0.5 m/s | 0.15 m/s |

当时低速配置减少入口切角，采用全场限速，尚未实现隧道区域专属限速；该速度策略现已由分离配置取代。车身轮廓及每侧 5 mm 余量、5 cm 栅格、膨胀层、0.5 s 指令预测、完整路径碰撞检查保持本轮前状态。

另修正 Gazebo 碰撞资源：新增 `resource/models/diting_3v3_field/meshes/3v3_field_collision.stl`，场地 `model.sdf` 的 collision 引用它；visual 继续引用原始 `3v3_field.stl`。仅把两条隧道底板顶面降低 0.5 mm，与公共地板齐平，消除当前麦轮圆柱碰撞体侧移时的入口卡滞。墙体、顶棚、机器人模型和二维 PGM 未改。原网格低速试验有速度指令却停在入口；同参数整平后穿越成功。新增资源需 `colcon build --base-paths src --packages-select rmu_gazebo_simulator --symlink-install` 安装；本机已经安装完成，重启仿真生效。

手动验收使用同一入口，关闭默认原点任务：

```bash
source /home/hui/diting-gazebo-feat-sentry-nav-simulation/source_all.bash
ros2 launch sentry_sim_bringup dual_sentry_simulation_launch.py use_rviz:=True auto_nav_to_origin:=False
```
另开终端加载环境后，下达北侧另一端目标（从默认出生点即可）：

```bash
source /home/hui/diting-gazebo-feat-sentry-nav-simulation/source_all.bash
ros2 action send_goal /sentry_robot1/navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: map}, pose: {position: {x: -1.0, y: 3.475, z: 0.0}, orientation: {z: -0.70710678, w: 0.70710678}}}}' --feedback
```
到达后，将目标改为 `x: -4.0, y: 3.475` 验证返程。北侧洞内测试目标 `(-2.43, 3.475)`；南侧先到 `(1.0, -3.475)`，再发 `(4.0, -3.475)`，最后返回 `(1.0, -3.475)`。不需要新增任务节点或运行临时测试脚本。使用中线附近目标，靠墙的目标仍可能被完整车身判据拒绝。


## 2026-10-30 隧道行为树测试版（早期版本，历史记录）

新增 `sentry_nav/src/sentry_tunnel_bt`，在一次 `NavigateToPose` 内执行减速接近→停稳居中→真实chassis车头对齐→重规划并跟踪隧道轴线→整车出洞→恢复原最终目标。入口限速0.3m/s、通行上限0.4m/s独立保持，退出解除区域限速；当前普通导航默认1.0m/s，由normal_speed设置。控制器现支持Nav2绝对值/百分比SpeedLimit，0表示解除限速，停车使用零Twist。

只有当前全局路径完整穿过已标定隧道才启用，不强制选洞。普通导航途中重新规划改为穿洞时，路线守卫会先停止普通跟踪，再按既有有界等待重试重新进入准备。普通路线不进入通行阶段；洞内目标、`NavigateThroughPoses`、非零独立自旋不属于本版自动入口对齐的保证范围。通行失败且车身仍在洞内时停车并明确失败，需人工安全移出再重试。入口和通行继续检查真实完整车身，未知格、墙体、TF或扫描过期不会放行。

手动测试仍显式 `auto_nav_to_origin:=False`，这是关闭自动原点目标，不是自动节点失效。本轮保留现存自动任务代码与开关，后续任务入口整合已记入待办。通过 `tunnel_passage:=False` 可以保留当前规划/控制及有界等待恢复，只跳过新增通行准备阶段。插件需退出旧实例后重新启动加载。

流程、参数与限制见 [sentry_tunnel_bt README](sentry_nav/src/sentry_tunnel_bt/README.md)。10月29日的验收段落为历史版本记录；最新验证另行归档。

速度分离之前、普通上限仍为0.4m/s的最终安装版记录：六个固定目标均SUCCEEDED，南北各双向完整走完专用通行流程；接近/转向取消能停车，入洞后动态封堵三次重试后明确ABORTED。BT18项、控制器15项GTest和启动/网关pytest26项通过。完整实测采用LIO，GT仅配置与回归验证；测试不代表任意目标或长期成功率。本轮自有实例已退出。

## 2026-10-30 普通／隧道速度分离与 RViz 排查

`normal_speed` 必须为有限正数，单位 m/s；它在启动时覆盖所选 `params_file` 中 FollowPath 的 `v_linear_min/max` 为 `-normal_speed/+normal_speed`，不改其他参数。隧道 0.3/0.4 m/s 不随普通速度提高，出洞发布 `SpeedLimit=0` 解除区域限制并恢复普通上限；0 不是停车命令。现有自动原点开关仍默认 True，手动验收显式关闭，与速度和 RViz 开关独立。

10 月 30 日 15:30 会话日志没有创建 RViz 节点，不属于“RViz 已启动后崩溃”的证据；原会话已退出，完整历史 CLI 未留存，不能据此断言当时输错参数。当前完整 LaunchService 的 True/False 两组 dry 检查分别执行／跳过 RViz 启动动作，抑制了实际进程，不能当作 RViz GUI 已测试通过。以新日志 `Sentry simulation settings` 显示的有效 `use_rviz`、`normal_speed` 为准。当前普通1.0m/s配置已完成构建、57项启动回归和18项控制器GTest。受控制动样本在实速0.802m/s切换任务后，约1.290仿真秒、0.630m位移后首次停稳；随后入口指令≤0.3、洞内≤0.4m/s，完整出洞并恢复普通实速峰值0.999m/s。该样本不代表真机制动距离或长期成功率。南北四方向完整成功来自三次独立运行，另有一次扫描超时及一次出口附近偏离保护停车记录，不记成一次连续四向全通过。详见本机记录：[速度分离验收说明](/home/hui/文档/ChatGPT/导航仿真/速度分离与RViz排查_2026-10-30/修改与验收说明.md)。
