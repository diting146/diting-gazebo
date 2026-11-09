# 自动哨兵与键盘手动车验收

更新日期：2026-10-27。当前只保留双车完整仿真入口，旧单车启动文件已删除。

## 固定范围

- 自动哨兵 sentry_robot1：(-5.3,3.0,0.3)，yaw=-pi/2；Point-LIO + small_gicp + YAstar。
- 手动车 rm25_robot1：(5.3,-3.0,0.3)，yaw=0；只用底盘键盘控制，不运行 Nav2。
- robot2 不生成；模型、雷达、地图与既定恢复策略保持不变；local/global costmap 使用 0.576 × 0.534 m 底盘矩形 footprint，每侧 padding 0.01 m。
- 独立自旋默认关闭；显式 spin_speed:=3.14 可开启。
- 默认启用 LIO；不启动隧道、裁判或机器人间共享数据；上游资源和旧实验模块源码没有作为当前仿真入口运行。

## 2026-10-27：默认关闭自旋补验

- 用户本次运行已加载新矩形 footprint 和完整碰撞检查，但启动只传 use_rviz:=True，
  仍继承旧默认 3.14 rad/s。现场角速度约 3.08 rad/s，控制器启用整圈旋转包络后
  在出生点附近停车；两个目标被接收，最终均为 ABORTED。
- 按用户确认，将共享 DEFAULT_SPIN_SPEED、LIO/GT 配置初始自旋统一为 0.0。
  双车、LIO、GT 三个入口均默认关闭；spin_speed:=3.14 继续支持显式开启。
- 对日志中的 (-0.49,2.96)、(-4.73,0.85)，关闭自旋后分别从原出生点在独立
  LIO+YAstar 场景复测，均 SUCCEEDED。另以不传 spin_speed 的真实默认启动复验
  第一个目标，角速度指令 0.0、实际角速度接近零，目标 SUCCEEDED。
- sentry_sim_bringup 构建通过，29 项 pytest 全部通过，新增三入口默认关闭和
  显式开启的参数接线回归；修改 Python 通过 ament_flake8。
- 未向用户现有实例发送控制、取消或新目标，未重启用户实例。旧进程不会热更新，
  需退出后重新启动。未推送 GitHub。

## 2026-10-27：矩形轮廓与完整碰撞检查

- 全模型碰撞几何 AABB 约 0.575015 × 0.531575 m；配置向外取整为
  0.576 × 0.534 m，另加每侧 0.01 m padding。两种定位的两个 costmap
  均使用 chassis 作为真实轮廓坐标系。出生点、模型、地图和 YAstar 源码未修改。
- 控制器显式 control_frame=gimbal_yaw_fake，速度转换链路保持原有语义；
  use_polygon_collision_check 仅在仿真配置开启，默认 false 保持上游兼容。
  完整凸包扫掠检查覆盖内部、边缘、四角、整段平移及 0.5 秒指令预测；
  未知格、越界和 TF 失败停车。局部路径裁剪为完整车体预留地图边界余量。
- 观察到或请求角速度超过 0.1 rad/s 时，使用覆盖整圈旋转的保守外接包络。
  此检查不负责管理独立自旋的启停；本轮验收均显式 spin_speed:=0。
- 独立 LIO+YAstar 仿真：原出生点到 (3.48,2.32) 成功；另一轮依次到
  (-4.8,-0.62)、(3.86,2.36)、(3.86,2.69) 全部 SUCCEEDED。后面三项为
  连续任务，不是各自从原点重新启动。完整轮廓预判曾触发停车和重规划，随后成功。
- 标准测试世界采用 800 mm 净宽、2000 mm 长通道；开顶及中央顶棚
  800 × 350 mm、净高 270 mm 两种场景均在 Ground Truth+YAstar、固定朝向下
  从 (-5.3,3.0) 导航到 (-5.3,-0.2) 成功。该专用几何场景不代表正式场地的
  LIO 隧道定位、入口自动对齐或出口自旋恢复已经完成。
- 手动车控制及断流停车在上述独立验证中通过。
- 26 项 pytest、7 项轮廓 GTest 通过；控制器的格式、许可、XML 等包检查通过。
  colcon 完整汇总 57 tests、0 errors、0 failures、0 skipped，包含测试包装项和 lint 项。
  新增用例覆盖窄道、物理朝向、轮廓内部障碍、角点栅格面积相交、路径点间障碍、
  未知区域、越界与非法几何。首次失败的格式/许可检查和临时路径裁剪实验已保留，
  不计为最终通过结果。完整 pre-commit 工具此前不可用，本轮使用上述 ament 包检查。
- 根工作区控制器与 overlay 控制器/仿真包均已完成本地构建；实际安装路径下
  LIO+YAstar 到 (-5.3,1.5) 返回 SUCCEEDED。实际 overlay 的 26 项 pytest、
  7 项碰撞 GTest 与注册 lint 结果均通过。xmllint 曾因在线 schema 不可访问失败，
  使用 [ROS 官方 schema](https://github.com/ros-infrastructure/rep/blob/master/xsd/package_format3.xsd)
  的本地 XML catalog 重新通过，未跳过 schema 校验或修改 package.xml 的 schema 声明。
- 仅处理本地修复，没有提交或推送 GitHub；用户已有实例未被重启、控制或停止。
  测试清理向自身进程组发 SIGINT，部分节点有 -2/traceback，不作为正常退出验收。
- 当前二维 YAstar 没有底盘朝向搜索，仍可能给出对特定朝向不可执行的路线；
  完整轮廓检查会停车，而不是保证所有窄道转弯均可自动通过。隧道朝向/自旋管理属于下一阶段。

## 新终端启动

```bash
source /opt/ros/humble/setup.bash
export WS=/home/hui/diting-gazebo-feat-sentry-nav-simulation
source "$WS/install/setup.bash"
source "$WS/sentry_nav/install/setup.bash"
ros2 launch sentry_sim_bringup dual_sentry_simulation_launch.py \
  use_rviz:=True
```

主入口直接生成两车、加载各自桥接，并通过隔离的 Include 按 localization 参数选择 lio_navigation_launch.py 或 nav2_ground_truth_launch.py。导航组件保留，不是单车仿真模式。默认 25 秒生成模型；启动 Gazebo 不等于导航就绪。正在运行的旧实例不受源文件删除影响，修改在下次启动生效。

```bash
ros2 lifecycle get /sentry_robot1/planner_server
ros2 lifecycle get /sentry_robot1/controller_server
ros2 lifecycle get /sentry_robot1/bt_navigator
ros2 lifecycle get /sentry_robot1/velocity_smoother
```

都应 active，并确认 /clock 推进、TF 与传感器有效。导航 action 为 /sentry_robot1/navigate_to_pose；RViz Fixed Frame 为 map。默认 LIO 会按固定出生点给出初始猜测，small_gicp 支持 /sentry_robot1/initialpose；Ground Truth 调试不使用该初始化。

```bash
ros2 action send_goal /sentry_robot1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: -5.3, y: 1.5}, orientation: {w: 1.0}}}}' --feedback
```

若只补开 RViz，先 source 环境后执行：

```bash
rviz2 -d "$(ros2 pkg prefix pb2025_nav_bringup)/share/pb2025_nav_bringup/rviz/nav2_default_view.rviz" \
  --ros-args -r __ns:=/sentry_robot1 -p use_sim_time:=true
```

## 手动车控制

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/rm25_robot1/cmd_vel -p speed:=0.2 -p turn:=0.5
```

焦点在键盘终端并按住移动键。i/逗号前后，j/l旋转，大写J/L平移，k/空格发零，Ctrl+C正常退出发零。松键约1秒无新命令后，网关才发零，之后还有物理减速；键盘程序本身没有松键检测。

链路：键盘 /rm25_robot1/cmd_vel → 独立1000ms网关 → /rm25_robot1/cmd_vel_safe → Gazebo /rm25_robot1/cmd_vel。手动车不响应哨兵自旋话题。

```bash
ros2 topic pub --once /rm25_robot1/cmd_vel geometry_msgs/msg/Twist '{}'
```

主动停车时先停止持续运动发布，以免后续运动指令覆盖零命令。

## 当前导航与恢复边界

- 自动哨兵使用 YAstar、OmniPid、costmap、LIO 定位、云台 TF 与点云过滤；手动车不运行这些导航模块。哨兵 frame 使用 sentry_robot1 前缀，map 与 /tf、/tf_static 传输保持全局。
- 自动哨兵：控制器 → 平滑器 → fake_vel_transform → /sentry_robot1/cmd_vel_chassis → 200ms / 50Hz网关 → /sentry_robot1/cmd_vel → Gazebo。
- 当前BT正常3Hz重规划，失败后等待1秒，最多恢复重试3次；不调用Spin、BackUp或清空整个costmap。持续失败ABORTED；取消会停止FollowPath或Wait。重试有界不等于整任务有绝对总时限。
- controller failure_tolerance保持0.6秒。下游物理减速与命令清零分开验收。
- 自旋控制话题为 /sentry_robot1/cmd_spin，完整停车使用spin_speed:=0。
- 多途经点RemovePassedGoals显式使用sentry_robot1/gimbal_yaw_fake，避免默认base_link错误。
- 普通手动车模型及其传感器、云台等部件不精简；当前仍只桥接命令和里程计。
- 已有一个窄道居中阻挡案例会因无进展失败，仍待优化；不把偏置障碍案例成功当作所有动态避障通过。

## 2026-10-27：仿真碰撞半径对齐远端

- 按用户确认，仅将 sentry_sim_bringup 的 LIO 与 Ground Truth 两份配置中的 local/global costmap robot_radius 统一为 0.4 m（直径 0.8 m），与远端 main 一致。
- 更新已有启动契约测试的期望值；同步当前操作文档。以下历史记录中的 0.266 m 是当时使用值，不代表当前配置。
- 手动初值覆盖、定位保护、算法实现、模型几何、速度与恢复策略均不属于本轮修改范围。旧工作区 /home/hui/diting_gezabo 未修改。
- 增量构建成功；已有 25 项回归全部通过，0 errors / 0 failures / 0 skipped。源码与安装副本 YAML 均解析成功，LIO/GT 的四处值均为 0.4；语义比较确认两个 YAML 只改变 robot_radius。
- 本轮未启动仿真；配置文件变更不等于已有进程热更新，重新启动后采用新值。

## 2026-10-26：同步远端传感器定位与环境脚本

- 同步来源：main / 5a1888d90cb4a9c3ff6ad4b37e15d1f2d8738b84。Point-LIO、small_gicp、loam_interface、sensor_scan_generation 的 80 个已跟踪文件与远端一致，因此未重复修改或构建这些 C++ 核心。
- 唯一双车入口默认 localization:=lio；用户明确确认默认切换后生效。localization:=ground_truth 保留原 GT 组件、速度网关和云台控制代码。没有恢复单车入口。
- 适配 remote 的点云格式转换、IMU 云台反馈、参考真值和同时间戳误差观测；同步 242239 点的先验 PCD 和 source_all.bash。LIO 专用网关沿用本地稳态时钟断流停车，并加入定位里程计就绪等待与自旋渐变。
- 本地 YAstar、0.266 m 半径、扫描代价地图、控制器参数和行为树保持不变；未照搬远端 Theta*、0.40 m 半径或地形代价地图配置。
- sentry_sim_bringup 构建成功；25 项回归全部通过，0 errors / 0 failures / 0 skipped；10 个修改/新增 Python 文件通过 ament_flake8。显式 LIO 模式在独立 ROS_DOMAIN_ID=220 / IGN_PARTITION 中实际启动，地图、扫描、运行业务里程计及误差有数据，四个关键 Nav2 lifecycle 节点 active。
- 手动车后退位移 -0.457 m，随后零命令停车；哨兵从 (-5.3,3) 到 (-5.3,1.5) 返回 action status 4 / SUCCEEDED。真值终点约 (-5.258,1.437)。
- 终点一次同时间采样误差：dx=0.002845 m、dy=0.009181 m、dyaw=0.000651 rad，平面误差约 0.009611 m。不是全程 RMSE，也不是全场景精度承诺。
- 运行验证采用无界面世界、start_simulator:=False、spawn_delay:=2、spin_speed:=0。默认世界包含及 RViz 参数由启动契约检查覆盖；本轮未做 GUI 目视、动态堵路、自旋或新地图质量验收。
- slam:=True 的模块构成已由启动测试核对；本轮没有采集、覆盖或重新制作先验地图。临时运行证据：/tmp/diting_lio_sync_20261026/runtime_result.json。桌面指南同步更新。

## 2026-10-26：删除单车入口验证

- 删除 bringup_sentry_simulation_launch.py 源码及对应 build/install 残留链接。双车直接负责世界、两台机器人及桥接，加载同一个 Ground Truth 导航组件；没有第二套完整仿真入口。
- sentry_sim_bringup 增量构建成功；注册 Python 回归 19 项全部通过，0 errors / 0 failures / 0 skipped；修改的 4 个 Python 文件通过 ament_flake8。已验证安装入口的 --show-args 解析。
- 隔离测试使用 ROS_DOMAIN_ID=218 与专用 IGN_PARTITION；无界面 Gazebo 世界运行，实际双车入口以 start_simulator:=False、spawn_delay:=2、spin_speed:=0、use_rviz:=False 接入该世界。
- 两车均有里程计，出生 x/y 分别为 (-5.3,3) 和 (5.3,-3)；地图、/clock、障碍扫描有数据；planner_server、controller_server、bt_navigator、velocity_smoother 全部 active。
- 经手动车原始控制话题发送 -0.2 m/s，实际 x 位移约 -0.457 m；显式零命令后观察到线速度绝对值低于 0.03 m/s。这里只验证启动与控制，没有新增导航到达或动态绕障结论。
- 首次测试探针在生命周期激活完成前读取到 inactive，已保留原始日志；改为等待 active 后完成上述重验。未据此修改导航代码。
- 桌面指南同步为双车唯一入口，6 个 Bash 代码块语法检查通过。旧单车相关验收归入下方历史记录。现有行为树 XML 与模型保持本轮修改前一致。
- 本轮验证进程已清理；没有停止用户实例、提交或推送代码。临时证据目录：/tmp/diting_dual_only_20261026，最终 runtime_result.json 为 DUAL_ONLY_STARTUP_PASS。

复验命令（先加载本文件环境）：

```bash
cd "$WS/sentry_nav"
colcon build --base-paths src --packages-select sentry_sim_bringup --symlink-install
colcon test --base-paths src --packages-select sentry_sim_bringup --return-code-on-test-failure
colcon test-result --test-result-base build/sentry_sim_bringup --verbose
ros2 launch sentry_sim_bringup dual_sentry_simulation_launch.py --show-args
```

## 历史记录

以下记录反映过去版本，含已经删除的单车入口及旧出生点，只用于追溯，不能作为当前启动说明或本轮验证结果。

## 上轮停车修复验证记录（出生点对调前）

2026-10-24 至 2026-10-25，本轮使用当前源模型，分别启动后清理测试实例；未提交或推送代码，未修改旧工作区 /home/hui/diting_gezabo。

| 检查 | 本轮实际结果 |
| --- | --- |
| 根工作区 fake_vel_transform | 增量构建成功；8 项 C++ 行为回归通过 |
| overlay fake_vel_transform / sentry_sim_bringup | 两包构建成功；注册检查共 21 项，0 errors / 0 failures / 0 skipped。其中 C++ 行为回归 8 项、其他注册检查 5 项、Python 测试 8 项 |
| 两份速度转换代码 | 源码、头文件、测试、CMake 和 package.xml 对应一致；加载两层环境后实际解析到 overlay 安装包 |
| Python 风格 | 修改的网关、启动文件和新增测试通过 ament_flake8、ament_pep257 |
| 双车出生与规划器 | 哨兵约 (0,0)，手动车约 (-5.3,3)、yaw=-pi/2；实际加载 tdt_yastar_planner/YastarPlanner |
| 双车 YAstar 到达 | (0,0) → (0,1.5)，action 状态 4 / SUCCEEDED，终点约 (-0.007,1.428) |
| 单车 Theta* 到达 | (0,0) → (0,1.5)，action 状态 4 / SUCCEEDED，终点约 (-0.029,1.432) |
| 取消目标 | 两种入口均返回状态 5 / CANCELED；取消后复测平移速度低于 0.001 m/s |
| 手动车实际运动 | teleop PTY 注入运动键，里程计速度约 0.197 m/s |
| k / Ctrl+C / 强制结束 | 三种情况下实际速度均降至 0.015 m/s 以下；强制结束实际 teleop 节点后约 0.91 秒观察到零速度命令 |
| 控制隔离 | 手动车操作期间哨兵平面漂移约 8.4e-07 m，没有随手动指令运动 |

回归命令（先加载本文件环境）：

```bash
cd "$WS/sentry_nav"
colcon test --base-paths src --packages-select fake_vel_transform sentry_sim_bringup --return-code-on-test-failure
colcon test-result --test-result-base build/fake_vel_transform/test_results --verbose
colcon test-result --test-result-base build/sentry_sim_bringup --verbose
```

详细临时证据位于 /tmp/diting_control_fix_20261024：runtime-results.json、dual_yastar.log、single_theta.log、gateway-tests.xml 和 root-velocity-tests.xml。源码修改前备份在 before/；after/ 和 changes.patch 保存本轮最终修改。

复验说明：首次自动测试终端缺少 DISPLAY，补齐桌面显示环境后重试；一次单车探针在 bt_navigator 激活前发目标被拒绝，已改为等待完整节点就绪；另一次连续复验遇到 Gazebo 创建服务超时、无里程计，改用独立通信分区后完成最终到达验证。失败尝试另存日志，不计作通过，也没有据此修改模型或定位链路。

边界：这些是工具驱动的 PTY/ROS action 验证，不代替用户目视驾驶与动态干扰验收。单车退出时，上游 referee_hp 节点仍有 shutdown traceback，Gazebo 偶尔需要 launch 升级终止信号；本轮进程均已清理，尚未改动这些既有退出处理。双车入口不启动 referee_hp。



## 2026-10-25 单双车统一验收记录

本轮仅修改 `sentry_sim_bringup` 及本文/README；YAstar 源码、机器人 SDF 几何、雷达安装、碰撞半径和旧工作区 `/home/hui/diting_gezabo` 未修改。手动车功能精简暂不执行，未提交或推送 GitHub。

| 验证 | 实际结果 |
| --- | --- |
| 构建与安装 | overlay `sentry_sim_bringup` 构建成功，共享模块与两份 BT XML 均安装 |
| 注册回归 | 17 passed，0 errors / failures / skipped，含 9 项共享启动契约测试及原有 8 项 |
| Python 风格检查 | 本轮三个 launch、公共模块与新增测试共 5 文件，ament_flake8 通过 |
| 真实 BT Navigator | 单目标/多途经点 × root/带前缀 frame × 恢复/持续无路/等待中取消，共 12 场景通过；使用 mock 动作服务器验证调度，不代替物理停车测试 |
| BT 有界失败 | 实际 4 次规划、3 次 1 秒等待，约 3.2 秒后 ABORTED；Spin/BackUp 调用 0；等待中取消后无新规划 |
| 单车真实导航 | 独立无界面 Gazebo + 实际单车入口，(-5.3,3) → (-5.3,1.5) SUCCEEDED；取消返回 CANCELED；最终物理速度分别约 0.0047 / 0.0077 m/s，命令全零 |
| 双车目标被占用 | 将真实手动车模型放在目标 (-5.3,0)，约 3.28 秒后 ABORTED，实际停车，未清除真实障碍强行通行 |
| 双车重试中清障 | 首次规划失败后移走手动车，同一个目标继续执行并 SUCCEEDED |
| 双车一个绕障场景 | 手动车置于 (-5.8,1.5)，哨兵从下方向 (-5.3,3) 导航，SUCCEEDED 并停稳 |
| 最终默认双车入口 | GUI 默认场景创建一个 Gazebo 世界，哨兵 gateway 与手动车 gateway 分别正确加载；共同单车 Include 使用 scoped Group 防止 start_simulator=False 泄漏 |
| 手动车控制与断流 | 从确认出生点向空旷方向持续输入 -0.2 m/s，实测速度约 0.200 m/s，停止输入后降至 0.0255 m/s；这是 ROS 速度输入验证，不是人工键盘验收 |
| 双车取消 | 默认 GUI 双车中 action 返回 CANCELED，最终命令全零，速度约 0.0280 m/s |

上述停车检查都使用 `spin_speed:=0`；以实际速度 < 0.04 m/s 和最终全零速度命令共同判定。控制链收到零命令后仍有平滑和物理减速时间。绕障停车实测经历约 2.28 秒墙钟、0.65 秒仿真时间，不能用固定墙钟等待替代仿真时间与实际速度检查。

未通过与验收边界：

- 手动车居中放在 (-5.3,1.5) 的另一窄道场景，哨兵出现无进展，最终 ABORTED。该案例保留为待优化问题，未修改算法、半径或模型以掩盖失败；不据一个偏置障碍场景成功宣称全部动态避障通过。
- 本轮两次默认 GUI 单车启动未完成：首次 Gazebo server 已初始化并注册 create，但跨进程发现出现 Network is unreachable、客户端创建超时；第二次缺乏 server 初始化证据，卡点未确定。最终单车导航与取消通过的是独立无界面世界；默认 GUI 单车仍待人工验收。
- 早期手动车控制探针向 +x 连续运动后抵达场地边界，末尾速度接近零被误判为不动；安全输出与位置记录证明其已从 x=5.3 移至约 5.713。改为从原出生点向 -x 空旷方向后，运动和断流停车通过；未因此修改模型或控制插件。
- 目标到达不等于所有绕行、动态穿越、隧道或真实定位已验收；本轮没有接入新局部规划器、LIO、裁判或机器人间通信。

当前启动命令不变，可给两入口添加 `use_rviz:=True`。`start_simulator:=False` 仅供明确已有同一仿真世界时使用；正常新终端启动保持默认 True，避免无世界运行。验收脚本只清理自身创建的进程，不终止其他用户实例。

本机证据目录：`/tmp/diting_unify_20261025/`。核心索引为 `validation-summary.json`；源码前后备份 `before/`、`stage/`、`changes.patch`；回归 XML 位于 overlay `build/sentry_sim_bringup/pytest.xml`。物理导航分项证据分别在 `single-simulation-results.json`、`dual-navigation-pass-controls-incomplete/dual-simulation-results.json` 和 `dual-simulation-results.json`；其中保存了失败的整轮测试不能改称整轮通过，以上表格按实际完成的分项引用。失败尝试与真实 BT 两轮结果均单独保留。


## 2026-10-27 启动后自动导航到地图原点

- 双车完整入口默认 `auto_nav_to_origin:=True`，建图模式忽略该任务。
- 新增一次性 `startup_navigation` action 客户端，等待 planner_server、controller_server、bt_navigator、velocity_smoother active，且操作里程计、障碍扫描、map → sentry_robot1/chassis TF 连续有效 3 秒后发送 map (0, 0)。目标朝向取发送时底盘朝向。
- 目标使用 /sentry_robot1/navigate_to_pose，沿用现有 YAstar、控制器和行为树。每次启动只发送一次，失败或取消后不重新发送；可以在 RViz 中取消或手动发新目标。
- 启动就绪超时为 120 秒仿真时间，暂停期间不计时；失败有明确日志。`auto_nav_to_origin:=False` 可关闭。
- 当前手动仿真不自动增加节点；退出旧启动进程后重新启动才加载新入口。

- 自动原点导航复测发现旧接近减速配置会在返回 SUCCEEDED 后滑行，停稳时两次偏离原点约 0.44–0.46 m；第二次同时对照地图估计与 Gazebo 真值，定位误差不足 1 cm，表明主要为停车超调。
- 仅把 LIO / Ground Truth 的 FollowPath.approach_velocity_scaling_dist 从 1.0 调为 2.0 m，保留最低接近速度 0.5 m/s。试验真值 (-0.0731, 0.0569)，距原点 0.0926 m，地图估计距原点 0.1126 m。更低最低速度的临时试验触发低速无法取得进展，未写入默认配置。
- 最终默认配置复验：未传 spin_speed、auto_nav_to_origin 或 params_file；仅 headless 验收使用 start_simulator:=False，由测试脚本在独立 Gazebo partition 创建同一场景，保留默认 25 秒出生延迟。观测脚本没有发送导航目标，唯一自动目标返回 SUCCEEDED；停稳真值 (-0.06714, 0.03195)，距原点 0.07435 m；地图估计距原点 0.08875 m。手动车保持出生位置。
- sentry_sim_bringup 增量构建成功；最终回归 35 passed，0 errors，0 failures；新增/修改的四个 Python 启动与测试文件通过 ament_flake8，文档 Bash 块语法通过。
- 本轮测试域 ROS_DOMAIN_ID=230/231，与用户现有仿真隔离；测试创建的进程已清理。未提交或推送 GitHub。此验收不代表所有路线、动态障碍或隧道均已验证。

- 用户要求清理后，已删除本轮新增的 test_startup_navigation.py、临时 runtime.py 及对应缓存，并移除原有 test_shared_bringup.py 中本轮追加的两个自动目标测试。保留项目原有测试及场景节点清单对新功能的必要适配。35 项通过记录属于清理前的历史验收；自动导航功能节点和已验证参数保持不变，清理后未重新启动仿真或运行测试。
