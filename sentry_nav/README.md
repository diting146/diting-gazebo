# Sentry Navigation Workspace

隧道几何、YAstar软代价ROI和默认三点/yaw统一由 `src/sentry_sim_bringup/config/scene_geometry.yaml` 提供；启动可传 `scene_file:=/绝对路径/scene_geometry.yaml`。`route_points` 省略或空字符串沿用scene默认；显式 `[]`/`null` 仍报错。自定义params/BT几何优先级及兼容回退见[场地几何配置说明](../docs/场地几何配置_2026-11-06.md)。

唯一推荐完整入口为 `sentry_sim_bringup dual_sentry_simulation_launch.py`：默认 LIO + small_gicp 定位、YAstar、固定三点任务及模拟血量返航；自旋默认关闭。构建和依赖安装见[当前构建与依赖说明](../docs/当前构建与依赖说明_2026-11-06.md)。根层限定6包，随后本overlay使用 `colcon build --base-paths src --packages-up-to sentry_sim_bringup --symlink-install`（当前本地闭包17包）。不要在已加载overlay的终端重建根层，也不要在根目录省略 `--base-paths src` 递归构建。

`src/sentry_sim_bringup` 为本项目仿真适配；`src/sentry_tunnel_bt` 为过洞BT；`src/pb2025_sentry_nav` 为上游源码快照及本地改动，保留各包许可证，其中15个包名与根层重复。当前运行以 `source_all.bash` 最后加载的overlay版本为先。生成的 build、install 和 log 不纳入版本控制；历史试验记录不是所有入口的当前功能声明。

推荐先运行 `bash "$WS/tools/build_current.bash" all --check`，需要构建时按层使用 `root`、`overlay` 或 `all`；详细归属和环境隔离规则见[包归属与受控构建](../docs/包归属与受控构建_2026-11-06.md)。手动构建命令仍保留在构建说明中。


自动哨兵默认出生位姿为 **(-4.6, 3.475, 0.3)，yaw=0°**，沿北侧隧道中线朝向洞口；距原(-5.3,3.0)约0.846m。LIO地图初猜同步为(-4.6,3.475,0)，yaw=0，两个参数组仍独立可覆盖。手动车默认(5.3,-3.0,0.3)、yaw=0°。

三点路线与自旋使用同一启动入口。显式设置 `spin_speed:=3.14` 时，默认三点任务先以零自旋接近、对齐和通过隧道；整车出洞且旋转包络安全后再启用该速度，随后导航到最终目标。正常到达只停止平移并保留自旋；取消或失败关闭自旋。`spin_speed:=0.0` 保持全程不自旋。

关闭三点任务（`auto_nav_to_origin:=False enable_route_task:=False`）时，`spin_speed` 保持原来的立即自旋语义。`defer_spin_until_exit` 是双车入口转交给导航组件的内部参数；不要用它替代三点任务开关。单目标 RViz 导航不自动继承三点路线的出洞启旋设置。云台世界朝向补偿由独立节点持续执行，底盘对齐和自旋期间均应保持开启。

固定三点任务默认同时启动模拟血量节点。满血为 100；手动向 `/referee/sentry_robot1/damage` 发布 `std_msgs/msg/Int32` 扣血，**当前血量严格小于 50** 时，由原任务节点停止自旋、取消本节点拥有的在途任务并导航回首次可信定位且静止连续0.5秒时记录的home位姿。50 血不触发；满血时扣 51 后为 49 血，每轮只触发一次。成功返航且空闲、有出生记录、无待处理返航，最新血量不低于100时解除锁存；50或99不解锁。返航途中回血需等成功终态再判，失败、取消、拒绝或未知终态保持锁存。回血不自动开始下一轮，也不覆盖首次出生位姿。

home记录与三点任务启动独立：保留任务节点并设 `route_auto_start:=False` 时，可信定位、TF和odom满足条件且车体静止连续0.5秒后也会记录；无需先启动三点，不依赖扫描或导航action服务器就绪。启用低血返航时，自动三点必须先获得home才出发。home表示首次可信静止的地图位置；如果在定位可信前已人为移动车辆，程序无法追溯更早的物理摆放点。`route_status` 的 `home_state` 为 `pending/captured/invalidated`，`home_blocked_reason` 说明等待或失效原因。

仿真时钟回退会清除home并标为 `invalidated`；输入恢复后不会把洞中当前位置自动记成新出生点。需要重启任务节点，并在确认的新初始位置完成可信静止定位后重新记录。无home时提前触发低血仍保持失败锁存，后来home建立或回血不会自动返航。该改动已通过87项相关Python回归（含22项新增home场景），未新增仿真验收结论。

```bash
source /home/hui/diting-gazebo-feat-sentry-nav-simulation/source_all.bash
ros2 launch sentry_sim_bringup dual_sentry_simulation_launch.py use_rviz:=True
```

另开已加载相同环境的终端，等待三点任务开始或完成后扣血：

```bash
ros2 topic pub --once /referee/sentry_robot1/damage std_msgs/msg/Int32 '{data: 51}'
```

成功返航后先查询实际血量。**当前恰为49**时，下面负伤害会加51回血到100；其他血量应按实际差值计算，`damage`不是设置值：

```bash
ros2 topic echo /referee/sentry_robot1/hp std_msgs/msg/Int32 --once
ros2 topic pub --once /referee/sentry_robot1/damage std_msgs/msg/Int32 '{data: -51}'
ros2 topic echo /referee/sentry_robot1/hp std_msgs/msg/Int32 --once
```

等待 `route_status` 显示 `hp=100`、`state=idle`、`low_hp_latched=false` 后，可手动发RViz目标；该目标完成后再扣51，可请求第二次返航。不直接向HP话题发布100，以免被实际血量源覆盖；外部活跃目标不会被本节点立即接管，边界见下方说明。重新允许返航逻辑已通过审查、24项相关测试及全包230项Python测试，局部构建成功；独立LIO/YAstar仿真完成首次返航→回血100→外部目标绕洞到达→第二次返航，两次均SUCCEEDED并停车，home不变。现有旧实例须关闭重启才能加载新逻辑。

返航失败、被取消或被拒绝后，故障排除且没有未确认动作时，可显式请求重试；该接口保留home和低血锁存，不自动开始三点任务。服务返回成功仅表示重试已受理，最终结果查看 `route_status.return_state/return_result`。无有效home、忙碌、已有返航请求、未知动作结果或已观察到外部活跃目标时拒绝；不会因回血自动解除失败锁定。

```bash
ros2 service call /sentry_robot1/retry_return std_srvs/srv/Trigger '{}'
```

任务节点监听两种导航action的状态，并按action名称和UUID区分自身与外部目标。已观察到外部活跃目标时，`start_route/retry_return` 会明确拒绝；此时低血只保留返航请求，`return_state=blocked_external_navigation`、`return_pending=true`，不取消外部目标，也不与外部控制器争发停车或自旋指令。等所有已观察到的外部UUID收到成功、取消或中止终态后，才停车确认并执行原返航流程。`cancel_route` 可取消本节点的待返航请求，不影响外部目标。外部状态缺项、UNKNOWN、等待超时或时钟回退均不等于外部目标已结束。

这是延后返航保护，**不是低血立即抢占RViz目标**。`external_goal_ids` 记录仍未确认结束的外部目标，`external_observation_state` 为 `not_observed/partial/observed`。状态订阅有发现窗口，不提供Nav2全局原子互斥；初始尚未收到状态不会卡住默认三点任务。未来若要求任意外部目标途中立即返航，需要再统一导航入口，当前未新增action代理。此次任务恢复改动新增26项边界测试，全包Python回归为278项通过；没有新增仿真验收结论。

模拟血量通过 ROS 话题传输，不使用实体串口。`referee_hp.py` 只负责血量（旧返航阈值设置为 0），避免它另发目标与任务节点争抢。建图模式或关闭固定任务时不启动；添加 `enable_sim_referee:=False` 可关闭模拟血量及该自动返航入口。详细边界和状态检查见 [模拟扣血返出生点说明](/home/hui/diting-gazebo-feat-sentry-nav-simulation/docs/模拟扣血返出生点_2026-11-04.md)。


## 2026-11-04 按序收尾

入口横移残余已按原位置精度置零。旧出生点(-5.3,3.0)周围半径25cm九点×三次实测18/27成功（不代表11月5日新默认点周围范围）；偏北三点因静态地图254格与完整起点足迹相交各0/3，其余六点各3/3，LIO均建立。测试只改真实出生XY，初猜保持默认；不是全圆或真机保证。

控制器新增 `controller_collision` 可靠瞬态JSON，首次拒绝同锁复制完整局部地图、足迹/扫掠、命中格、候选指令，解锁后发布；成功返回速度后重置事件。没有更改碰撞通行判据。Gazebo自建场景按server/GUI分别跟踪退出，未显式设置时自动分区隔离；外置模式不结束外部server。

当前操作与验收边界见工程 `docs/待办收尾与当前验收_2026-11-04.md` 和桌面启动说明。根工作区构建从干净ROS环境开始，避免将overlay反向写入根install/setup的父链；overlay再加载根install构建。实际插件路径和运行证据优先于源码推测。
