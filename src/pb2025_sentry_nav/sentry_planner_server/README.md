# 目标级取消规划服务

这是 Nav2 Humble 1.1.20 的本地小范围扩展，目标是把真实 action goal 的取消检查传入 YAstar 搜索和轨迹优化。已于 2026-11-01 在本工程构建、安装并接入 LIO/GT 仿真。18 项真实 action/规划服务测试通过，整合版三点路线和行进转向取消停车通过。

## 接入

- 新包放入根工作区 `src/pb2025_sentry_nav/sentry_planner_server`。
- 同级 YAstar 包安装 `include/tdt_yastar_planner/cancelable_planner.hpp`，导出其 include 目录。
- YAstar 插件多继承 `nav2_core::GlobalPlanner` 和 `tdt_yastar_planner::CancelablePlanner`。
- 普通启动：package 从 `nav2_planner` 改为 `sentry_planner_server`；executable 仍为 `planner_server`。
- 组合启动：package 同样调整，组件类型改为 `sentry_planner_server::PlannerServer`。
- 节点名、namespace、参数文件和 lifecycle manager 的节点列表保持原约定。

依赖沿用 nav2_planner，新增 `tdt_yastar_planner` 仅用于可选接口；YAstar 不反向依赖 server，没有循环依赖。无需升级或覆盖系统 Nav2 包。

## 取消边界

取消是协作式检查，不是强行打断线程。每次 checker 必须对应本次调用的 current goal handle。新 pending goal 的取消不能影响 current goal；取消 current 也不能取消 pending。

原 Humble `SimpleActionServer::is_cancel_requested()` 在存在 pending 时可能只读取 pending，不在本包执行路径使用。原 helper 的 `terminate_all()` 也不用于普通取消或规划错误。

成功提交和完整的“接受取消请求→真实 CANCELING 转换”使用同一互斥量串行。仅在用户 cancel callback 中加锁不足以保证这一点；本包通过覆写受保护的 rclcpp action cancel 分发入口来覆盖整个状态转换。没有 accepted-but-not-yet-CANCELING 的终止等待，也不会用 ABORTED 冒充 CANCELED。

标准未实现扩展的插件仍可使用，但其内部计算不可即时取消；取消后拒绝结果。YAstar 与优化器必须在各自可中断点调用 checker，或抛出 `PlannerCancelled`。

costmap 等待和跨 action 共用规划锁等待使用 10 ms 检查周期。TF 变换、单次代价地图快照、单次碰撞操作、底层求解库调用可能仍有不可中断区段，实际响应时间必须测量，不能承诺硬实时。

## 已执行的测试（2026-11-01，18 项通过）

- `test_goal_scoped_cancel`：真实 rclcpp action 请求，覆盖两种 action 的 current 取消时保留 pending、pending 取消时保留 current、成功边界取消，再补 ACCEPT 到真实 CANCELING 转换间隙的确定性竞态检查，共 8 项。
- `test_planner_callback_cancel`：实际执行 PlannerServer 两种 compute callback + 慢插件，在真实空 costmap 上覆盖搜索取消、地图等待取消、等待另一种 action 的规划锁时取消，共 5 项；无需 Gazebo。
- `test_planner_cancel_dispatch`：验证 checker 确实进入扩展插件、空 planner_id 仍使用扩展、标准插件返回后的取消拒绝、预先取消不调用插件，以及旧插件兼容，共 5 项。

已在隔离 ROS_DOMAIN_ID 下构建、测试本包。以下为验收边界清单；受控回调测试使用慢插件覆盖搜索与等待，不冒称真实 YAstar 在 Gazebo 中长搜索的全部组合都已实测：

1. 单点与 ThroughPoses 各发送耗时足够的规划请求，取消后收到该 goal UUID 的 `CANCELED`，不返回可执行路径；记录取消延迟。
2. current 搜索中另发 pending，再取消 current，pending 必须仍能执行并成功。反向只取消 pending，current 必须不受影响。
3. 暂停 costmap 输入令其等待后取消，目标须结束，无永久等待。
4. 同时占用一个 action 的搜索，另一个 action 等待规划锁后取消，等待者须及时结束，不等待前一个规划完成。
5. 使用真实 YAstar 与优化器，分别验证搜索取消、优化取消和完整三点路线成功；对照日志区分取消、无路径、时间预算和普通错误。
6. 标准 Theta* 接口仍可加载，但不以它内部即时取消作为该扩展的验收项。

如果一次实际请求在发出取消前已经成功，测试应标记为竞态未覆盖并重试受控耗时场景，不能把成功完成当取消成功。

完整构建、运行证据、复测及未闭环问题见工程 docs/轨迹优化与附近定位验收_2026-11-01.md。标准非扩展插件内部及单次 Eigen 求解不能抢占；本包不是硬实时取消服务。
