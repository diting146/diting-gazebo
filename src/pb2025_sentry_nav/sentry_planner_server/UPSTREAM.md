# 上游来源与本地修改

- 官方仓库：https://github.com/ros-navigation/navigation2
- tag：`1.1.20`
- commit：`a097086719c88f781aa59788eca29ac6ca5e56db`
- 下载：https://codeload.github.com/ros-navigation/navigation2/tar.gz/refs/tags/1.1.20
- 归档 SHA256：`c965b7a36ef48cd7f35f01c1f98883741693d195dae582232d6d0444d2eedab6`
- 原始路径：`nav2_planner`；归档与未改原件保存在上一级审计目录。
- 原始许可证：Apache-2.0。源文件版权和许可证声明保留，`LICENSE` 使用上游同版文件。

保留原有 ROS 节点名 `planner_server`、两种规划 action、参数结构、costmap、生命周期和插件加载。
本地包与 C++ namespace 改为 `sentry_planner_server`，库名改为 `sentry_planner_server_core`，避免覆盖系统官方库。

本地实质修改：

1. 使用小型 `GoalAwareSimpleActionServer` 扩展，在锁内捕获执行中的真实 goal handle；取消检查绑定该 handle，而不是 action 的全局状态或 pending goal。
2. 两种 action 的规划锁等待、地图等待、分段前后、最终成功前都可取消。取消只终止对应 current handle，不清空 pending goal。
3. `getPlan` 对实现了可选 `tdt_yastar_planner::CancelablePlanner` 接口的插件传入 checker；标准 Humble 插件保留原接口，至少在返回后拒绝已取消结果。
4. 空 ThroughPoses 请求明确终止并返回，错误日志不再索引空 goals。
5. 规划结果在成功终态确认后才发布到可视化话题。
6. `SerializedCancelServer` 在真实 rclcpp action 的 `call_handle_cancel_callback` 外层持有同一互斥量，覆盖 ACCEPT 决定直到 `_cancel_goal()` 转换，关闭取消与成功提交的竞态。注册/删除 waitable 方式参考本机 Humble `rclcpp_action/create_server.hpp`，保留其 Open Source Robotics Foundation 版权。此 API 在升级 rclcpp_action 时须重新核对。

未改 `/opt`、`nav2_core` ABI 和系统 SimpleActionServer。代价是需要维护一份明确版本的本地 planner server 源码；升级 Nav2 时必须重新对照上游差异。

2026-11-01 已在本工程构建并运行：18 项受控 action/实际 planner callback/分派测试通过，实际 LIO 三点导航与行进转向取消停车通过。完整证据和边界见工程 docs/轨迹优化与附近定位验收_2026-11-01.md；审计目录的候选快照保留其当时未执行说明。
