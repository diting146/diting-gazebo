# 哨兵操控上位机

独立于 gazebo/ 和 sentry_nav/ 的本地浏览器应用。所有上位机代码、网页、启动入口与联调记录集中在本目录，不需要作为 ROS 包安装，也不需要单独 colcon 构建。

## 目录

```text
operator_console/
├── start.bash              # 启动入口
├── operator_console.py     # ROS 2 / HTTP 后端
├── web/
│   ├── index.html          # 页面结构
│   ├── style.css           # 白底简约主题
│   └── app.js              # 地图与控制交互
├── validation/             # 只读记录工具及运动联调证据
└── README.md
```

## 启动

先启动现有仿真导航，再从仓库根目录运行（也可使用脚本的绝对路径）：

```bash
./operator_console/start.bash
# 自定义端口：
./operator_console/start.bash --port 8990
```

浏览器访问 **http://127.0.0.1:8989**。启动脚本加载相邻工作区的 `source_all.bash`，再直接运行本目录的 Python 后端。ROS 2 Humble、rclpy、tf2_ros、nav2_msgs、nav_msgs、geometry_msgs、example_interfaces 由现有 ROS 环境提供；HTTP 服务使用 Python 标准库，网页不使用外部 CDN 或 npm 构建。

加载 ROS 环境后，也可直接运行：

```bash
source ./source_all.bash
python3 ./operator_console/operator_console.py --port 8989
```

## 使用

- 地图单击空地选点，或输入 map 系 X、Y；点击“加入任务列表”，最多 20 个点按顺序执行。
- 勾选“任务中开启自旋”并设置速度、方向；默认不自旋，任务结束自动停转。运行中可独立开启/停止自旋。
- “取消任务并停转”取消本台下发的任务并停止自旋，不是硬件急停，也不取消其他客户端的导航任务。
- 位置、规划路径、任务进度和日志均来自实时 ROS 数据。静态地图选点检查不代替 Nav2 的实时障碍与可达性判断。
- 当前导航配置不执行到点转向；目标朝向仍随消息下发。
- 打开、刷新页面不发送运动指令。刷新或关闭浏览器后，服务端任务继续执行；正常 Ctrl+C 关闭后端时，请求取消本台活动任务并停止本台控制的自旋。
- 后端仅监听本机 127.0.0.1。导航离线或仿真暂停时，界面可打开，但不能下发新目标或开启自旋。

## ROS 接口

导航任务使用 `NavigateToPose` action，自旋指令发布到 `/cmd_spin`。界面读取 `/map`、`/plan`、`/odometry` 和 `map → chassis` TF，不读取真值。仅 `validation/record_motion.py` 为误差验证读取 `/reference/*`，不参与控制。

## 联调记录

见 [运动联调记录](validation/operator-console-acceptance.md)，包括两点顺序导航、途中取消、自旋导航及自旋启停。


## UL 开局业务

安装并加载 `sentry_bot` 后，可在界面载入配置中的入口、出口和中心三个点，点击列表“编辑”调整坐标并保存，再启动开局任务。路线在启动时冻结，未经测量的隧道线会由后台拒绝。界面显示入口导航、入口对正、穿越、出洞停车和中心导航阶段。停止按钮调用业务仲裁停车并取消任务；业务模式关闭自旋、禁止直接下发普通导航任务。未启动业务节点时，原普通导航模式仍可用。

默认路线由 `sentry_business_bringup/config/opening_2027rmul.yaml` 提供，经 `/api/opening-route` 读取；`POST /api/opening` 使用现有三点 `points` 格式。业务状态合并在 `/api/state`。保持现有雷达和车体模型不变，已连续通过 10 次完整开局和隧道正反方向各 10 次；取消与实体障碍物停车测试通过。证据见业务工作空间文档。
