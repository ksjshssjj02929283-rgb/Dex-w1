# dexe_robot_client 文档

## 导航客户端

`src/dexe_robot_client/navigation_client/` 封装底盘导航、底盘控制、回充、电量、障碍物和 Nav2 参数相关接口。

### ROS/Nav2 接口依赖

`NavigationClient` 当前只需要 Nav2、自定义导航 action 的接口定义和 ROS2 Python 客户端能力；客户端侧不需要安装完整 `ros-humble-navigation2`。推荐安装最小 apt 接口依赖：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-rclpy \  可能会升级，应该不需要升级
  ros-humble-nav2-msgs \
  ros-humble-geometry-msgs \
  ros-humble-std-srvs \
  ros-humble-rcl-interfaces \
  ros-humble-diagnostic-msgs \
  ros-humble-action-msgs \
  ros-humble-std-msgs
```

底盘精控 action 来自工作空间内的 `nav_interfaces/action/PreciseControl.action`，使用前需要先构建并 source `/home/lovebc/dexforce_ws/dexe_interfaces/nav_interfaces` 所在工作空间，确保 Python 能导入 `nav_interfaces.action.PreciseControl`。

不建议仅为了运行本客户端安装完整 Nav2 栈：

```bash
sudo apt install ros-humble-navigation2
```

`ros-humble-navigation2` 会拉起完整 Nav2 运行组件，其中 `ros-humble-nav2-smac-planner` 依赖 `ros-humble-ompl`。本仓库已通过 `dependencies/install_planner_dependencies.sh` 使用 wheel 安装 OMPL；混装 apt 版 OMPL 和本仓库规划依赖可能造成依赖冲突。只有在当前机器需要实际启动 Nav2 server 时，才应由对应部署文档单独声明完整 Nav2 栈依赖。

当前导航客户端使用的 ROS/Nav2 类型和接口如下：

| 类型来源 | 类型 / 接口 | 用途 |
| --- | --- | --- |
| `nav2_msgs/action/NavigateToPose` | action `navigate_to_pose` | 发送 Nav2 导航目标并接收结果 |
| `nav_interfaces/action/PreciseControl` | action `control/precise_control` | 发送底盘精控目标、接收 feedback/result、取消精控任务 |
| `action_msgs/msg/GoalStatus` | `STATUS_SUCCEEDED`、`STATUS_CANCELED` 等 | 解析 Nav2 action 结果状态 |
| `geometry_msgs/msg/PoseStamped` | `/control/nav2_pose`、`/feedback/chassis_pose` | 表达导航目标和底盘反馈位姿 |
| `geometry_msgs/msg/PoseStamped` | `/control_sim/nav2_pose`、`/feedback_sim/chassis_pose` | 仿真模式下的导航目标和反馈位姿 |
| `geometry_msgs/msg/PoseStamped` | `PreciseControl.Goal.target_pose` | 表达底盘精控目标；客户端保持使用 `base_link` 坐标系 |
| `diagnostic_msgs/msg/DiagnosticArray` | `/diagnostics_agg` | 读取电池百分比诊断信息 |
| `std_msgs/msg/Bool` | `obstacle_detector/has_obstacle` | 读取前方障碍物检测状态 |
| `std_srvs/srv/Trigger` | `auto_charging` | 请求底盘自动回充 |
| `rcl_interfaces/srv/GetParameters` | `/controller_server/get_parameters`、`/local_costmap/local_costmap/get_parameters`、`/global_costmap/global_costmap/get_parameters` | 读取 Nav2 controller 和 costmap 参数 |
| `rcl_interfaces/srv/SetParameters` | `/controller_server/set_parameters`、`/local_costmap/local_costmap/set_parameters`、`/global_costmap/global_costmap/set_parameters` | 设置 Nav2 controller 和 costmap 参数 |
| `rcl_interfaces/msg/ParameterType` | 参数类型枚举 | 校验参数读取结果类型和值 |
| `rclpy` | node、executor、publisher、subscription、service client、action client | Python ROS2 客户端运行基础 |

代码中还存在 `geometry_msgs/msg/Polygon`、`geometry_msgs/msg/Point32` 和 `diagnostic_msgs/msg/DiagnosticStatus` 导入，但当前导航客户端逻辑未实际使用这些类型；它们不构成当前运行接口依赖。

真实模式会初始化 Nav2 action、底盘精控 action、回充服务、电池诊断、障碍物订阅和 Nav2 参数服务。仿真模式当前初始化底盘反馈订阅、导航目标发布器和底盘精控 action client。

当前支持调整的 Nav2 参数包括：

- `goal_checker.lon_goal_tolerance`
- `goal_checker.lat_goal_tolerance`
- `goal_checker.yaw_goal_tolerance`
- `footprint`
- `inflation_layer.inflation_radius`
- `FollowPath.vx_max`
- `FollowPath.wz_max`

### 底盘精控 action

`NavigationClient.set_chassis_control_position(position)` 通过 `control/precise_control` action 发送底盘精控目标，不再发布 `/control/ctrl_pose`，也不再调用 `control/get_control_status` 和 `control/cancel_chassis_control` 服务。

目标语义保持不变：

- `position` 仍为 `[x, 0, yaw]`，`yaw` 单位为度。
- `target_pose.header.frame_id` 仍为 `base_link`。
- `x` 和 `yaw` 不能同时非零，否则接口返回 `False`。
- 发送行为与导航一致：客户端不因本地旧 goal handle 或旧状态拦截新目标；每次发送都会递增任务 ID，旧 action 回调到达时会被忽略。若底层仍有精控 action 活跃，是否接受新目标由 `control/precise_control` action server 决定。

状态和取消语义：

- `get_control_status(consume=True)` 读取本地 action 状态缓存，不再主动请求服务，也不在接口内部调用 `rclpy.spin_once()`；状态更新依赖外部 executor 处理 action feedback/result。`consume` 参数仅为兼容旧调用保留，当前不会把终态重置为 `"IDLE"`，语义与 `get_nav_state()` 保持一致。
- 发送目标后本地状态先置为 `"CONTROL"`；收到 action feedback 时使用 `Feedback.current_status` 更新状态。
- action result 成功时状态为 `"SUCCEEDED"`，取消时为 `"CANCELLED"`，失败、拒绝或异常时为 `"FAILED"`。
- 收到 `"SUCCEEDED"`、`"FAILED"`、`"CANCELLED"` 终态 feedback/result 后会释放本地 goal handle，允许后续继续发送新的精控 action。
- `cancel_chassis_control()` 通过当前 action goal handle 调用 `cancel_goal_async()`，不再调用 Trigger 取消服务；如果 goal response 尚未被外部 executor 处理导致本地还没有 goal handle，会返回 `False`，避免在业务线程里嵌套 spin。

实现上使用 `_control_lock` 保护 goal handle、任务 ID、结果和状态缓存。新任务会递增 `_control_task_id`，旧 action 回调到达时会被忽略，避免旧 feedback/result 覆盖新任务状态；发送入口不做本地活跃任务拦截，保持与导航 action 相同的异步发送模型。
