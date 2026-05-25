# dexe_robot_client 文档

## 导航客户端

`src/dexe_robot_client/navigation_client/` 封装底盘导航、底盘控制、回充、电量、障碍物和 Nav2 参数相关接口。

### ROS/Nav2 接口依赖

`NavigationClient` 当前只需要 Nav2 的接口定义和 ROS2 Python 客户端能力；客户端侧不需要安装完整 `ros-humble-navigation2`。推荐安装最小接口依赖：

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

不建议仅为了运行本客户端安装完整 Nav2 栈：

```bash
sudo apt install ros-humble-navigation2
```

`ros-humble-navigation2` 会拉起完整 Nav2 运行组件，其中 `ros-humble-nav2-smac-planner` 依赖 `ros-humble-ompl`。本仓库已通过 `dependencies/install_planner_dependencies.sh` 使用 wheel 安装 OMPL；混装 apt 版 OMPL 和本仓库规划依赖可能造成依赖冲突。只有在当前机器需要实际启动 Nav2 server 时，才应由对应部署文档单独声明完整 Nav2 栈依赖。

当前导航客户端使用的 ROS/Nav2 类型和接口如下：

| 类型来源 | 类型 / 接口 | 用途 |
| --- | --- | --- |
| `nav2_msgs/action/NavigateToPose` | action `navigate_to_pose` | 发送 Nav2 导航目标并接收结果 |
| `action_msgs/msg/GoalStatus` | `STATUS_SUCCEEDED`、`STATUS_CANCELED` 等 | 解析 Nav2 action 结果状态 |
| `geometry_msgs/msg/PoseStamped` | `/control/nav2_pose`、`/control/ctrl_pose`、`/feedback/chassis_pose` | 表达导航目标、底盘控制目标和底盘反馈位姿 |
| `geometry_msgs/msg/PoseStamped` | `/control_sim/nav2_pose`、`/control_sim/ctrl_pose`、`/feedback_sim/chassis_pose` | 仿真模式下的导航、控制和反馈位姿 |
| `diagnostic_msgs/msg/DiagnosticArray` | `/diagnostics_agg` | 读取电池百分比诊断信息 |
| `std_msgs/msg/Bool` | `obstacle_detector/has_obstacle` | 读取前方障碍物检测状态 |
| `std_srvs/srv/Trigger` | `control/get_control_status` | 阻塞式查询底盘控制状态 |
| `std_srvs/srv/Trigger` | `control/cancel_chassis_control` | 请求取消底盘控制任务 |
| `std_srvs/srv/Trigger` | `auto_charging` | 请求底盘自动回充 |
| `rcl_interfaces/srv/GetParameters` | `/controller_server/get_parameters`、`/local_costmap/local_costmap/get_parameters`、`/global_costmap/global_costmap/get_parameters` | 读取 Nav2 controller 和 costmap 参数 |
| `rcl_interfaces/srv/SetParameters` | `/controller_server/set_parameters`、`/local_costmap/local_costmap/set_parameters`、`/global_costmap/global_costmap/set_parameters` | 设置 Nav2 controller 和 costmap 参数 |
| `rcl_interfaces/msg/ParameterType` | 参数类型枚举 | 校验参数读取结果类型和值 |
| `rclpy` | node、executor、publisher、subscription、service client、action client | Python ROS2 客户端运行基础 |

代码中还存在 `geometry_msgs/msg/Polygon`、`geometry_msgs/msg/Point32` 和 `diagnostic_msgs/msg/DiagnosticStatus` 导入，但当前导航客户端逻辑未实际使用这些类型；它们不构成当前运行接口依赖。

真实模式会初始化 Nav2 action、底盘控制服务、回充服务、电池诊断、障碍物订阅和 Nav2 参数服务。仿真模式当前只初始化位姿发布和底盘反馈订阅。

当前支持调整的 Nav2 参数包括：

- `goal_checker.lon_goal_tolerance`
- `goal_checker.lat_goal_tolerance`
- `goal_checker.yaw_goal_tolerance`
- `footprint`
- `inflation_layer.inflation_radius`
- `FollowPath.vx_max`
- `FollowPath.wz_max`

### 控制状态查询

`NavigationClient.get_control_status()` 不使用定时器后台轮询。上层每次调用时，接口都会请求一次 `control/get_control_status` Trigger 服务，并阻塞等待本次响应。

该接口使用独立临时 node 和独立 `SingleThreadedExecutor` 查询服务，避免主客户端所在 executor 阻塞影响状态查询。底层控制状态服务存在一次性状态语义，底层返回一次 `SUCCEED` 后会切回普通状态；因此不能由后台定时器主动查询服务，否则可能提前消费这次 `SUCCEED`，导致业务调用方读不到真实完成状态。

默认等待超时时间为 1 秒，可通过 `get_control_status(timeout_sec=...)` 调整。

异常处理规则：

- 服务不可用时返回 `"UNKNOWN"`。
- 服务调用超时时返回 `"UNKNOWN"`。
- 服务异常或响应 `success=False` 时返回 `"UNKNOWN"`。

`refresh_control_status()` 作为兼容旧接口保留，内部调用 `get_control_status()` 判断是否成功获取有效状态。
