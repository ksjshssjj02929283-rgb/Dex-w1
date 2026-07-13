# navigation_client 交接文档

## 文档目的

本文用于交接 `src/dexe_robot_client/navigation_client/` 的当前实现、ROS 接口协议、上层调用方式、已知问题和后续维护重点。内容基于当前代码确认，不代表下游导航服务端的完整设计说明。

当前对外实际入口是 `ChassisClient`。`ChassisClient` 继承 `NavigationClient`，负责按 `is_sim` 初始化底盘位姿订阅、Nav2 action、底盘精控 action、回充 action、电量诊断、障碍物 topic 和 Nav2 参数服务。上层 `W1Manager` 在 `use_nav=True` 时持有真实模式 `ChassisClient(is_sim=False)`，并把底盘能力包装成 `StatusCode` 风格接口；统一停止底盘运动时会同时尝试取消导航、路网导航、精控和回充 action。

## 代码边界

| 文件 | 职责 |
| --- | --- |
| `navigation_client.py` | 通用 ROS2 Node、位姿解析、连接检测、Nav2 单点/路网目标发送与取消、精控 action、回充 action、Nav2 参数读写 |
| `chassis_client.py` | 按真实/仿真模式创建具体 topic、action、service 和 parameter client |
| `__init__.py` | 导出 `ChassisClient` |
| `../w1/w1_manager.py` | 上层业务包装，不在本包内，但调用方主要通过它使用底盘导航 |

维护时优先保持底层 `ChassisClient` 的 ROS 协议清晰，上层 `W1Manager` 只做状态码包装和模式选择，不应把 ROS action/service 细节扩散到业务调用层。

## 模式现状

| 能力 | 真实模式 `is_sim=False` | 仿真模式 `is_sim=True` | 备注 |
| --- | --- | --- | --- |
| 底盘位姿反馈 | `/feedback/chassis_pose` | `/feedback_sim/chassis_pose` | `PoseStamped` |
| Nav2 导航 | `navigate_to_pose` action | 未初始化 action client | 仿真模式调用 `set_chassis_nav2_position()` 当前会有空指针风险 |
| 路网/多路点导航 | `/navigate_through_poses` action | 未初始化 action client | `NavigateThroughPoses`，waypoints yaw 使用弧度 |
| 路径导航 | `/navigate_through_points` action | 未初始化 action client | `nav_interfaces/action/NavigateThroughPoints`，按 `nav_point_names` 串接路径，由 route_executor 解析路点逐段执行 |
| 路网导航点映射 | `/map_asset_manager/get_all_nav_points` service | `/map_asset_manager/get_all_nav_points` service | 返回点位名称到 `(x, y, theta_rad)` 的映射 |
| Nav2 pose publisher | `/control/nav2_pose` | `/control_sim/nav2_pose` | 已创建，但当前导航发送逻辑不使用 publisher |
| 底盘精控 | `control/precise_control` action | `control/precise_control` action | 目标 frame 为 `base_link` |
| 自动回充 | `/auto_charging` action | 未初始化 | `chassis_interfaces/action/AutoCharging` |
| 电量读取 | `/diagnostics_agg` | 未初始化 | 只解析固定诊断名和 key |
| 前方障碍物 | `obstacle_detector/has_obstacle` | 未初始化 | 代码注释说仿真也订阅，但实际没有 |
| Nav2 参数调整 | controller/local/global 参数服务 | 未初始化 | 仅适合真实模式 |

`W1Manager` 当前只创建 `_chassis_client_real = ChassisClient(is_sim=False)`。切到仿真模式时，如果启用了导航，会提示 `Navigation client only support real mode. use_nav set to False.`，因此正常上层路径不会在仿真中使用 `ChassisClient`。

## ROS 接口协议

### 订阅与发布

| 名称 | 类型 | 方向 | 使用位置 | 协议说明 |
| --- | --- | --- | --- | --- |
| `/feedback/chassis_pose` | `geometry_msgs/msg/PoseStamped` | subscribe | 真实模式 | 底盘当前 map 位姿，内部转换为 `[x, y, yaw_deg]` |
| `/feedback_sim/chassis_pose` | `geometry_msgs/msg/PoseStamped` | subscribe | 仿真模式 | 仿真底盘当前位姿，内部转换同真实模式 |
| `/diagnostics_agg` | `diagnostic_msgs/msg/DiagnosticArray` | subscribe | 真实模式 | 查找 status name `/Devices/System/PowerBoard/collector_node: PowerBoard`，再读取 key `battery_percentage` |
| `obstacle_detector/has_obstacle` | `std_msgs/msg/Bool` | subscribe | 真实模式 | `data=True` 表示前方检测到障碍物 |
| `/control/nav2_pose` | `geometry_msgs/msg/PoseStamped` | publish | 真实模式 | 当前只创建 publisher，没有被 `set_chassis_nav2_position()` 使用 |
| `/control_sim/nav2_pose` | `geometry_msgs/msg/PoseStamped` | publish | 仿真模式 | 当前只创建 publisher，没有被 `set_chassis_nav2_position()` 使用 |

位姿输出单位约定：`x/y` 为米，`yaw` 为角度。内部通过 `scipy.spatial.transform.Rotation` 在四元数和欧拉角之间转换。

### Action

| action | 类型 | 使用接口 | 目标 frame | 输入约束 | 状态来源 |
| --- | --- | --- | --- | --- | --- |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `set_chassis_nav2_position()`、`cancel_chassis_nav2()` | `map` | `[x, y, yaw_deg]` | action result，本地缓存 `_nav_status` |
| `/navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | `set_chassis_road_navigation()`、`cancel_road_navigation()` | 默认 `map` | `[[x, y, yaw_rad], ...]`，至少 1 个路点 | action feedback/result，本地缓存 `_road_nav_status` |
| `/navigate_through_points` | `nav_interfaces/action/NavigateThroughPoints` | `set_chassis_path_navigation()`、`cancel_path_navigation()` | 服务端默认 `map` | `nav_point_names: list[str]`（`nav_point_ids`/`pose`/`target_yaw` 可选透传，当前服务端未消费） | action feedback/result，本地缓存 `_path_nav_status` |
| `control/precise_control` | `nav_interfaces/action/PreciseControl` | `set_chassis_control_position()`、`cancel_chassis_control()` | `base_link` | `[x, 0, yaw_deg]`，`x` 和 `yaw` 不能同时非零 | action feedback/result，本地缓存 `_control_status` |
| `/auto_charging` | `chassis_interfaces/action/AutoCharging` | `set_chassis_auto_charging()`、`cancel_auto_charging()` | 无 | `force: bool`，默认 `True` | action feedback/result，本地缓存 `_auto_charging_status` |

Nav2 发送流程：

1. 构造 `NavigateToPose.Goal()`。
2. `pose.header.frame_id = "map"`。
3. 位置写入 `position[0]`、`position[1]`，`z=0.0`。
4. `position[2]` 按角度转四元数。
5. 等待 `navigate_to_pose` action server 最多 1 秒。
6. 异步 `send_goal_async()`，通过 `_nav_task_id` 忽略旧任务回调。

精控发送流程：

1. 检查 `x` 和 `yaw` 不能同时非零。
2. 构造 `PreciseControl.Goal()`。
3. `target_pose.header.frame_id = "base_link"`。
4. `target_pose.pose.position.x = float(position[0])`，`y/z=0.0`。
5. `position[2]` 按角度转四元数。
6. 等待 `control/precise_control` action server 最多 1 秒。
7. 异步 `send_goal_async()`，通过 feedback 和 result 更新本地状态。

路网/多路点导航发送流程：

1. 检查 `_road_nav_action_client` 已初始化，且 `waypoints` 非空。
2. 等待 `/navigate_through_poses` action server，默认最多 10 秒。
3. 将每个 `[x, y, yaw_rad]` 转为 `PoseStamped`，默认 `frame_id="map"`。
4. `yaw_rad` 按弧度直接转四元数，和 Nav2 `NavigateThroughPoses` 原生接口一致。
5. 可选写入 `behavior_tree` 字段。
6. 异步 `send_goal_async()` 并注册 feedback/result callback。
7. feedback 更新为 `EXECUTING`，result 根据 action status 映射为 `SUCCEEDED`、`CANCELED`、`ABORTED` 等状态。

路网导航点查询流程：

1. `ChassisClient` 默认创建 `/map_asset_manager/get_all_nav_points` service client。
2. `get_all_nav_points()` 等待 `nav_interfaces/srv/GetAllNavPoints` 服务，默认最多 10 秒。
3. 服务返回 `success=False`、调用超时或异常时返回空字典。
4. 对 `response.points` 中的 `nav_interfaces/msg/NavPoint` 逐个解析，跳过空名称或非预期类型。
5. 返回 `dict[str, tuple[float, float, float]]`，键为点位名称，值为 `(x, y, theta_rad)`。

路径导航发送流程：

1. 检查 `_path_nav_action_client` 已初始化（仅真实模式）。
2. 等待 `/navigate_through_points` action server，默认最多 10 秒。
3. 构造 `NavigateThroughPoints.Goal()`，写入 `nav_point_names`；`nav_point_ids`、`pose`、`target_yaw` 为可选透传字段，当前服务端未消费。
4. 客户端不校验 `nav_point_names` 数量，也不拦截并发 goal，是否接受完全由服务端裁决；被拒绝的 goal 走 `REJECTED` 状态分支。
5. 每次发送前自增 `_path_nav_task_id`，后续 feedback、goal response、cancel response 和 result callback 只处理当前 task，忽略旧回调。
6. 异步 `send_goal_async()` 并注册带 task id 的 feedback callback。
7. feedback 记录 `current_status`、`current_point_index`、`total_points`，状态置 `EXECUTING`。
8. result 根据 action status 映射状态名，`success` 要求 `status==SUCCEEDED` 且 `result.success` 为真，message 取自 `result.message`。

回充发送流程：

1. 等待 `/auto_charging` action server，默认最多 10 秒。
2. 构造 `AutoCharging.Goal()`，写入 `force` 字段，默认 `True`。
3. 每次发送前自增 `_auto_charging_task_id`，后续 feedback、goal response、cancel response 和 result callback 只处理当前 task，忽略旧回调。
4. 异步 `send_goal_async()` 并注册带 task id 的 feedback callback。
5. goal accepted 后缓存当前 goal handle，并注册带 task id 的 result callback。
6. result callback 根据 action result status 和 `result.success` 更新 `_auto_charging_status`、`_auto_charging_success` 和 `_last_auto_charging_result_message`，并清空当前 goal handle。

回充取消流程：

1. 如果当前 goal handle 尚未生成，但回充状态仍处于 `WAITING_SERVER`、`SENDING`、`ACCEPTED` 或 `EXECUTING`，`cancel_auto_charging()` 会等待最多 1 秒，让当前 goal accepted callback 有机会写入 goal handle。
2. goal handle 可用后调用 `cancel_goal_async()`，并注册带 task id 的 cancel response callback。
3. cancel response 仅代表 action cancel 请求响应；最终是否取消成功仍以后续 result callback 的 `CANCELED` 状态为准。

### Parameter 服务

| 服务 | 类型 | 用途 |
| --- | --- | --- |
| `/map_asset_manager/get_all_nav_points` | `nav_interfaces/srv/GetAllNavPoints` | 获取路网导航点名称到 `(x, y, theta_rad)` 的映射 |
| `/controller_server/set_parameters` | `rcl_interfaces/srv/SetParameters` | 写 controller 参数 |
| `/controller_server/get_parameters` | `rcl_interfaces/srv/GetParameters` | 读取校验 controller 参数 |
| `/local_costmap/local_costmap/set_parameters` | `SetParameters` | 写 local costmap 参数 |
| `/local_costmap/local_costmap/get_parameters` | `GetParameters` | 读取校验 local costmap 参数 |
| `/global_costmap/global_costmap/set_parameters` | `SetParameters` | 写 global costmap 参数 |
| `/global_costmap/global_costmap/get_parameters` | `GetParameters` | 读取校验 global costmap 参数 |

参数写入统一走 `_set_param()`：先 `SetParameters`，等待最多 1 秒；成功后 sleep 0.1 秒，再 `GetParameters` 读取校验。支持 `double`、`integer`、`bool`、`string` 类型。

| 公开接口 | 写入参数 | 说明 |
| --- | --- | --- |
| `set_goal_tolerance(xy_tolerance, yaw_tolerance)` | `goal_checker.lon_goal_tolerance`、`goal_checker.lat_goal_tolerance`、`goal_checker.yaw_goal_tolerance` | `xy_tolerance` 直接写 lon/lat，`yaw_tolerance` 按 Nav2 参数语义使用弧度 |
| `set_footprint(vertices)` | `footprint` | 要求 4 个 `[x, y]` 顶点，写入 local/global costmap |
| `set_inflation_radius(radius)` | `inflation_layer.inflation_radius` | 写入 local/global costmap |
| `set_follow_velocity_limits(vx_max, wz_max)` | `FollowPath.vx_max`、`FollowPath.wz_max` | 写入 controller |

## 公开 Python 接口

### `ChassisClient`

| 接口 | 输入 | 返回 | 说明 |
| --- | --- | --- | --- |
| `is_connected(timeout_sec=0.1)` | 超时时间，秒 | `bool` | 在窗口内 `spin_once()`，收到过底盘位姿反馈即认为连接成功 |
| `get_chassis_position()` | 无 | `(bool, np.ndarray | None)` | 返回 `[x, y, yaw_deg]`；未收到反馈返回 `(False, None)` |
| `get_battery_percentage()` | 无 | `(bool, float)` | 未收到电量时返回 `(False, -1.0)` |
| `get_front_obstacle()` | 无 | `bool` | 返回最近一次障碍物 topic 缓存 |
| `set_chassis_nav2_position(position)` | `[x, y, yaw_deg]` | `bool` | 发送 Nav2 action 目标，非阻塞 |
| `get_nav_state()` | 无 | `str` | 返回本地导航状态缓存 |
| `cancel_chassis_nav2()` | 无 | `bool` | 通过当前 Nav2 goal handle 发送取消请求 |
| `set_chassis_road_navigation(waypoints, frame_id="map", timeout_sec=10.0, behavior_tree="")` | `[[x, y, yaw_rad], ...]` | `bool` | 发送 `/navigate_through_poses` 路网/多路点导航目标，非阻塞 |
| `get_road_navigation_status()` | 无 | `str` | 返回本地路网导航状态缓存 |
| `get_road_navigation_result()` | 无 | `(bool, str)` | 返回最近一次路网导航结果成功标志和消息 |
| `cancel_road_navigation()` | 无 | `bool` | 通过当前路网导航 goal handle 发送取消请求 |
| `set_chassis_path_navigation(nav_point_names, nav_point_ids=None, pose=None, target_yaw=None, timeout_sec=10.0)` | `list[str]` 路点名称，可选 ID/位姿/目标朝向 | `bool` | 发送 `/navigate_through_points` 路径导航目标，非阻塞；不校验点位数量、不拦截并发 |
| `get_path_navigation_status()` | 无 | `str` | 返回本地路径导航状态缓存 |
| `get_path_navigation_result()` | 无 | `(bool, str)` | 返回最近一次路径导航结果成功标志和消息 |
| `cancel_path_navigation()` | 无 | `bool` | 通过当前路径导航 goal handle 发送取消请求；goal handle 尚未生成时最多等待 1 秒 |
| `get_all_nav_points()` | 无 | `dict[str, tuple[float, float, float]]` | 获取所有路网导航点，返回点位名称到 `(x, y, theta_rad)` 的映射 |
| `set_chassis_control_position(position)` | `[x, 0, yaw_deg]` | `bool` | 发送底盘精控 action，非阻塞 |
| `get_control_status(consume=True)` | 兼容参数 | `str` | 返回本地精控状态缓存，当前不消费终态 |
| `refresh_control_status()` | 无 | `bool` | 兼容旧接口，当前只判断状态不是 `UNKNOWN` |
| `cancel_chassis_control()` | 无 | `bool` | 通过当前精控 goal handle 发送取消请求 |
| `set_chassis_auto_charging(force=True, timeout_sec=10.0)` | force、等待 action server 超时 | `bool` | 发送 `/auto_charging` action，非阻塞返回是否成功发出 |
| `get_auto_charging_status()` | 无 | `str` | 返回本地回充 action 状态缓存 |
| `get_auto_charging_result()` | 无 | `(bool, str)` | 返回最近一次回充结果成功标志和消息 |
| `cancel_auto_charging()` | 无 | `bool` | 通过当前回充 goal handle 发送取消请求；goal handle 尚未生成时最多等待 1 秒 |
| `set_goal_tolerance(xy_tolerance, yaw_tolerance)` | float, float | `bool` | 设置并校验 Nav2 到达容差 |
| `set_footprint(vertices)` | 4 点列表 | `bool` | 设置并校验 local/global footprint |
| `set_inflation_radius(radius)` | float | `bool` | 设置并校验 local/global inflation radius |
| `set_follow_velocity_limits(vx_max, wz_max)` | float, float | `bool` | 设置并校验 FollowPath 速度限制 |

### `W1Manager` 包装接口

| `W1Manager` 接口 | 底层接口 | 返回 | 说明 |
| --- | --- | --- | --- |
| `move_chassis_nav2(base_position)` | `set_chassis_nav2_position()` | `StatusCode` | 导航到 map 目标 |
| `move_chassis_road_navigation(waypoints, frame_id="map", timeout_sec=10.0, behavior_tree="")` | `set_chassis_road_navigation()` | `StatusCode` | 发送路网/多路点导航任务，waypoints yaw 使用弧度 |
| `move_chassis_ctrl(base_position)` | `set_chassis_control_position()` | `StatusCode` | 相对 `base_link` 前进或旋转 |
| `get_current_chassis_pose()` | `get_chassis_position()` | `(StatusCode, np.ndarray | None)` | 获取底盘当前位姿 |
| `get_current_battery_percentage()` | `get_battery_percentage()` | `(StatusCode, float)` | 获取电量 |
| `get_front_obstacle()` | `get_front_obstacle()` | `(StatusCode, bool)` | 获取障碍物状态 |
| `get_nav_status()` | `get_nav_state()` | `(StatusCode, str)` | 获取导航状态 |
| `get_road_navigation_status()` | `get_road_navigation_status()` | `(StatusCode, str)` | 获取路网/多路点导航状态 |
| `get_road_navigation_result()` | `get_road_navigation_result()` | `(StatusCode, bool, str)` | 获取最近一次路网/多路点导航结果 |
| `get_all_nav_points()` | `get_all_nav_points()` | `(StatusCode, dict[str, tuple[float, float, float]])` | 获取所有路网导航点名称到 `(x, y, theta_rad)` 的映射 |
| `get_control_status()` | `get_control_status()` | `(StatusCode, str)` | 当前函数标注为 `str`，实际返回元组 |
| `set_auto_charging()` | `set_chassis_auto_charging()` | `StatusCode` | 请求自动回充 |
| `cancel_auto_charging()` | `cancel_auto_charging()` | `StatusCode` | 取消当前回充 action |
| `cancel_chassis_motion()` | `cancel_chassis_nav2()` + `cancel_road_navigation()` + `cancel_chassis_control()` + `cancel_auto_charging()` | `StatusCode` | 任一取消请求发出成功即返回成功 |
| `set_chassis_footprint(vertices)` | `set_footprint()` | `StatusCode` | 设置 costmap footprint |
| `set_chassis_nav_tolerance(xy_tol, yaw_tol)` | `set_goal_tolerance()` | `StatusCode` | 设置导航容差 |
| `set_chassis_inflation_radius(radius)` | `set_inflation_radius()` | `StatusCode` | 设置膨胀半径 |
| `set_follow_velocity_limits(vx_max, wz_max)` | `set_follow_velocity_limits()` | `StatusCode` | 设置 FollowPath 限速 |

## 状态语义

### 导航状态

| 状态 | 含义 | 来源 |
| --- | --- | --- |
| `IDLE` | 初始空闲 | 初始化 |
| `NAVIGATING` | 已发送导航目标，等待结果 | `set_chassis_nav2_position()` |
| `SUCCEEDED` | Nav2 result 成功 | action result |
| `FAILED` | goal 被拒绝或 result 非成功/非取消 | action response/result |
| `CANCELLED` | 取消响应或 action result 为取消 | cancel/result callback |
| `ERROR` | result 获取异常 | `_on_nav_result()` 异常路径 |

### 精控状态

| 状态 | 含义 | 来源 |
| --- | --- | --- |
| `IDLE` | 初始空闲 | 初始化 |
| `CONTROL` | 已发送精控目标 | `set_chassis_control_position()` |
| `CANCELLING` | 取消请求已被 action 接收 | cancel callback |
| `SUCCEEDED` | 精控 result 成功且 `result.success=True` | action result |
| `FAILED` | goal 拒绝、发送异常、result 失败或 result 异常 | action response/result |
| `CANCELLED` | action result 为取消 | action result |
| 其他字符串 | `PreciseControl` feedback 的 `current_status` | action feedback |

### 路网导航状态

| 状态 | 含义 | 来源 |
| --- | --- | --- |
| `IDLE` | 初始空闲 | 初始化 |
| `WAITING_SERVER` | 正在等待 `/navigate_through_poses` action server | `set_chassis_road_navigation()` |
| `SENDING` | 已开始发送路网 goal | `set_chassis_road_navigation()` |
| `ACCEPTED` | 路网 goal 被 action server 接受 | goal response |
| `EXECUTING` | 收到路网导航 feedback | action feedback |
| `CANCELING` | 已发送取消请求 | `cancel_road_navigation()` |
| `SUCCEEDED` | action result 成功 | action result |
| `CANCELED` | action result 为取消 | action result |
| `ABORTED` | action result 为中止 | action result |
| `REJECTED` | goal 被拒绝 | goal response |
| `FAILED_TO_SEND` | action client 未初始化、server 不可用、路点非法或发送异常 | 发送路径 |
| `UNKNOWN` | action result 异常或未知状态码 | result callback |

### 回充状态

| 状态 | 含义 | 来源 |
| --- | --- | --- |
| `IDLE` | 初始空闲 | 初始化 |
| `WAITING_SERVER` | 正在等待 `/auto_charging` action server | `set_chassis_auto_charging()` |
| `SENDING` | 已开始发送回充 goal | `set_chassis_auto_charging()` |
| `ACCEPTED` | 回充 goal 被 action server 接受 | goal response |
| `EXECUTING` | 收到回充 action feedback | action feedback |
| `CANCELING` | 已发送取消请求 | `cancel_auto_charging()` |
| `SUCCEEDED` | action result 成功且 `result.success=True` | action result |
| `CANCELED` | action result 为取消 | action result |
| `ABORTED` | action result 为中止 | action result |
| `REJECTED` | goal 被拒绝 | goal response |
| `FAILED_TO_SEND` | action client 未初始化、server 不可用或发送异常 | 发送路径 |
| `UNKNOWN` | action result 异常或未知状态码 | result callback |

注意：`get_nav_state()` 和 `get_control_status()` 只是读取本地缓存，不主动 `spin_once()`。在 `W1Manager` 正常路径中，节点已经加入 `MultiThreadedExecutor` 后台 spin；如果直接单独使用 `ChassisClient`，调用方需要自己 spin，或调用会触发 spin 的接口。

## 安装与测试

在 `dexe_robot_client` 仓库根目录下可以通过 Python build 生成安装包：

```bash
python3 -m build
```

构建完成后，`dexe_robot_client/dist/` 下会生成 `dexe_robot_client_*.whl` 文件。进入该目录或直接指定 wheel 路径后，可以用 `pip install` 安装进行测试：

```bash
pip install *.whl
```

如果在仓库根目录直接安装，也可以使用：

```bash
pip install dist/*.whl
```

## 典型调用方式

```python
from dexe_robot_client.common import StatusCode, W1ArmType, W1Version
from dexe_robot_client.end_effector_client import EndEffectorType
from dexe_robot_client.w1 import W1Manager

w1 = W1Manager(
    is_sim=False,
    arm_type=W1ArmType.ANTHROPOMORPHIC,
    w1_version=W1Version.V2,
    end_effector_type=EndEffectorType.BRAINCO_HAND,
    use_nav=True,
)

if w1.connect() == StatusCode.SUCCESS:
    w1.move_chassis_nav2([1.0, 0.5, 90.0])
    status_code, nav_status = w1.get_nav_status()

    status_code, nav_points = w1.get_all_nav_points()
    if status_code == StatusCode.SUCCESS:
        waypoints = [
            nav_points["point_a"],
            nav_points["point_b"],
        ]
        w1.move_chassis_road_navigation(waypoints)

    w1.move_chassis_road_navigation([
        [1.0, 0.5, 0.0],
        [1.5, 0.5, 1.57],
    ])
    status_code, road_status = w1.get_road_navigation_status()
```

直接使用 `ChassisClient` 时需要先确保 `rclpy.init()` 和 executor/spin 逻辑由调用方负责，否则 action feedback/result 和订阅缓存可能不会及时更新。

## 当前已确认问题

### P0

1. 仿真模式没有初始化 `_set_nav_client`，但 `set_chassis_nav2_position()` 直接访问 `_set_nav_client.wait_for_server()`；直接使用仿真 `ChassisClient` 调导航会触发异常。
2. 状态更新依赖外部 spin，但公开状态读取接口不触发 spin；单独使用 `ChassisClient` 时容易误以为 action 状态没有变化。

### P1

1. `_check_connection_status()` 使用 ROS clock，位姿回调用 `time.time()` 写 `_last_msg_time`；仿真时间或系统时间不一致时连接判断可能异常。
2. 连接超时后只清空 `_cur_pos` 和 `_last_msg_time`，没有清空 `_last_chassis_processed_feedback`；`get_chassis_position()` 可能继续返回过期位姿。
3. `timeout_sec` 构造参数只影响初始化后第一次 `spin_once()`，连接超时阈值固定为 `_connection_timeout = 1.0`。
4. `cancel_chassis_nav2()` 在 goal handle 尚未生成时会内部循环 spin 最多 1 秒，与“非阻塞”描述不完全一致。
5. `cancel_chassis_control()` 不等待 goal response；刚发送精控目标后立即取消时，如果 goal handle 未缓存，会返回 `False`。
6. `cancel_auto_charging()` 在 goal handle 尚未生成时会内部等待最多 1 秒，与“非阻塞”描述不完全一致。
7. 参数读写内部调用 `rclpy.spin_until_future_complete()`；如果节点已在外部 executor 中运行，存在嵌套 spin 或阻塞业务线程风险。
8. `set_chassis_nav2_position()`、`set_chassis_control_position()` 没有输入长度、数值类型、`NaN/Inf` 校验。

### P2

1. 真实/仿真模式都创建了 nav2 pose publisher，但当前导航逻辑只走 action，publisher 用途已经不清晰。
2. 代码注释称仿真和真实模式都订阅障碍物 topic，但实际只在真实模式订阅。
3. 电量解析依赖固定诊断名和固定 key，诊断结构变化后会一直返回未收到电量。
4. `navigation_client.py` 和 `chassis_client.py` 存在重复导入、未使用导入和重复创建参数 client。
5. `W1Manager.get_control_status()` 类型标注与实际返回 `(StatusCode, str)` 不一致。
6. `set_footprint()` 强制 4 个顶点；如果后续 Nav2 footprint 需要多边形，需要调整接口约束。
7. 导航异常状态存在 `ERROR`，但上层状态说明主要列出 `FAILED`，调用方可能未处理。

## 后续维护建议

| 优先级 | 建议 | 理由 |
| --- | --- | --- |
| P0 | 明确仿真导航路径：要么补齐 action client，要么恢复 publisher 发送逻辑并给 action 路径加空值保护 | 避免直接异常 |
| P0 | 明确 `ChassisClient` 的 executor 使用约束，必要时把状态读取改成可选 spin 或文档化后台 executor 要求 | 避免状态不可见 |
| P1 | 统一时间源，超时时同时清空位姿反馈缓存 | 避免返回过期底盘位姿 |
| P1 | 增加输入校验：长度、有限值、类型、精控互斥约束 | 避免非法输入导致运行时异常 |
| P1 | 清理重复 client 初始化、重复导入和无用字段 | 降低维护成本 |
| P2 | 梳理真实/仿真能力差异，在上层显式拒绝不支持能力 | 降低误用概率 |
| P2 | 把电量诊断名/key 做成可配置项 | 适配不同设备诊断命名 |

## 参考位置

- `src/dexe_robot_client/navigation_client/navigation_client.py`
- `src/dexe_robot_client/navigation_client/chassis_client.py`
- `src/dexe_robot_client/w1/w1_manager.py`
- `README.md` 的 `Navigation Client` 章节
