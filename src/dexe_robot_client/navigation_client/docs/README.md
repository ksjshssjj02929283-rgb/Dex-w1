# navigation_client 接口与问题整理

## 模块定位

`navigation_client` 位于 `src/dexe_robot_client/navigation_client/`，当前实际对外入口是 `ChassisClient`，其继承 `NavigationClient` 并根据 `is_sim` 初始化 ROS publisher、subscription、action client 和 service client。

上层 `W1Manager` 将底盘导航能力包装为 `move_chassis_nav2()`、`move_chassis_ctrl()`、`get_current_chassis_pose()`、`get_current_battery_percentage()`、`get_front_obstacle()`、`get_nav_status()`、`get_control_status()`、`set_auto_charging()`、`cancel_chassis_motion()`、`set_chassis_footprint()`、`set_chassis_nav_tolerance()`、`set_chassis_inflation_radius()` 和 `set_follow_velocity_limits()`。

## 初始化与 ROS 接口

| 模式 | 初始化内容 | 当前用途 |
| --- | --- | --- |
| 真实模式 `is_sim=False` | 订阅 `/feedback/chassis_pose` | 获取底盘当前位姿 |
| 真实模式 `is_sim=False` | 订阅 `/diagnostics_agg` | 读取电池百分比 |
| 真实模式 `is_sim=False` | 订阅 `obstacle_detector/has_obstacle` | 读取前方障碍物状态 |
| 真实模式 `is_sim=False` | action `navigate_to_pose` (`nav2_msgs/action/NavigateToPose`) | 发送 Nav2 目标、取消导航、读取本地导航状态 |
| 真实模式 `is_sim=False` | action `control/precise_control` (`nav_interfaces/action/PreciseControl`) | 发送精控目标、取消精控、读取本地精控状态 |
| 真实模式 `is_sim=False` | service `auto_charging` (`std_srvs/srv/Trigger`) | 请求自动回充 |
| 真实模式 `is_sim=False` | `/controller_server/*_parameters` | 读写 controller 参数 |
| 真实模式 `is_sim=False` | `/local_costmap/local_costmap/*_parameters` | 读写 local costmap 参数 |
| 真实模式 `is_sim=False` | `/global_costmap/global_costmap/*_parameters` | 读写 global costmap 参数 |
| 仿真模式 `is_sim=True` | 订阅 `/feedback_sim/chassis_pose` | 获取仿真底盘当前位姿 |
| 仿真模式 `is_sim=True` | publisher `/control_sim/nav2_pose` | 已创建，但当前 `set_chassis_nav2_position()` 不使用该 publisher |
| 仿真模式 `is_sim=True` | action `control/precise_control` | 发送精控目标 |

真实模式还会创建 `/control/nav2_pose` publisher，但当前导航发送逻辑走 `navigate_to_pose` action，不使用该 publisher。

## 底层接口功能

### 连接与位姿

| 接口 | 输入 | 返回 | 行为 |
| --- | --- | --- | --- |
| `is_connected(timeout_sec=0.1)` | 超时时间，单位秒 | `bool` | 在超时时间内反复 `spin_once()`，只要收到过底盘反馈就返回 `True` |
| `get_chassis_position()` | 无 | `(bool, np.ndarray | None)` | `spin_once()` 后返回 `[x, y, yaw]`，`yaw` 单位为度 |
| `get_battery_percentage()` | 无 | `(bool, float)` | 返回已缓存电量百分比；未收到诊断消息时返回 `(False, -1.0)` |
| `get_front_obstacle()` | 无 | `bool` | 返回已缓存的前方障碍物布尔值 |

### Nav2 导航

| 接口 | 输入 | 返回 | 行为 |
| --- | --- | --- | --- |
| `set_chassis_nav2_position(position)` | `[x, y, yaw]`，`yaw` 单位为度 | `bool` | 构造 `NavigateToPose.Goal`，`frame_id="map"`，异步发送到 `navigate_to_pose` |
| `get_nav_state()` | 无 | `str` | 返回本地导航状态缓存 |
| `cancel_chassis_nav2()` | 无 | `bool` | 通过当前 Nav2 goal handle 发送取消请求 |

导航状态缓存当前包含：`IDLE`、`NAVIGATING`、`SUCCEEDED`、`FAILED`、`CANCELLED`，异常路径会写入 `ERROR`。

### 底盘精控

| 接口 | 输入 | 返回 | 行为 |
| --- | --- | --- | --- |
| `set_chassis_control_position(position)` | `[x, 0, yaw]`，`yaw` 单位为度 | `bool` | 构造 `PreciseControl.Goal`，`target_pose.header.frame_id="base_link"`，异步发送到 `control/precise_control` |
| `get_control_status(consume=True)` | 兼容旧接口的 `consume` 参数 | `str` | 返回本地精控状态缓存，不消费终态 |
| `refresh_control_status()` | 无 | `bool` | 兼容旧接口，当前只判断本地状态是否不是 `UNKNOWN` |
| `cancel_chassis_control()` | 无 | `bool` | 通过当前精控 goal handle 发送取消请求 |

精控目标限制：`x` 和 `yaw` 不能同时非零，否则直接返回 `False`。精控状态来自 action feedback/result，本地用 `_control_task_id` 忽略旧任务回调，避免旧 feedback/result 覆盖新任务状态。

精控状态缓存当前包含：`IDLE`、`CONTROL`、`SUCCEEDED`、`FAILED`、`CANCELLED`，取消请求发出后会进入 `CANCELLING`，并且可能透传 action feedback 的 `current_status`。

### 自动回充

| 接口 | 输入 | 返回 | 行为 |
| --- | --- | --- | --- |
| `set_chassis_auto_charging()` | 无 | `bool` | 等待 `auto_charging` 服务，发送 `Trigger.Request()`，最多等待 2 秒结果 |

### Nav2 参数调整

| 接口 | 参数 | 写入范围 | 行为 |
| --- | --- | --- | --- |
| `set_goal_tolerance(xy_tolerance, yaw_tolerance)` | `xy_tolerance`、`yaw_tolerance` | `/controller_server` | 写入 `goal_checker.lon_goal_tolerance`、`goal_checker.lat_goal_tolerance`、`goal_checker.yaw_goal_tolerance` 后读取校验 |
| `set_footprint(vertices)` | 4 个 `[x, y]` 顶点 | local/global costmap | 写入 `footprint` 字符串后读取校验 |
| `set_inflation_radius(radius)` | 膨胀半径 | local/global costmap | 写入 `inflation_layer.inflation_radius` 后读取校验 |
| `set_follow_velocity_limits(vx_max, wz_max)` | 线速度、角速度上限 | `/controller_server` | 写入 `FollowPath.vx_max`、`FollowPath.wz_max` 后读取校验 |

参数写入统一使用 `_set_param()`：先调用 `SetParameters`，再调用 `GetParameters` 读取校验；支持 double、integer、bool 和 string 类型。

## 上层包装接口

`W1Manager` 对底盘接口做了 `StatusCode` 包装：

| `W1Manager` 接口 | 底层接口 | 说明 |
| --- | --- | --- |
| `move_chassis_nav2(base_position)` | `set_chassis_nav2_position(base_position)` | 导航到 map 坐标目标 |
| `move_chassis_ctrl(base_position)` | `set_chassis_control_position(base_position)` | 精控相对 `base_link` 的前进或旋转 |
| `get_current_chassis_pose()` | `get_chassis_position()` | 获取 `[x, y, yaw]` |
| `get_current_battery_percentage()` | `get_battery_percentage()` | 获取电量百分比 |
| `get_front_obstacle()` | `get_front_obstacle()` | 获取障碍物布尔值 |
| `get_nav_status()` | `get_nav_state()` | 获取导航状态 |
| `get_control_status()` | `get_control_status()` | 获取精控状态 |
| `set_auto_charging()` | `set_chassis_auto_charging()` | 请求自动回充 |
| `cancel_chassis_motion()` | `cancel_chassis_nav2()` + `cancel_chassis_control()` | 尝试同时取消导航和精控，任一取消请求发出成功即返回成功 |
| `set_chassis_footprint(vertices)` | `set_footprint(vertices)` | 设置 costmap footprint |
| `set_chassis_nav_tolerance(xy_tol, yaw_tol)` | `set_goal_tolerance(xy_tol, yaw_tol)` | 设置导航容差 |
| `set_chassis_inflation_radius(radius)` | `set_inflation_radius(radius)` | 设置膨胀半径 |
| `set_follow_velocity_limits(vx_max, wz_max)` | `set_follow_velocity_limits(vx_max, wz_max)` | 设置 FollowPath 速度上限 |

## 当前存在的问题

### 功能语义问题

1. 仿真模式下没有初始化 `_set_nav_client`，但 `set_chassis_nav2_position()` 无空值保护并直接调用 `_set_nav_client.wait_for_server()`；如果外部在仿真模式调用导航接口，会触发 `NoneType` 异常。
2. 仿真模式和真实模式都创建了 nav2 pose publisher，但当前导航方法只使用 `navigate_to_pose` action，不再发布 `/control_sim/nav2_pose` 或 `/control/nav2_pose`；publisher 的用途与实现已经脱节。
3. 仿真模式不订阅 `obstacle_detector/has_obstacle`，而代码注释写的是“仿真和真实模式都订阅”；`get_front_obstacle()` 在仿真模式只会返回默认 `False`。
4. `timeout_sec` 构造参数只用于初始化后的第一次 `spin_once()`，连接超时阈值固定为 `_connection_timeout = 1.0`，调用方传入的 `timeout_sec` 不会改变连接判定窗口。
5. `_check_connection_status()` 使用 ROS clock 秒数，底盘反馈回调用 `time.time()` 写入 `_last_msg_time`；如果启用仿真时间或系统时间与 ROS time 不一致，连接超时判断可能异常。
6. `_check_connection_status()` 超时后只清空 `_cur_pos` 和 `_last_msg_time`，不清空 `_last_chassis_processed_feedback`；`get_chassis_position()` 仍可能返回过期位姿。
7. `get_nav_state()` 和 `get_control_status()` 不主动 `spin_once()`，状态更新依赖外部 executor；如果调用方没有持续 spin，action 状态会停留在旧值。
8. `cancel_chassis_nav2()` 在 goal handle 尚未生成时会循环 `spin_once()` 最多 1 秒；这与注释“非阻塞”不完全一致，也可能与外部 executor 嵌套 spin 冲突。
9. `cancel_chassis_control()` 不等待 goal response；如果刚发送精控目标后立即取消，goal handle 还未缓存时会返回 `False`。
10. `set_chassis_auto_charging()` 和参数读写接口内部使用 `rclpy.spin_until_future_complete()`；如果节点已经在外部 executor 中运行，存在嵌套 spin 失败或阻塞业务线程的风险。
11. `set_chassis_auto_charging()` 的日志文本为 `Cancel control response`，与自动回充语义不一致。
12. `get_battery_percentage()` 只识别固定诊断名 `/Devices/System/PowerBoard/collector_node: PowerBoard` 和固定 key `battery_percentage`；诊断命名变化时会一直返回未收到电量。
13. `set_chassis_nav2_position()` 和 `set_chassis_control_position()` 没有检查 `position` 长度、数值类型或 `NaN/Inf`，非法输入可能在构造消息或旋转时抛异常。
14. `set_footprint()` 强制 4 个顶点；如果后续底盘 footprint 需要非四边形，需要调整接口约束。
15. 导航异常路径会把 `_nav_status` 置为 `ERROR`，但公开状态说明主要列出 `FAILED`，上层调用方可能没有处理 `ERROR`。

### 代码维护问题

1. `navigation_client.py` 存在重复导入 `SetParameters`、`GetParameters`、`ParameterType`、`Parameter`、`Polygon`、`Point32`，其中 `Polygon` 和 `Point32` 当前未使用。
2. `chassis_client.py` 存在重复导入 `GetParameters`、`SetParameters`、`Parameter`，并导入了未使用的 `DiagnosticStatus`。
3. `chassis_client.py` 真实模式下重复创建 controller/local/global 参数 client，两段代码完全相同。
4. `NavigationClient` 继承 `ABC` 和 `Node`，但真正的抽象方法只用于要求子类初始化 publisher/subscriber；如果后续有多个子类，建议把 ROS 接口初始化职责和通用业务方法边界写得更明确。
5. `_spin_once_if_not_executed()` 当前未被调用，保留价值不明确。
6. `_last_control_status`、`_last_control_status_success`、`_last_control_status_time` 当前主要随 `_control_status` 写入，但公开接口没有读取这些字段，可能是旧服务轮询逻辑遗留。

### 文档与接口一致性问题

1. 根目录 `README.md` 和仓库根 `docs/README.md` 已描述底盘精控 action，但 `navigation_client` 包内此前没有独立文档，接口功能和问题需要在本目录持续维护。
2. `ChassisClient` 的真实/仿真能力不对等：真实模式包含导航、精控、回充、电量、障碍物和 Nav2 参数；仿真模式只有位姿反馈、nav publisher 和精控 action。上层文档需要明确哪些接口只适合真实模式。
3. 上层 `W1Manager.get_control_status()` 标注返回 `str`，实际返回 `(StatusCode, str)`，类型标注和行为不一致。

## 建议优先级

| 优先级 | 建议处理项 | 原因 |
| --- | --- | --- |
| P0 | 给仿真模式导航接口增加空值保护，或明确改为 publisher 路径 | 当前会直接异常 |
| P0 | 统一状态更新策略，明确是否由外部 executor 负责 spin | 影响导航/精控状态可见性和取消时序 |
| P1 | 统一时间源并处理过期位姿缓存 | 避免连接状态和位姿读数不一致 |
| P1 | 清理重复 client 初始化和重复/未使用导入 | 降低维护成本，减少误读 |
| P1 | 为输入参数增加长度、类型和有限值校验 | 避免非法输入导致运行时异常 |
| P2 | 梳理仿真模式能力边界，移除未使用 publisher 或恢复对应发布逻辑 | 消除接口语义歧义 |
| P2 | 修正文档和类型标注不一致问题 | 降低上层调用误用概率 |

## 参考代码位置

- `src/dexe_robot_client/navigation_client/navigation_client.py`
- `src/dexe_robot_client/navigation_client/chassis_client.py`
- `src/dexe_robot_client/w1/w1_manager.py`
