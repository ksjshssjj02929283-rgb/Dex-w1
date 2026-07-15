# navigation_client

底盘 / 导航能力的客户端封装，供 `w1.W1Manager` 及以上业务层调用。本文档面向本模块的维护者与直接使用者。

> 分支：`glf/modify_navigation`（基线 `dev_w1`）。本文档描述**重构 v2** 之后的结构（原单文件 `navigation_client.py` 已拆分并删除）。

## 目录结构

```text
navigation_client/
├── __init__.py                  # 导出 ChassisClient
├── chassis_client.py             # 对外门面：全部公开 API，组合 features，按 is_sim 选配置
├── interfaces.py                 # 接口配置表：REAL / SIM 两份 topic/action/service 名称
├── common/                       # 本模块内部通用机制（与具体功能无关）
│   ├── states.py                 #   TaskState 枚举 + 活跃/终态集合
│   ├── action_task_client.py     #   ⭐ 通用 action 二次封装（发送/取消/状态机/watchdog）
│   ├── service_caller.py         #   service 同步调用封装（守卫 + 执行器感知等待）
│   ├── runtime.py                #   spin_once / wait_for_future / standalone 后台执行器
│   └── validation.py             #   位姿输入校验
└── features/                     # 一功能一文件
    ├── chassis_state.py          #   位姿 / 电量 / 障碍物订阅缓存 + 连接检测
    ├── nav2_navigation.py        #   Nav2 单点导航（ActionTaskClient(NavigateToPose)）
    ├── road_navigation.py        #   路网 / 多路点导航（ActionTaskClient(NavigateThroughPoses)）
    ├── nav_points.py             #   导航点查询（GetAllNavPoints service）
    ├── precise_control.py        #   底盘精控（ActionTaskClient(PreciseControl)）
    ├── auto_charging.py          #   自动回充（ActionTaskClient(AutoCharging)）
    └── nav2_params.py            #   Nav2 参数读写（写后读校验）
```

依赖方向严格单向：`chassis_client → features → common`；`features` 之间互不依赖。

> ⚠️ **命名提醒**：本文的 `common/` 是 `navigation_client` **模块内部**的通用机制目录，与 `dexe_robot_client` **顶层**的 `common/`（logger/StatusCode/config，全部 client 共享）是两个不同的包（`dexe_robot_client.navigation_client.common` vs `dexe_robot_client.common`），只是恰好同名。本模块**不 import 顶层 `common`**（旧版曾因顶层 `common.__init__` 反向拉起 `end_effector_client` 导致无关导入报错，拆分后已根治），import 时注意区分 `from .common import ...`（模块内）与 `from ...common import ...`（顶层，本模块未使用）。

## 设计原则

1. **组合替代继承**：`ChassisClient` 是唯一的 `rclpy.node.Node`，持有 7 个 feature 实例；不再有"抽象基类塞满逻辑、子类只拼接口"的倒挂结构。
2. **接口配置化**（`interfaces.py`）：真实/仿真的差异是数据，不是代码分支。仿真缺失的能力（导航/路网/回充/参数/导航点）配置里就是 `None`，`common` 层统一守卫——调用方永远拿到明确的 `False`/空结果/日志，不会 `AttributeError`。
3. **对外契约不变**：`ChassisClient` 的公开方法签名、参数含义、状态字符串取值均与 `dev_w1` 基线一致，`W1Manager` 侧零改动。
4. **通信稳健性由 `ActionTaskClient` 统一保证**，不是每个 action 各写一遍（见下）。

## ActionTaskClient（`common/action_task_client.py`）

管理一个 action 从发送到终态的完整生命周期，用于 **导航（NavigateToPose）**、**路网导航（NavigateThroughPoses）**、**精控（PreciseControl）**、**自动回充（AutoCharging）** 全部四类任务：

- **枚举状态机**：`IDLE → WAITING_SERVER → SENDING → ACCEPTED → EXECUTING → SUCCEEDED/CANCELED/ABORTED`，所有转移在锁内完成，避免"状态字符串和实际进度不一致"。
- **task_id 世代**：每次 `send()` 递增，所有回调闭包携带 task_id，过期回调自动丢弃——旧任务不会串扰新任务。
- **accept watchdog**：`send()` 后若在 `accept_timeout` 内没收到 goal response，判定 `LOST` 并告警，而不是无限停在 `SENDING`（防 action server 掉线的静默丢失）。
- **pending-cancel**：`cancel()` 在 goal 还未被接受时，登记标志立即返回；一旦收到 accepted，自动补发取消——不轮询、不阻塞、不丢失取消意图。
- **异常全部转成状态**：发送异常 → `FAILED_TO_SEND`；result 获取异常 → `LOST`；不会出现"异常被 log 吞掉、状态却停留在旧值"的情况。

新增一个 action 类接口，只需要在对应 feature 里构造 `ActionTaskClient(node, ActionType, action_name, label=...)`，组装 goal、状态映射即可，通常 50~80 行。

## 精控（`precise_control.py`）—— action 协议

`control/precise_control` action（`nav_interfaces/action/PreciseControl`；server 见 `mobile_perception/src/mobile_control_action_node.cpp`）：

| 字段 | 内容 |
|------|------|
| Goal | `target_pose`（PoseStamped，frame=`base_link`，相对位移/转角，x 与 yaw 互斥）——server 内部会 TF 转换到 map 系执行 |
| Result | `success`（bool）+ `message`（str）|
| Feedback | `current_status`（server 定义的状态字符串：`IDLE`/`CONTROL`/`CANCELLING`/`SUCCEEDED`）|

本 feature 直接透传 feedback 字符串作为查询结果，终态（`SUCCEEDED`/`CANCELLED`/`FAILED`）由 action result 二次覆盖确认，与 feedback 相互独立不冲突。

## 路网 / 多路点导航（`road_navigation.py`）

`/navigate_through_poses` action（`nav2_msgs/action/NavigateThroughPoses`）。`waypoints` 为 `[x, y, yaw_rad]` 列表，**yaw 单位是弧度**（与 Nav2 原生接口一致，故意不与 Nav2 单点导航的"度"统一——详见下方单位约定表）。状态机与自动回充共用同一套 `GoalStatus` 映射。

## 导航点查询（`nav_points.py`）

`/map_asset_manager/get_all_nav_points` service（`nav_interfaces/srv/GetAllNavPoints`），同步调用（内部走 `common.call_service`，带超时与守卫）。返回 `dict[点位名称] -> (x, y, theta_rad)`。

## 单位与坐标约定

| 接口 | position 含义 | yaw 单位 | frame |
|------|--------------|---------|-------|
| `set_chassis_nav2_position([x, y, yaw])` | 目标位姿 | **度** | map |
| `set_chassis_control_position([x, 0, yaw])` | 相对位移/转角，x 与 yaw 互斥 | **度** | base_link |
| `set_chassis_road_navigation(waypoints)` | 路点列表 | **弧度** | map（默认，可传 `frame_id` 覆盖）|
| `get_all_nav_points()` 返回值 | 点位坐标 | **弧度**（`theta`）| map |
| `set_goal_tolerance(xy, yaw)` | 到达容差 | **弧度**（Nav2 参数原生单位）| — |

单点导航/精控的 yaw 用度（贴近业务层直觉），路网导航/导航点/参数服务的 yaw 用弧度（直接透传 Nav2 原生语义）。**这是历史遗留的单位不一致，本次未统一**（改动会牵涉 `W1Manager` 调用方约定，属于 red line 之外的改动，见项目方案文档）。

## 状态字符串（对外兼容）

`ActionTaskClient` 内部用 `TaskState` 枚举，但公开方法仍返回旧版字符串，保证 `W1Manager` 不用改：

- **导航** `get_nav_state()`：`IDLE / NAVIGATING / SUCCEEDED / FAILED / CANCELLED`
- **路网导航** `get_road_navigation_status()`：`IDLE / WAITING_SERVER / SENDING / ACCEPTED / EXECUTING / CANCELING / SUCCEEDED / CANCELED / ABORTED / REJECTED / FAILED_TO_SEND / UNKNOWN`
- **回充** `get_auto_charging_status()`：同路网导航取值集合
- **精控** `get_control_status()`：`IDLE / CONTROL / SUCCEEDED / CANCELLING / CANCELLED / FAILED`（`CONTROL`/`CANCELLING` 来自 server feedback 透传；终态来自 action result）

## 使用方式

### 依附外部 Executor（如 W1Manager，默认方式）

```python
client = ChassisClient(is_sim=False)
# W1Manager 把 client 加入自己的 MultiThreadedExecutor 并 spin
```

### 独立使用（standalone，不依赖 W1Manager）

```python
client = ChassisClient(is_sim=False, standalone=True)   # 自带后台执行器线程
client.is_connected(timeout_sec=2.0)
client.set_chassis_nav2_position([1.0, 0.5, 90.0])
...
client.close()   # 释放执行器线程 + 销毁节点
```

### 测试（无硬件、无 W1Manager）

`src/examples/navigation_client_mock_server.py` + `navigation_client_standalone_test.py`：mock 底盘服务端 + 38 项自动判定驱动，覆盖导航/路网导航/导航点/精控/回充/参数/连接的正常与取消路径（分组：`conn`/`nav`/`road`/`points`/`ctrl`/`charge`/`param`，`--only <组>` 单跑）。运行方式与已知环境注意事项（务必用独立 `ROS_DOMAIN_ID` 隔离本机其他 nav2 进程）见脚本内 docstring。

## 已知限制 / TODO

- yaw 单位在导航/精控（度）与路网导航/参数（弧度）之间不统一，未在本次改动。
- 仿真模式当前仅支持：位姿反馈 + 精控 action；导航/路网导航/回充/参数/导航点接口在仿真下会安全返回 `False`/空结果（不崩溃，但也不可用）。
- 底盘精控的 sim 端 action server 是否真实存在未验证（仅确认真实模式 server 存在，见 `mobile_control_action_node.cpp`）。
