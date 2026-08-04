# navigation_client

底盘 / 导航能力的客户端封装，供 `w1.W1Manager` 及以上业务层调用。本文档面向本模块的维护者与直接使用者。

> 包版本：`dexe_robot_client 0.7.1`；代码基线：`glf/modify_navigation@7e07427` + 当前工作树（基于 `dev_w1`）。本文档描述**重构 v2** 之后的结构（原单文件 `navigation_client.py` 已拆分并删除）。
>
> 正式交付文档：[`docs/README.md`](docs/README.md)；开发侧设计见
> [`docs/design.md`](docs/design.md)，调用侧接口说明见
> [`docs/api_and_usage.md`](docs/api_and_usage.md)。`docs/` 是随代码版本维护的交付文档 SSOT。

## 目录结构

```text
navigation_client/
├── __init__.py                  # 导出 ChassisClient
├── chassis_client.py             # 对外门面：全部公开 API，组合 features，按 is_sim 选配置
├── interfaces.py                 # 接口配置表：REAL / SIM 两份 topic/action/service 名称
├── interface_test.py             # RC-A-01～16 自动接口自测入口
├── test_support/                 # 自动自测的进程内 ROS2 mock peers
├── docs/                         # 随包版本维护的交付文档、版本清单与一致性检查
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
2. **接口配置化**（`interfaces.py`）：真实/仿真的差异是数据，不是代码分支。仿真支持位姿、精控和 controller 参数；缺失的导航/路网/回充/导航点配置为 `None`，由 `common` 层统一守卫。
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
| `set_goal_tolerance(xy, yaw)` | 当前 GoalChecker 的到达容差 | yaw 为**弧度**（Nav2 参数原生单位）| — |

单点导航/精控的 yaw 用度（贴近业务层直觉），路网导航/导航点/参数服务的 yaw 用弧度（直接透传 Nav2 原生语义）。**这是历史遗留的单位不一致，本次未统一**（改动会牵涉 `W1Manager` 调用方约定，属于 red line 之外的改动，见项目方案文档）。

## 延迟诊断打点（统一检查点链，默认关闭）

四个"触发类"公开方法（`set_chassis_nav2_position` / `set_chassis_road_navigation` /
`set_chassis_control_position` / `set_chassis_auto_charging`）在同一个
task_id 上走一串**检查点**，覆盖**发送链路**（触发→server就绪→已发出→
accepted）和**反馈链路**（进度→result）。

### 正常模式（默认，诊断关闭）

只打印两行**普通日志**（不带 `[延迟诊断]` 标签），不刷屏：

```
[<label>] 触发 wall=<HH:MM:SS.mmm>
[<label>] task_id=<N> 返回 wall=<HH:MM:SS.mmm>  耗时=<总耗时>
```

### 诊断模式（排查问题时打开）

打开后，中间检查点（`server就绪`/`已发出`/`accepted`/`进度`）也会打印，全部由
`common/timing.format_checkpoint()` 生成，**格式完全一致**：

```
[延迟诊断][<label>] task_id=<N> 阶段=<阶段名> wall=<HH:MM:SS.mmm>  Δ(<上一阶段>→<阶段名>)=<X.X>ms  累计(自触发)=<Y.Y>ms
```

打开方式（二选一）：
- 环境变量（启动前设置，进程级）：`export DEXE_NAV_CLIENT_DIAGNOSTICS=1`
- 代码里调用（运行时切换）：`from dexe_robot_client.navigation_client.common import timing; timing.set_diagnostics_enabled(True)`

排查完问题记得关掉（或直接不设环境变量重启），避免正常运行时刷屏。

| 检查点 | 所在链路 | 含义 |
|-------|---------|------|
| `触发` | 发送链路起点 | `chassis_client.py` 方法入口——客户端收到上层调用的时刻（**唯一**没有 `Δ`/`累计` 字段，因为它是基准；两种模式下都会打印）。与上层 application 自己记录的调用时刻相减 → app→client 延迟 |
| `server就绪` | 发送链路 | `wait_for_server()` 返回后——`Δ` 反映 DDS graph 查询 + 触发到这里的客户端开销（仅诊断模式打印）|
| `已发出` | 发送链路 | 真正调用 `send_goal_async` 前一刻——`Δ` 反映 `server就绪` 到这里的代码开销，正常应接近 0（仅诊断模式打印）|
| `accepted` | 发送链路 | 收到 goal 被接受的响应——`Δ` = 网络 + server 处理耗时（原"accept 往返"，仅诊断模式打印）|
| `进度` | 反馈链路 | 每条 feedback 到达（受 `feedback_log_interval` 节流打印，内部记账不节流）——`Δ` 反映相邻两次 feedback 间隔；正常模式下只打印内容（无诊断数值），诊断模式下带完整 `Δ`/`累计` |
| `result`/`返回` | 反馈链路 | goal 完成/取消/中止（**两种模式下都会打印**，格式不同）——诊断模式的 `Δ(进度→result)` = **result 回传本身的通信延时**（不含执行耗时）；正常模式的"耗时"= 端到端总耗时（触发→返回）|

`wall`（挂钟时间 `time.time()`，非 ROS clock/monotonic）用于跨进程比较（与上层 application 自己的时间戳相减）；`Δ`/`累计`/"耗时"都用同一挂钟时间做差。发送链路和反馈链路共用同一套 `_checkpoint()`/`format_checkpoint()`/`format_plain_*()` 实现，两种模式下格式内部各自完全一致。

### 已知注意事项：日志打印顺序可能与真实执行顺序不一致

`MultiThreadedExecutor` 下，触发回调（用户调用 `set_chassis_control_position` 等）与 `send_goal_async` 的响应回调（`_on_goal_response`/`_on_feedback`）可能跑在不同线程上。曾在真机日志上观察到："已发出"检查点这行打印，**在日志文件里出现在**"accepted"检查点甚至"SENDING -> ACCEPTED"状态转移**之后**——这不是时间戳错误（每个检查点的 `ts` 都是在对应事件发生的那一刻捕获的，数值可信），而是打印该行日志的线程本身被调度延迟了（GIL 竞争 / 其他回调线程抢先执行）。**结论：诊断延迟时永远看 `wall`/`Δ`/`累计` 的数值，不要看日志文件里几行谁先谁后。**

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

### 自动接口自测（无硬件、无 W1Manager）

当前正式自动目录为 `RC-A-01～RC-A-16`。进程内 mock 通过真实 rclpy
topic/action/service/parameter 通道覆盖连接与状态读取、Nav2 单点导航、导航点查询、
精控、回充、参数读写和仿真能力边界；每个用例默认连续执行 15 轮，以
`RC-A-xx/R01～R15` 记录逐轮期望—实际和状态，再按用例汇总 15/15。真实运动与人工观察
不进入自动结论。

PC2 当前采用“复制 `navigation_client/` 到 Python `site-packages`”的更新方式，正式入口
使用模块调用，不依赖 console script：

```bash
python3 -m dexe_robot_client.navigation_client.interface_test list-tests
python3 -m dexe_robot_client.navigation_client.interface_test mock --domain-id 188
```

`mock` 的 `--repeat` 默认值和下限均为 `15`，可为压力验证提高轮次，但不能降到 15 以下。
成功用例只验证成功路径；`RC-A-11` 每轮依次验证四点、六点两种合法 footprint，且两次
都要求 local/global costmap 精确回读一致。导航非法输入和三点 footprint 分别由
`RC-A-15/16` 验证“返回 false 且未发送 goal/未修改参数”。正式全量仍为 16×15=240 轮，
其中 `RC-A-11` 在 15 轮内共执行 30 次合法 footprint 设置。

全部选定用例结束后，终端会输出“自动测试最终结果”表格，逐项列出计划、完成、
PASS、FAIL、ERROR、PENDING_ENV 和结论，并以 `TOTAL` 行给出总体结果；详细逐轮记录继续
写入 `summary.json` 与 `summary.md`。

`ROS_DOMAIN_ID=188` 用于隔离本机其他 Nav2 进程；如现场已占用，应换成批准的空闲域。
运行结果写入
`${HOME}/.dexe/navigation/robot_client_interface_test/mock_<时间戳>/`。

只有执行过 pip/wheel 安装并生成 console script 后，才可使用等价的可选短命令：

```bash
dexe-navigation-client-test list-tests
dexe-navigation-client-test mock --domain-id 188
```

直接复制包目录不会创建 `${HOME}/.local/bin/dexe-navigation-client-test`，因此不能用
`command -v dexe-navigation-client-test` 作为 PC2 当前部署方式的前置门禁。

## 已知限制 / TODO

- yaw 单位在导航/精控（度）与路网导航/参数（弧度）之间不统一，未在本次改动。
- 仿真模式当前支持：位姿反馈、精控 action、`/controller_server` 目标容差参数；导航/路网导航/回充/导航点接口会安全返回 `False`/空结果。
- 底盘精控的 sim 端 action server 是否真实存在未验证（仅确认真实模式 server 存在，见 `mobile_control_action_node.cpp`）。
