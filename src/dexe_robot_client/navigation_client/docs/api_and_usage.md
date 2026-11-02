# navigation_client 接口与使用说明（使用侧）

> **适用包版本**：`0.7.1`
> **代码基线**：`xmk/modify_navigation1` 基于 `glf/modify_navigation@4832984` 的当前 rebase 工作树
> **文档集版本**：`1.4.0`
> **更新日期**：2026-08-04

## 1. 快速结论

- 直接入口：`from dexe_robot_client.navigation_client import ChassisClient`。
- W1 application 的后端通常通过 `W1Manager` 间接调用，不需要自行管理 ROS action。
- `set_*` action 方法不等待物理任务完成，但会同步等待 server 就绪；是否最终成功要继续读
  状态/result。
- `get_all_nav_points()` 和四个参数设置方法是同步 service 流程，可能等待到超时。
- 当前正式自动入口是 `python3 -m dexe_robot_client.navigation_client.interface_test`；
  `dexe-navigation-client-test` 只在 pip/wheel 安装生成 console script 后可用。
- production 路网和路径导航接口均保留，但当前 `RC-A-01～16` 不测试 through-poses 或
  graph-points；路径导航暂由 examples standalone `graph` 分组补充回归。

## 2. 创建与执行器

### 2.1 W1Manager（推荐业务入口）

```python
from dexe_robot_client.common import StatusCode, W1ArmType, W1Version
from dexe_robot_client.w1 import W1Manager

w1 = W1Manager(
    is_sim=False,
    arm_type=W1ArmType.ANTHROPOMORPHIC,
    w1_version=W1Version.V2,
    use_nav=True,
)

if w1.connect() == StatusCode.SUCCESS:
    w1.move_chassis_nav2([1.0, 0.5, 90.0])
```

### 2.2 直接使用

```python
import rclpy
from dexe_robot_client.navigation_client import ChassisClient

rclpy.init()
client = ChassisClient(is_sim=False, standalone=True)
try:
    if client.is_connected(timeout_sec=2.0):
        client.set_chassis_nav2_position([1.0, 0.5, 90.0])
finally:
    client.close()
    rclpy.shutdown()
```

`standalone=False` 是默认值，调用方必须把节点加入自己的 executor 并 spin；
`standalone=True` 自带后台 executor，必须调用 `close()`。

## 3. 生命周期与底盘状态

| `ChassisClient` 方法 | 参数 | 返回 | 行为 |
|---|---|---|---|
| `close()` | 无 | 无 | 关闭 standalone runner 并销毁节点 |
| `spin_once(timeout_sec=0.1)` | 秒 | 无 | 未被外部 executor 管理时处理一次回调 |
| `spin()` | 无 | 无 | 未被外部 executor 管理时阻塞 spin |
| `is_connected(timeout_sec=0.1)` | 秒 | `bool` | 在超时内等待有效位姿反馈 |
| `get_chassis_position()` | 无 | `(bool, ndarray\|None)` | `[x, y, yaw_deg]`；位姿超过连接阈值会清空 |
| `get_battery_percentage()` | 无 | `(bool, float)` | 0～100；未收到时 `(False, -1.0)` |
| `get_front_obstacle()` | 无 | `bool` | 最近一次前障碍物状态；SIM 未订阅时默认 `False` |

底盘“连接”在 client 内的直接判据是 `/feedback/chassis_pose` 是否在连接超时内持续更新。
业务上游可能以 `map→base_link` 定位有效为可用条件，但 W1Manager 并不在这里直接查询 TF。

## 4. 单点导航

| 方法 | 参数 | 返回 |
|---|---|---|
| `set_chassis_nav2_position(position)` | `[x, y, yaw_deg]`，frame=`map` | `bool`，目标是否成功发出 |
| `get_nav_state()` | 无 | `IDLE/NAVIGATING/SUCCEEDED/FAILED/CANCELLED` |
| `has_cancelable_nav_goal()` | 无 | `bool` |
| `cancel_chassis_nav2()` | 无 | `bool`，取消是否已发送或登记 |

```python
if client.set_chassis_nav2_position([2.0, -0.5, 180.0]):
    while client.get_nav_state() in {"IDLE", "NAVIGATING"}:
        client.spin_once(0.1)
```

## 5. 路网/多路点导航（production 兼容）

| 方法 | 参数 | 返回 |
|---|---|---|
| `set_chassis_road_navigation(waypoints, frame_id="map", timeout_sec=10.0, behavior_tree="")` | `[[x,y,yaw_rad], ...]` | `bool` |
| `get_road_navigation_status()` | 无 | action 状态字符串 |
| `get_road_navigation_result()` | 无 | `(bool, str)` |
| `has_cancelable_road_navigation_goal()` | 无 | `bool` |
| `cancel_road_navigation()` | 无 | `bool` |

该接口仍被 `W1Manager.move_chassis_road_navigation()` 消费，因此保留。当前正式自动自测已按
评审结论移除 through-poses 用例；这不代表 production API 已废弃。

### 5.1 路径导航（按点位名称）

| 方法 | 参数 | 返回 |
|---|---|---|
| `set_chassis_graph_points_navigation(nav_point_names, nav_point_ids=None, pose=None, target_yaw=None, timeout_sec=10.0)` | 点位名称列表必传；其余 goal 字段可选透传 | `bool` |
| `get_graph_points_navigation_status()` | 无 | action 状态字符串 |
| `get_graph_points_navigation_result()` | 无 | `(bool, str)` |
| `has_cancelable_graph_points_navigation_goal()` | 无 | `bool` |
| `cancel_graph_points_navigation()` | 无 | `bool` |

底层 action 为 `/navigate_through_graph_points`（`nav_interfaces/NavigateThroughGraphPoints`）。
client 不限制点位数量，也不阻止并发 goal，是否接受由服务端裁决；空列表等业务规则因此
必须结合服务端结果判断。当前 `nav_point_ids`、`pose` 和 `target_yaw` 只是可选透传字段，
服务端尚未消费。SIM 未配置该 action，会 fail-closed 返回 `False`。

### 5.2 迦智调度导航（`CLOUDIA_SCHED_IP` 启用）

连迦智**调度**服务（`http://<调度>:8858`，无鉴权），与 `DEXE_CHASSIS_BACKEND` 正交。目标是
点位 id / 站点编码，**不是**世界坐标，所以不复用 `set_chassis_nav2_position`。

| 方法 | 参数 | 返回 |
|---|---|---|
| `set_chassis_sched_target(id=None, name=None, position=None)` | `name`：`str`（优先），迦智底盘上即点位 id 的字符串，内部转 `int`；`id`：点位 id，`name` 为 `None` 时才用，原样下发；`position` 暂不使用。`id`/`name` 不能同时为 `None` | `bool`（任务创建成功） |
| `get_sched_nav_state()` | 无 | `IDLE / NAVIGATING / SUCCEEDED / FAILED / CANCELLED` |
| `get_sched_nav_result()` | 无 | `(bool, str)`，失败时为调度 `err_msg` |
| `has_cancelable_sched_goal()` | 无 | `bool` |
| `cancel_sched_nav()` | 无 | `bool` |

环境变量：`CLOUDIA_SCHED_IP`（调度主机 IP，必填）、`CLOUDIA_SCHED_PORT`（缺省 8858，迦智调度 /api/v1 接口端口）、`CLOUDIA_SCHED_ROBOT`（调度车号，必填正整数，缺失/非法即报错拒绝启动）、
`CLOUDIA_SCHED_MAP`（缺省取该车当前地图）。轮询间隔/失败阈值复用
`CLOUDIA_POLL_INTERVAL_S` / `CLOUDIA_POLL_FAIL_LIMIT`。W1Manager 对应
`move_chassis_sched` / `get_sched_nav_status`，`cancel_chassis_motion` 已并入调度取消。
现场前提：车须处于调度模式且电量足够，否则任务一直排队（`NAVIGATING`）。

## 6. 导航点查询

| 方法 | 参数 | 返回 |
|---|---|---|
| `get_all_nav_points()` | 无 | `dict[name] -> (x, y, theta_rad)`；失败/空数据均返回 `{}` |

底层 service 为 `/map_asset_manager/get_all_nav_points`。接口本身无法从空字典区分“服务失败”
和“确实没有点位”，需要结合日志判断。

## 7. 底盘精控

| 方法 | 参数 | 返回 |
|---|---|---|
| `set_chassis_control_position(position)` | `[x, y, yaw_deg]`；实际只使用 x/yaw，二者不可同时非零 | `bool` |
| `get_control_status(consume=True)` | `consume` 为兼容参数，当前不消费终态 | 状态字符串 |
| `refresh_control_status()` | 无 | `True`，兼容入口 |
| `has_cancelable_control_goal()` | 无 | `bool` |
| `cancel_chassis_control()` | 无 | `bool` |

目标 frame 为 `base_link`。界面可能显示 `[x,y,theta]`，但当前 goal 只写 x 和 yaw，y 不参与
目标构造。直线与旋转应分两次执行。

```python
client.set_chassis_control_position([0.20, 0.0, 0.0])   # 前进 0.20m
client.set_chassis_control_position([0.0, 0.0, 30.0])   # 原地旋转 30°
```

## 8. 自动回充

| 方法 | 参数 | 返回 |
|---|---|---|
| `set_chassis_auto_charging(force=True, timeout_sec=10.0)` | 是否强制、server 等待超时 | `bool` |
| `get_auto_charging_status()` | 无 | action 状态字符串 |
| `has_cancelable_auto_charging_goal()` | 无 | `bool` |
| `get_auto_charging_result()` | 无 | `(bool, str)` |
| `cancel_auto_charging()` | 无 | `bool` |

W1 application 当前不是直接点击底层回充 action，而是创建回充任务后通过“执行任务”触发
W1Manager，再由 W1Manager 调用 client。

## 9. Nav2 参数

| 方法 | 参数 | 实际写入 | 返回 |
|---|---|---|---|
| `set_goal_tolerance(xy_tolerance, yaw_tolerance)` | 米、弧度；均须大于 0 且有限 | 当前 `current_goal_checker` 支持的 xy 或 lon/lat 及 yaw tolerance | `bool` |
| `set_footprint(vertices)` | 至少 4 个有限数值 `[x,y]` | local/global footprint | `bool` |
| `set_inflation_radius(radius)` | 米 | local/global inflation radius | `bool` |
| `set_follow_velocity_limits(vx_max, wz_max)` | m/s、rad/s | `FollowPath.vx_max/wz_max` | `bool` |

四个接口都是“写入 → 等待 0.1s → 读取校验”，只有全部 service 成功且读回值匹配才返回
`True`。SIM 未配置参数节点，会 fail-closed 返回 `False`。

## 10. 底层接口速查

| 模式 | 类别 | 接口 |
|---|---|---|
| REAL | Topic | `/feedback/chassis_pose`、`/diagnostics_agg`、`obstacle_detector/has_obstacle` |
| REAL | Action | `navigate_to_pose`、`/navigate_through_poses`、`/navigate_through_graph_points`、`control/precise_control`、`/auto_charging` |
| REAL | Service | `/map_asset_manager/get_all_nav_points` |
| REAL | Parameter node | `/controller_server`、`/local_costmap/local_costmap`、`/global_costmap/global_costmap` |
| SIM | Topic | `/feedback_sim/chassis_pose` |
| SIM | Action | `control/precise_control` |
| SIM | Parameter node | `/controller_server`（目标容差） |

## 11. W1Manager 对照

| W1Manager | ChassisClient |
|---|---|
| `move_chassis_nav2` | `set_chassis_nav2_position` |
| `move_chassis_road_navigation` | `set_chassis_road_navigation` |
| `move_chassis_sched` | `set_chassis_sched_target` |
| `get_sched_nav_status` | `get_sched_nav_state` |
| `move_chassis_ctrl` | `set_chassis_control_position` |
| `get_current_chassis_pose` | `get_chassis_position` |
| `get_current_battery_percentage` | `get_battery_percentage` |
| `get_front_obstacle` | `get_front_obstacle` |
| `get_nav_status` | `get_nav_state` |
| `get_road_navigation_status/result` | `get_road_navigation_status/result` |
| `get_all_nav_points` | `get_all_nav_points` |
| `get_control_status` | `get_control_status` |
| `set_auto_charging` | `set_chassis_auto_charging` |
| `cancel_auto_charging` | `cancel_auto_charging` |
| `cancel_chassis_motion` | 同时调用五类 cancel（含路径导航）|

W1Manager 将多数 `bool` 包装为 `StatusCode`。当前没有新增路径导航发送门面，但统一
`cancel_chassis_motion()` 已覆盖路径导航；五类取消中任一成功即返回 `StatusCode.SUCCESS`。

## 12. 状态判定

| 能力 | 对外状态 |
|---|---|
| 单点导航 | `IDLE / NAVIGATING / SUCCEEDED / FAILED / CANCELLED` |
| 路网导航 | `IDLE / WAITING_SERVER / SENDING / ACCEPTED / EXECUTING / CANCELING / SUCCEEDED / CANCELED / ABORTED / REJECTED / FAILED_TO_SEND / UNKNOWN` |
| 路径导航 | 同路网导航 |
| 回充 | 同路网导航 |
| 精控 | `IDLE / CONTROL / SUCCEEDED / CANCELLING / CANCELLED / FAILED` |

不同能力保留历史拼写，调用方应按对应接口判断，不能假设全局只有一种 canceled 拼法。

## 13. 延迟诊断

默认模式始终打印触发与返回总耗时。需要定位 app→client、server 查询、goal accept、feedback
或 result 回传时，在启动进程前设置：

```bash
export DEXE_NAV_CLIENT_DIAGNOSTICS=1
```

诊断检查点为 `触发 → server就绪 → 已发出 → accepted → 进度 → result`。多线程日志打印
顺序可能乱序，必须按 task_id 和时间数值分析。

## 14. 自动接口自测

### 14.1 PC2 直接复制部署

PC2 当前只复制 `navigation_client/` 到 Python `site-packages`，不会执行 `pyproject.toml`，
因此正式入口为：

```bash
python3 -m dexe_robot_client.navigation_client.interface_test list-tests
python3 -m dexe_robot_client.navigation_client.interface_test mock --domain-id 188
```

正式 `mock` 默认每个用例执行 15 轮，共 16×15=240 次调用。逐轮编号为
`RC-A-xx/R01～R15`，`summary.json` schema v2 保存 `repeat_per_case`、
`case_summaries` 和全部逐轮 `tests[]`；`summary.md` 给出每项 15 轮通过数和耗时统计。
命令结束前还会打印“自动测试最终结果”表格，逐项显示计划/完成轮次、
PASS/FAIL/ERROR/PENDING_ENV 和结论，并用 `TOTAL` 行汇总总体状态。
`--repeat <15..100>` 只允许提高压力轮次，不能降到 15 以下。

执行前核对实际导入位置：

```bash
python3 -c 'import dexe_robot_client.navigation_client as m; print(m.__file__)'
```

只有通过 pip/wheel 安装后，以下别名才是等价入口：

```bash
dexe-navigation-client-test mock --domain-id 188
```

### 14.2 当前 16 个自动用例

| ID | 名称 |
|---|---|
| `RC-A-01` | 底盘状态读取 |
| `RC-A-02` | 位姿断连与恢复 |
| `RC-A-03` | Nav2 导航成功 |
| `RC-A-04` | Nav2 导航取消 |
| `RC-A-05` | 导航点查询 |
| `RC-A-06` | 底盘精控成功 |
| `RC-A-07` | 底盘精控取消与非法输入 |
| `RC-A-08` | 自动回充成功 |
| `RC-A-09` | 自动回充取消 |
| `RC-A-10` | 目标容差参数 |
| `RC-A-11` | footprint 参数 |
| `RC-A-12` | 膨胀半径参数 |
| `RC-A-13` | 跟随速度上限参数 |
| `RC-A-14` | 仿真模式能力边界 |
| `RC-A-15` | 导航非法输入拒绝 |
| `RC-A-16` | footprint 三点输入拒绝 |

结果写入 `${HOME}/.dexe/navigation/robot_client_interface_test/mock_<时间戳>/`。原
`RC-A-01～14` 保持不变，拒绝路径追加为 `RC-A-15/16`；`R01～R15` 是调用轮次。
`RC-A-11` 每轮先设置四点矩形、再设置六点凸多边形，两次都自动比较 local/global
costmap 的实际回读，因此 15 轮共完成 30 次合法 footprint 设置。
路径导航尚未加入本表；可用 examples 下的 `navigation_client_standalone_test.py --only graph`
做开发侧发送/结果/取消回归，但该结果不计入 240 轮正式结论。
开发机 mock
PASS 不能替代 PC2 同步、W1Manager、真实运动、回充和 application 手动验收。

## 15. 常见失败定位

- 找不到 `dexe-navigation-client-test`：当前是直接复制部署，使用 `python3 -m ...`。
- 导航返回 `False`：先查输入是否为 3 个有限数值，再查 action server 是否在等待时间内可用。
- 路径导航返回 `False`：确认 `/navigate_through_graph_points` server 可用及 action 类型一致；
  goal 被接受后失败则继续检查服务端点位解析和 result message。
- 精控返回 `False`：确认 x 和 yaw 没有同时非零；y 即使输入也不会进入 goal。
- 参数接口超时：避免从占用唯一 executor 的回调线程同步调用，并检查目标参数节点。
- `get_all_nav_points()` 返回 `{}`：结合日志区分 service 失败和确实无点位。
- mock 无法写 ROS 日志：设置可写的 `ROS_LOG_DIR` 后重试；这属于环境阻塞，不是功能 PASS。

## 16. 文档版本检查

从仓库根目录执行：

```bash
python3 src/dexe_robot_client/navigation_client/docs/check_docs.py
```

检查通过只证明当前源码契约与交付文档一致，不证明目标机器人功能通过。
