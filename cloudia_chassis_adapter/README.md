# cloudia_chassis_adapter

该包是迦智底盘 WebSocket 的服务端适配器。`backend=cloudia` 时，它是
WebSocket 的唯一连接所有者：接收 `upstream_state`，发布位姿和完整状态 JSON，
并把 `/control/cmd_vel` 以固定周期转换成 `downstream_command/speedCommand`。

默认 `backend=dexe` 时不会建立连接、不会发送速度，也不会发布 Cloudia 状态。

启动前需要设置：

```bash
export DEXE_CHASSIS_BACKEND=cloudia
export CLOUDIA_HOST=192.168.17.107
ros2 launch cloudia_chassis_adapter cloudia_chassis.launch.py
```

Cloudia 模式下不要同时启动本地 `ChassisNode` 和其它同名底盘状态发布者；
否则 `/control/cmd_vel` 和标准状态话题会有两个底盘后端。

## ROS 接口

| 方向 | 默认话题 | 类型 | 参数 |
|---|---|---|---|
| 订阅 | `/control/cmd_vel` | `geometry_msgs/msg/Twist` | `cmd_vel_topic` |
| 发布 | `/feedback/chassis_pose` | `geometry_msgs/msg/PoseStamped` | `pose_topic` |
| 发布 | `/feedback/cloudia_state_raw` | `std_msgs/msg/String` | `raw_state_topic` |

原始状态的 `data` 是完整 `upstream_state` 文本：保留 `header`、`state`、
`information`、位姿以及未知字段，保持原字段、单位和文本格式，不重组 JSON。
仅转发可解码的 JSON 对象且 `communicationType == "upstream_state"` 的帧；
每收到一帧发布一次，不通过定时器重播缓存。发布 QoS 为 KeepLast(10)、Reliable、Volatile。

位姿单独读取 `state.agvPosition.x/y/theta`，theta 保持弧度并转为四元数；
`header.frame_id` 使用 `frame_id` 参数（默认 `map`），时间戳仍使用本节点 ROS 时间。
位姿缺失、类型错误或非有限数值时不发布位姿，但原始 JSON 照常转发。

原始转发不等于状态验证成功：`state` 缺失、为空或不是对象时仍保留原文，
但不刷新用于速度保护的状态接收时间。非空 `state` 对象刷新该时间，位姿可以缺失；
其余业务字段的有效性由消费者判断。原始话题消费者必须自行校验状态内容和新鲜度。

速度通道按 ROS 输入是否新鲜持有，不能按速度数值是否为零判断：

- 启动后没有收到速度输入时只接收状态，不发送 `speedCommand`。
- 有新鲜 `/control/cmd_vel` 时按 `command_rate_hz` 发送，包括持续的故障零速度。
- 输入超过 `command_timeout_s`（默认 0.5 秒）后，成功写入 3 帧零速度再停止发送，
  释放迦智内部速度仲裁通道；默认 10 Hz 下停车尾段约 0.3 秒。
- 输入仍新鲜但状态超时时，继续发送零速度保护。
- 断开/退出只在尚未释放速度控制时尝试发送零速；重连不重放旧速度，等待新的 ROS 输入。
  停车发送失败会保留待停车状态，重连后先尝试零速。

零速帧成功写入 WebSocket 不等于底盘已停稳，停车确认仍由上层结合反馈完成。
连接和超时日志继续输出，但不再直接生成 ROS 诊断消息。

## 停止发布的接口与部署影响

本适配器不再发布 `/feedback/battery_status`、`obstacle_detector/has_obstacle`、
`/cloudia/chassis_velocity`、`/diagnostics_agg`。对应 `battery_topic`、`obstacle_topic`、
`velocity_topic`、`diagnostics_topic` 参数已移除，自定义参数文件也应更新。

Supervisor 后续订阅完整 JSON，自行解析电池、障碍物、速度和故障等，经原有
`/diagnostics → diagnostic_aggregator → /diagnostics_agg` 链路聚合。
本次仅修改 Server，未修改或迁移 Supervisor、Client、网页和产测平台；这些消费者
在迁移前不会再从本适配器收到上述四个话题的数据，其它节点的发布行为不受本包控制。
位姿消费者继续使用 `/feedback/chassis_pose`；Cloudia 模式应确保位姿来源唯一。

重新构建并部署本包后，重启适配器使代码和 YAML 一起生效。

```bash
ros2 topic echo /feedback/cloudia_state_raw
ros2 topic echo /feedback/chassis_pose
```

## 定向验证

source ROS 2 环境后，在包目录运行：

```bash
python3 -m unittest discover -s test -v
```

测试使用内存发布器，不连接机器人；覆盖原文保留、未知字段、异常状态/位姿、
禁用后端、状态/命令超时保护。实际 DDS、WebSocket 和机器人运动仍需部署后验证。
