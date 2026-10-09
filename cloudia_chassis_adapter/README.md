# cloudia_chassis_adapter

该包是迦智底盘 WebSocket 的服务端适配器。`backend=cloudia` 时，它是
WebSocket 的唯一连接所有者：接收 `upstream_state`，发布现有 ROS 2 状态，
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
