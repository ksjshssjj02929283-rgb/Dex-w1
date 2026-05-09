# dexe_robot_client 开发记录

## 模块入口
- W1 对外主入口：`src/dexe_robot_client/w1/w1_manager.py`
- 底盘导航与控制基类：`src/dexe_robot_client/navigation_client/navigation_client.py`
- W1 底盘 ROS2 topic/client 初始化：`src/dexe_robot_client/navigation_client/chassis_client.py`

## 底盘出桩接口
- `W1Manager.chassis_undocking(forward=True)` 会通过 `ChassisClient.chassis_undocking(forward)` 发布 `geometry_msgs/Twist` 到 `/control/cmd_vel`。
- 当前出桩速度曲线沿 `base_link` x 方向：最大线速度 `0.3m/s`，加速度 `0.3m/s^2`，持续 `2s`，发布频率 `20Hz`，结束后主动发布零速度。
- `forward=True` 时 `linear.x` 为正，机器人前进；`forward=False` 时速度取反，机器人后退。
- 调用前需要 `W1Manager(use_nav=True)` 且处于 real mode；导航客户端当前仅支持真实底盘模式。

## 路网/多路点导航接口
- 底层 action 使用 Nav2 `nav2_msgs.action.NavigateThroughPoses`，action 名为 `/navigate_through_poses`。
- `W1Manager.move_chassis_road_navigation(waypoints, frame_id="map", timeout_sec=10.0, behavior_tree="")` 非阻塞发送路点任务；每个路点格式为 `[x, y, yaw_rad]`，yaw 单位为弧度。
- `W1Manager.get_road_navigation_status()` 返回当前状态字符串，可能为 `IDLE`、`WAITING_SERVER`、`ACCEPTED`、`EXECUTING`、`CANCELING`、`SUCCEEDED`、`CANCELED`、`ABORTED`、`REJECTED`、`FAILED_TO_SEND`。
- `W1Manager.get_road_navigation_result()` 返回最近一次路网导航是否成功及结果状态。
- `W1Manager.cancel_chassis_motion()` 会同时尝试取消单点导航、路网导航和底盘控制任务。

## 底盘回充接口
- 回充 action 使用 `chassis_interfaces.action.AutoCharging`，action 名为 `/auto_charging`。
- `W1Manager.set_auto_charging()` 默认以 `force=True`、`timeout_sec=10.0` 非阻塞发送回充 goal。
- `W1Manager.get_auto_charging_status()` 返回当前状态字符串，可能为 `IDLE`、`WAITING_SERVER`、`ACCEPTED`、`EXECUTING`、`CANCELING`、`SUCCEEDED`、`CANCELED`、`ABORTED`、`REJECTED`、`FAILED_TO_SEND`。
- `W1Manager.get_auto_charging_result()` 返回最近一次回充是否成功及结果消息；`W1Manager.cancel_auto_charging()` 用于取消已接受的回充 goal。
