# dexe_robot_client 开发记录

## 模块入口
- W1 对外主入口：`src/dexe_robot_client/w1/w1_manager.py`
- 底盘导航与控制基类：`src/dexe_robot_client/navigation_client/navigation_client.py`
- W1 底盘 ROS2 topic/client 初始化：`src/dexe_robot_client/navigation_client/chassis_client.py`

## 底盘出桩接口
- `W1Manager.chassis_undocking()` 会通过 `ChassisClient.chassis_undocking()` 发布 `geometry_msgs/Twist` 到 `/control/cmd_vel`。
- 当前出桩速度曲线沿 `base_link` 正 x 方向：最大线速度 `0.3m/s`，加速度 `0.3m/s^2`，持续 `2s`，结束后主动发布零速度。
- 调用前需要 `W1Manager(use_nav=True)` 且处于 real mode；导航客户端当前仅支持真实底盘模式。
