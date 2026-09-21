# 阶段 1

当前阶段目标：统一底盘类型配置来源，并将差速/全向速度解算统一接入 `RobotServer`。

已完成：

- 公共 `ChassisType` 解析函数；
- `RobotServer` 与 `ChassisNode` 使用同一底盘配置；
- `RobotServer` 全向反馈缓存与闭环解算；
- 底盘目标互斥保护和驻车控制权超时接管。

保留项：原始 `mobile_cmd_omni_region.cpp` 未修改；其 Action 专用副本仍需后续裁剪并接入构建/launch。
