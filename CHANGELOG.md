# CHANGELOG

## [Unreleased]

## [V0.3.0]
### Feat
 - 支持双臂固连的MoveLine(基于双臂IK和基于全身IK)
 - 支持双臂多个任务点一一对齐同步的MoveLine
 - 增加robot_client读取机器人配置的接口 get_robot_type(), get_arm_type() 和 get_robot_version()
 
### Fix
- 修复 plan_cartesian_traj 的 bug

### Refactor
- W1Manager对外API返回值从bool修改为StatusCode

### Improve
- 优化路径、轨迹的碰撞检测器（仅作为checker，不参与路径生成）
- 增加碰撞检测中末端执行器之间是否检测的开关
- 支持碰撞检测返回结果包含碰撞对和碰撞时刻的可视化模型

### Remove

## [V0.2.4]
### Feat
- 支持灵心巧手 EndEffectorType.LINKER_HAND
- 增加关节空间blend路径生成
- 增加导航中取消导航、取消控制、获取状态、获取电量等接口

### Fix

### Refactor

### Improve
- 优化笛卡尔空间blend路径生成

### Remove
- 移除open3d的依赖