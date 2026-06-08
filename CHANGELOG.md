# CHANGELOG

## [Unrelease]
### Feat
- 适配ee_devices 0.4.3 (dexe_interfaces >= dev-b7a8dbf56ec23b6e6ff5b8b0a6f9383653bb05ed)

## [V0.5.0]
### Feat
- 支持dexe_rp提供的DoubleS轨迹规划算法和calc_next_iks算法
- 新增设置振动抑制相关的接口

### Improve
- 优化基于全身IK的move_line，增加eye_focus功能、单臂缺省输入、避开肘部极限等功能

### Fix
- get_current_qpos修改当前值为指令值, 防止出现累积偏差
- 修复run_mode值与server不一致的情况

## [V0.4.3]
### Fix
- 修复w1_params先于资产检查导致的bug

## [V0.4.2]
### Fix
- w1_params从URDF中读取关节极限

## [V0.4.1]
### Fix
- 修复了基于全身IK move_line中的一个浮点数累加计算bug

## [V0.4.0]
### Feat
- 增加了Toprra规划轨迹后的关节角超限检查
- 增加了streaming模式中use_filter,use_velocity,use_acceleration等开关
- 增加依赖全身IK求解的move_line规划器
- 增加了双臂固连move_line的向后屈腿模式

### Fix
- 修复了RRT类规划算法中的一些bug以及examples

### Refactor
- 部分规划接口改为被动接受起始关节角而非主动获取当前关节角

### Improve
- 增加了是否使用碰撞检测检查规划出的轨迹是否会发生自碰撞

## [V0.3.1]
### Feat
- 增加脚本安装指令 dexe_robot_client-install-scripts

### Fix
- 修复Ros2 node的初始化和销毁顺序，避免初始化失败

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