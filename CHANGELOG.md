# CHANGELOG

## [Unrelease]

## [V0.8.2]
### Refactor
- ignored_collision_pairs.json不从内网下载，改为自动生成


## [V0.8.1]
### Refactor
- 优化ros2 client的初始化，避免超时等待
- 优化碰撞检测结果返回，无碰撞返回StatusCode.SUCCESS，碰撞则返回StatusCode.COLLISION_FAIL

### Improve
- logger增加日志节流机制

### Fix
- 修复toppra轨迹规划采样漏掉最后到位点的问题
- 修复end_effector_client未处理UNKNOWN状态、关节状态长度不匹配及位置越界反馈导致的异常问题


## [V0.8.0]
### Feat
- 新增coal collision checker
- 支持左右不同末端执行器类型
- 新增调度路网接口graph points client

### Refactor
- 重构全身IK模块，提供IK manager
- 重构navigation client

### Improve
- 自动从urdf的link tree加载碰撞体

### Remove
- 移除 dexsim server
- 移除 fcl_checker
- 移除 planner_pinocchio
- 移除 030硬件专用全身IK
- 暂时移除 ompl planner、hybrid planner
- 暂时移除 末端执行器的碰撞检测


## [V0.7.0]
### Feat
- 支持W1 V030硬件版本
- 支持 W1Manager 类构造函数，自动从系统配置中读取类型

### Refactor
- W1Manager构造使用 dexe_rp 中的RobotVersion 和 EndEffectorType

### Remove
- 移除move_torso_linear()、move_arm_linear()、rotate_arm_linear()
- 移除transfer_grasp_pose()
- 移除 W1ArmType、W1Version 和 EndEffectorType

### Improve
- 优化StatusCode日志输出

## [V0.6.0]
### Feat
- 适配ee_devices 0.4.3 (dexe_interfaces >= dev-e112b31c851b361d74156e2deee1e46ae5e8cac1)

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