# CHANGELOG

## [Unreleased]
### Feat
 - 支持双臂固连的MoveLine(基于双臂IK和基于全身IK)
 - 支持双臂多个任务点一一对齐同步的MoveLine
 
### Fix

### Refactor

### Improve
- 优化路径、轨迹的碰撞检测器（仅作为checker，不参与路径生成）
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