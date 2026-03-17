# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

DexEmbodied Robot Client 是一个 Python 机器人客户端库，用于控制 W1 人形机器人及其末端执行器。项目基于 ROS2 Humble，使用 Python 3.10。

## 构建与安装

### 初始化子模块
```bash
git submodule update --init --recursive
```

### 构建 ROS2 接口包
```bash
cd ./dexe_interfaces
colcon build --packages-select robot_server_interfaces
source ./dexe_interfaces/install/setup.bash
```

### 安装客户端库
```bash
pip install -e .  # 开发模式安装
dexe_robot_client-install-scripts  # 安装示例和脚本到 dexe_robot_client_data
```

### 打包
```bash
python3 -m build  # whl 包生成在 dist/ 目录
```

## 代码风格检查

### 检查代码风格
```bash
python utils/check_style.py
```

### 自动修复代码风格
```bash
python utils/check_style.py --apply
```

**要求：**
- Python: yapf==0.43.0（基于 Google 风格，配置在 `.style.yapf`）
- C++: clang-format-10（配置在 `.clang-format`）

## 测试

```bash
pytest  # 运行所有测试
```

## 核心架构

### 模块结构

```
dexe_robot_client/
├── common/              # 通用工具和配置
│   ├── config.py       # 数据目录、URL 配置
│   ├── logger.py       # 日志系统
│   ├── status.py       # 状态码和上下文管理
│   ├── robot_type.py   # W1ArmType, W1Version 枚举
│   └── mapping_index.py # 仿真/真实机器人索引映射
├── robot_client/        # 底层机器人客户端
│   ├── robot_client.py # RobotClient 抽象基类
│   └── robot_client_ros2_implement.py # ROS2 实现
├── end_effector_client/ # 末端执行器客户端
│   ├── end_effector_type.py # EndEffectorType 枚举
│   ├── end_effector_factory.py # 工厂模式创建客户端
│   ├── brainco_hand_client.py
│   ├── linker_hand_client.py
│   └── pgc_gripper_client.py
├── navigation_client/   # 底盘导航客户端
│   ├── chassis_client.py
│   └── navigation_client.py
├── w1/                  # W1 人形机器人高级接口
│   ├── w1_manager.py   # 主管理类（单例模式）
│   ├── path_planner/   # 路径规划器
│   │   ├── blend_path.py # Bezier 混合路径
│   │   ├── cartesian_planner.py # 笛卡尔空间规划
│   │   ├── cartesian_to_joint_planner.py
│   │   ├── hybrid_planner.py # 混合规划器
│   │   ├── ompl_plan.py # OMPL 规划
│   │   ├── fcl_checker.py # 碰撞检测
│   │   └── better_fcl_checker.py
│   ├── trajectory_planner/ # 轨迹规划器
│   │   └── toppra_trajectory_planner.py # TOPPRA 时间最优轨迹
│   ├── kinemtaics/     # 运动学
│   │   └── pin_ik_whole_body.py # Pinocchio 全身 IK
│   └── utils/
│       ├── w1_params.py # W1 参数配置
│       └── unit_conversion.py
└── sim_server/          # 仿真服务器
    └── dexsim_server.py
```

### 关键设计模式

**W1Manager（单例模式）**
- 核心入口类，管理机器人、末端执行器、底盘客户端
- 支持仿真/真实模式切换（`set_sim_mode()`）
- 内部维护两套客户端实例（`_*_sim` 和 `_*_real`）
- 使用 ROS2 MultiThreadedExecutor 在独立线程中运行节点

**EndEffectorFactory（工厂模式）**
- 根据 `EndEffectorType` 创建对应的末端执行器客户端
- 支持类型：BRAINCO_HAND, LINKER_HAND, PGC_GRIPPER

**RobotClient（抽象基类）**
- 定义机器人控制的统一接口
- `RobotClientRos2` 实现 ROS2 通信层
- 管理运动组（MotionGroup）、状态字、控制字

### 索引映射系统

由于仿真和真实机器人的关节顺序不同，使用 `mapping_index.py` 进行转换：
- `get_robot_sim2real_idx()` / `get_robot_real2sim_idx()`
- `get_ee_sim2real_idx()` / `get_ee_real2sim_idx()`
- `get_pin2real_idx()` / `get_real2pin_idx()` （Pinocchio 模型索引）

### 规划器层次

1. **路径规划器**：生成无时间参数的路径点序列
   - `CartesianPlanner`: 笛卡尔空间直线插值
   - `CartesianToJointPlanner`: 笛卡尔转关节空间
   - `HybridPlanner`: 结合多种规划策略
   - `PathRnBezier` / `PathR3xSO3Bezier`: Bezier 曲线混合

2. **轨迹规划器**：为路径添加时间参数
   - `ToppraTrajectoryPlanner`: 基于 TOPPRA 的时间最优轨迹

3. **碰撞检测**：
   - `FCLChecker` / `BetterFCLChecker`: 基于 FCL 的碰撞检测

### 运动学求解

- 使用 `dexe_rp` 库（DexforceW1Kinematics）进行正逆运动学
- `pin_ik_whole_body.py`: 基于 Pinocchio 的全身 IK 求解器

## 依赖项

**核心依赖：**
- ROS2 Humble
- dexe_rp >= 0.1.0（从内部源安装）
- OMPL & FCL（通过 `dependencies/install_planner_dependencies.sh` 安装）
- numpy>=1.24.4,<2
- scipy>=1.12
- toppra==0.6.3
- trimesh>=4.2.4

**可选依赖：**
- rlia（强化学习接口）
- dexsim（仿真可视化）

## 分支策略

- `main` / `dev`: 通用机器人功能
- `main_w1` / `dev_w1`: W1 人形机器人专用（需持续合并 `main` 的更新）
- 功能分支命名：`<name>/<type>/<feature>`（如 `zyn/dev/xxx`, `fsh/w1/xxx`）

## Commit 规范

遵循 Angular 提交规范：
- `feat`: 新功能
- `fix`: 修复 bug
- `perf`: 性能优化
- `refactor`: 重构
- `docs`: 文档更新
- `test`: 测试相关
- `build`: 构建系统或依赖项变更
- `ci`: CI 配置变更

## 示例代码

参考 `src/examples/` 目录：
- `w1_manager_example*.py`: W1Manager 使用示例
- `robot_client_example.py`: 底层 RobotClient 示例
- `end_effector_client_example.py`: 末端执行器控制
- `planner/`: 各种规划器示例
- `kinematics/`: 运动学求解示例

## 注意事项

- 所有可调参数必须通过 ROS2 参数系统声明（`declare_parameter`），禁止硬编码
- 使用 `get_logger()` 获取统一的日志记录器
- 状态管理使用 `StatusCode` 和 `StatusContext`
- URDF 文件通过 `download_w1_urdf()` 自动下载到 `DATA_DIR`
- 仿真模式需要先启动 `sim_server` 和 `dexsim_server`
