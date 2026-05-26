# 工具补偿 (重力 + 动力学) 算法与实现

## 1 概述

末端力传感器 (SRI 六维力) 测得的 wrench 中包含三类信号：

1. 真实外力 / 接触力 (我们想要的)
2. 工具自重在 EE 系下的投影 (随姿态变化)
3. 工具运动产生的惯性反作用 (随加速度 / 角速度 / 角加速度变化)

本模块在 `w1_server/src/w1_server.cpp` 的 1 kHz 力矩控制线程中, 顺序扣掉 (2) 和 (3), 让下游 `JointImpedanceController` 经 $J^T$ 投影得到的 `torque_ext` 反映真实外力, 而不是被工具自重 / 惯性污染.

```
raw_force_sensor (传感器原始)
   ↓ - sensor zero offset (零偏标定)
zeroed_sensor_data
   ↓ - ApplyToolGravityCompensation     ← 第 2 节
gravity-compensated wrench
   ↓ + sensor_force_ (force_array 话题手动注入, 可选)
   ↓ - ApplyToolInertiaCompensationPrimary  ← 第 3 节, 仅 primary 臂
sensor_data_primary  → 写入下游 raw_measurement.sensor_wrench
```

补偿仅对 **primary 臂** 做完整管线 (重力 + 惯量), secondary 臂只做重力补偿.

## 2 工具重力补偿

### 2.1 输入参数 (yaml)

`w1_server/robot_config.yaml::tool_gravity_compensation` 块:

| 字段 | 单位 | 含义 | 来源 |
|---|---|---|---|
| `mass_kg` | kg | 工具总质量 $m$ | 实测 / 标定 |
| `com_mm` | mm | COM 在 EE/法兰系下的偏移 ${}^E\!r_C$ | `script/tool_com_estimate.py` |
| `gravity_magnitude` | m/s² | 当地重力加速度 $g$ | 默认 9.79 |
| `model_sign` | $\pm 1$ | 力学约定 (传感器 wrench 是"作用力"还是"反作用力") | 标定时定 |

### 2.2 数据来源

| 量 | 取值 | 坐标系 |
|---|---|---|
| EE 旋转 ${}^W\!R_E$ | `GetEERotationWorldFromEE(arm_index)` 内部基于全身 pinocchio 模型 + 反馈关节角做 FK 取 `oMf[ee_link]` | 全身世界系 (URDF root) ← EE |
| 重力向量 | $(0, 0, -g)$ | 全身世界系 |

注意: `manager_state_` 处于 `Init` / `Error` 时, `GetEERotationWorldFromEE` 返回 `nullopt`, 重力补偿这一周期跳过 (return wrench_ee 原值, applied=false).

### 2.3 算法

预期工具自重在 EE 系下产生的 wrench:

$$
{}^E\!\boldsymbol{F}_g = \mathrm{sign} \cdot \big({}^W\!R_E\big)^T \cdot (m \cdot \boldsymbol{g}_W)
$$

$$
{}^E\!\boldsymbol{\tau}_g = {}^E\!r_C \times {}^E\!\boldsymbol{F}_g
$$

扣减:

$$
\boldsymbol{w}_{ee} \leftarrow \boldsymbol{w}_{ee} - \begin{bmatrix}{}^E\!\boldsymbol{F}_g \\ ^E\!\boldsymbol{\tau}_g\end{bmatrix}
$$

`model_sign` 用于把"工具加在传感器上的力"和"传感器测得的力"两种符号约定对齐.

### 2.4 实现位置

- `RobotServer::LoadToolGravityCompensationConfig` (w1_server.cpp): yaml 解析 + 启用 flag
- `RobotServer::ApplyToolGravityCompensation` (w1_server.cpp): 公式实现, 接受可选 `bool* applied` 输出本周期是否成功扣到
- 调用点: `TorqueControlThreadFunc` 中, 拿到零偏后的 wrench 立即调用

### 2.5 静态验证

机械臂保持任意姿态静止时, 期望 `force_sensor` 字段三轴力 $\le 0.5\,\mathrm{N}$, 三轴力矩 $\le 0.05\,\mathrm{N\cdot m}$. 若某一姿态下出现 ~10 N 偏置, 通常是 `mass_kg` / `com_mm` / `model_sign` 之一标错.

## 3 工具动力学 (惯量) 补偿

仅对 primary 臂启用. 在重力补偿之后串接, 把工具因运动产生的惯性反作用 wrench 进一步从读数中扣掉.

### 3.1 输入参数 (yaml)

`tool_gravity_compensation` 块新增 / 复用字段:

| 字段 | 单位 | 含义 | 来源 |
|---|---|---|---|
| `inertia_com_kg_m2` | kg·m² | 工具惯量张量 ${}^E\!I_C$, **关于 COM, 在 EE 系基下表达** | `script/tool_inertia_estimate.py` |
| `inertia_compensation_filter_hz` | Hz | 末端 / 关节加速度的低通截止 | yaml, 默认 5.0, 合法 [0.5, 100] |
| `mass_kg`, `com_mm`, `model_sign` | — | 同重力补偿块 | 同上 |

启用条件: `mass_kg` 必须 $> \texttt{FloatZeroTol}$ 且 `inertia_com_kg_m2` 是合法 3×3, 否则惯量补偿不启用 (重力补偿不受影响).

### 3.2 坐标系约定 (核心)

| 符号 | 含义 | 基矢量 |
|---|---|---|
| arm-base | primary 臂的根 link 坐标系 | — |
| EE | 末端法兰 / `ee_link` | — |
| ${}^A\!R_E$ | 从 EE 系到 arm-base 系的旋转 | arm-base ← EE |
| ${}^A\!\boldsymbol{v}_{ee}$ | EE 原点速度, 在 arm-base 基下 | arm-base |
| ${}^A\!\boldsymbol{\omega}$ | EE 相对 arm-base 的角速度, 在 arm-base 基下 | arm-base |
| ${}^A\!\boldsymbol{a}_{ee},\ {}^A\!\boldsymbol{\alpha}$ | 由前两者数值微分得到 | arm-base |
| ${}^E\!r_C$ | COM 相对 EE 原点的偏移 | EE |
| ${}^E\!I_C$ | 工具惯量张量, 关于 COM | EE 基 |

**为什么不用 world (全身根)**: W1 左右臂的 base 相对全身躯干有固定旋转, 而 arm-only pinocchio 模型的 "world" 就是 arm-base. 用 arm-only Jacobian 拿到的 twist 天然在 arm-base 系下, 与 arm-only FK 取的 ${}^A\!R_E$ 互相自洽. 用 whole-body 路线则需要全身 Jacobian + ee_link 帧寻址, 同时引入"躯干自由度参与"的额外问题, 暂不在本次范围.

**隐含假设**: arm-base 视为惯性系 (即"躯干静止"). arm-only 模型的 dq 中本就不含躯干自由度, 所以这个假设跟"用 arm-only 模型"是同一件事.

### 3.3 数据来源

| 量 | 取值 | 备注 |
|---|---|---|
| 关节位置 ${}^P q$ (7 维) | `robot_feedback_.joint_feedback[jid].joint_position` | `jid` 来自 `inertia_primary_joint_id_` 缓存 (whole-body 模型 `getJointId(LEFT_J{k})-1`), 首拍解析失败则**永久关闭**惯量补偿 |
| 关节速度 ${}^P \dot{q}$ (7 维) | `joint_feedback[jid].joint_velocity` | 直接来自电机驱动器, 不经过 server 端额外滤波 |
| Jacobian ${}^A J$ (6×7) | `pinocchio::GetJacobiMatrix(primary_arm_model_ptr, q)` | 默认 `LOCAL_WORLD_ALIGNED`, 在 arm-only 模型下即 arm-base 对齐 |
| EE 位姿 ${}^A T_E$ | `pinocchio::GetFK(primary_arm_model_ptr, q)` | 取 `oMf[ee_link]`, 用其旋转部分作为 ${}^A\!R_E$ |
| 控制周期 $\Delta t$ | `torque_ctrl_dt` = 1 ms | 1 kHz 力矩控制线程 |

