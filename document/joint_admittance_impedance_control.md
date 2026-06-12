# 含导纳环的关节阻抗控制算法设计文档

## 1 概述

本项目在力矩控制层实现了一种**关节空间导纳-阻抗串联控制**（Admittance-in-Impedance Control）架构，用于在力控模式下实现柔顺交互。整个系统由三个核心组件串联而成：

1. **外力估计**（`JointImpedanceController::ComputeExternalTorque`）——通过力矩传感器读数与雅可比转置映射，将末端六维力/力矩转换为关节空间外力矩 $\boldsymbol{\tau}_{\text{ext}}$。
2. **导纳环**（`JointAdmittanceController`）——以 $\boldsymbol{\tau}_{\text{ext}}$ 为输入，模拟一个虚拟质量-弹簧-阻尼系统，输出关节位置修正量 $\boldsymbol{q}_{\text{offset}}$ 及其导数 $\dot{\boldsymbol{q}}_{\text{offset}}$。
3. **阻抗环**（`JointImpedanceController::UpdateTorqueCommandWithoutExternalTorque`）——以修正后的参考位置（$\boldsymbol{q}_{\text{cmd}} + \boldsymbol{q}_{\text{offset}}$）与实际反馈计算跟踪误差，输出刚度-阻尼-积分力矩。
4. **逆动力学前馈**（`dynamics::InverseDynamics`）——补偿重力、科氏力与摩擦力，构成最终的力矩输出。

上述组件在 `TorqueControllerInvDynamic::ComputeCommand` 中被编排为一个完整的控制周期。

### 1.1 为什么使用导纳-阻抗串联而非纯阻抗控制

纯阻抗控制直接将外力矩前馈叠加到力矩输出上（$\boldsymbol{\tau} = K_p \boldsymbol{e} + K_v \dot{\boldsymbol{e}} + \boldsymbol{\tau}_{\text{ext}}$），存在以下局限：

- **刚度与柔顺性耦合**：高刚度保证跟踪精度，但外力直接叠加到力矩上会产生过大的反向力，难以兼顾精度与安全。
- **缺乏动态滤波**：传感器噪声和高频外力直接注入力矩环路。

导纳环在外力与阻抗控制器之间引入一个虚拟动力学系统，对外力进行**低通滤波并转换成位置偏移**，使机器人在受到外力时表现为"虚拟地变软"——参考位置主动退让，而非在力矩层硬扛。这实现了刚度与柔顺性的解耦：阻抗环保持高刚度跟踪精度，导纳环独立调节交互柔顺性。

### 1.2 控制框图

```
                 ┌─────────────────┐
 力矩传感器 ───→ │  外力矩估计      │ ─τ_ext───────────────────────┐
                 │  J^T · wrench    │                               │
                 └─────────────────┘                               ▼
                                                          ┌──────────────────┐
                                                          │   导纳环          │
                                                          │  M·ddq+B·dq+K·q  │
                                                          │   = torque_gain   │
                                                          │     · τ_ext       │
                                                          └──────┬───────────┘
                                                                 │ q_offset, dq_offset
                                                                 ▼
 规划指令 ──→ q_cmd ──(+)──→ q_cmd_admittance ──→ ┌──────────────────────────┐
                                                 │     阻抗环                 │
                                                 │  τ = K_p·(q_cmd_adm - q)  │
                                                 │    + K_v·(dq_cmd_adm - dq)│
                                                 │    + K_i·K_p·∫e dt        │
                                                 └──────────┬─────────────────┘
                                                            │ τ_impedance
                                                            ▼
                                              ┌──────────────────────────┐
                                              │  逆动力学前馈             │
                                              │  τ_ff = RNEA(q,dq,0)     │
                                              └──────────┬─────────────────┘
                                                         │
                                                         ▼
                                          τ_total = τ_impedance + τ_ff
```

## 2 导纳环

**源文件**：`robot_server_lib/dynamics/joint_admittance_controller.{h,cpp}`

### 2.1 动力学方程

导纳环为每个关节模拟一个独立的二阶虚拟系统。对于关节 $i$，其动力学方程为：

$$
M_i \ddot{q}_{\text{offset},i} + B_i \dot{q}_{\text{offset},i} + K_i q_{\text{offset},i} = g_i \cdot \tau_{\text{ext},i}
$$

其中：

| 符号 | 含义 | 配置字段 |
|------|------|----------|
| $M_i$ | 虚拟惯量 | `virtual_inertia`（默认 1.0） |
| $g_i$ | 力矩增益 | `torque_gain`（默认 1.0） |
| $\tau_{\text{ext},i}$ | 关节外力矩（由雅可比转置计算） | 运行时输入 |
| $q_{\text{offset},i}$ | 位置修正量（输出） | `q_offset_` |
| $K_i$ | 虚拟刚度 | 由固有频率导出 |
| $B_i$ | 虚拟阻尼 | 由阻尼比导出 |

### 2.2 刚度与阻尼的参数化

本实现不直接配置 $K$ 和 $B$，而是通过**固有频率** $\omega_n$ 和**阻尼比** $\zeta$ 间接推导，物理意义更直观：

$$
\omega_{n,i} = 2\pi f_{n,i}, \quad f_{n,i} = \text{natural\_frequency\_hz} \ (\text{默认}\ 1.5\text{Hz})
$$

$$
K_i = M_i \omega_{n,i}^2
$$

$$
B_i = 2\zeta_i M_i \omega_{n,i}, \quad \zeta_i = \text{damping\_ratio} \ (\text{默认}\ 1.2)
$$

对应代码（`joint_admittance_controller.cpp:99-105`）：

```cpp
const double omega_n = 2.0 * M_PI * natural_frequency_hz_[i];
stiffness_[i] = virtual_inertia_[i] * omega_n * omega_n;
damping_[i] = 2.0 * damping_ratio_[i] * virtual_inertia_[i] * omega_n;
```

默认 $\zeta = 1.2 > 1$，使导纳系统为**过阻尼**，避免外力突变时产生位置偏移振荡。

### 2.3 数值积分

采用显式欧拉法进行数值积分（`joint_admittance_controller.cpp:84-91`）：

$$
\ddot{q}_{\text{offset},i}^{[k]} = \frac{g_i \cdot \tau_{\text{ext},i} - B_i \dot{q}_{\text{offset},i}^{[k]} - K_i q_{\text{offset},i}^{[k]}}{M_i}
$$

$$
\dot{q}_{\text{offset},i}^{[k+1]} = \dot{q}_{\text{offset},i}^{[k]} + \ddot{q}_{\text{offset},i}^{[k]} \cdot dt
$$

$$
q_{\text{offset},i}^{[k+1]} = q_{\text{offset},i}^{[k]} + \dot{q}_{\text{offset},i}^{[k+1]} \cdot dt
$$

在每次积分步之后进行限幅，防止发散。

### 2.4 状态限幅与抗饱和

导纳环对输出的三个状态量施加严格的限幅（`ClampState()`，`joint_admittance_controller.cpp:107-131`）：

| 状态 | 限幅字段 | 默认值 |
|------|----------|--------|
| $q_{\text{offset}}$ | `offset_limit` | 0.05 rad（约 2.9°） |
| $\dot{q}_{\text{offset}}$ | `velocity_limit` | 0.4 rad/s |
| $\ddot{q}_{\text{offset}}$ | `acceleration_limit` | 5.0 rad/s² |

当 $q_{\text{offset}}$ 到达限幅边界时，清除继续向外积分的速度和加速度（积分抗饱和），避免饱和后偏移量在边界处持续积累：

```cpp
if (q_offset_[i] >= offset_limit_[i]) {
    if (dq_offset_[i] > 0.0) dq_offset_[i] = 0.0;       // 清除外向速度
    if (dq_offset_[i] >= 0.0 && ddq_offset_[i] > 0.0)
        ddq_offset_[i] = 0.0;                             // 清除外向加速度
}
```

### 2.5 衰减模式（DecayToZero）

当导纳环不需要激活（如 Idle、Decelerating 状态或无力矩传感器时），调用 `DecayToZero(dt)` 而非 `Update()`。该方法以零外力矩驱动虚拟系统，利用阻尼项使 $q_{\text{offset}}$ 自然衰减至零，而非立即置零，避免位置参考突变。

### 2.6 配置鲁棒性

所有配置参数经过 `PositiveOrDefault` / `NonNegativeOrDefault` 校验，对非有限值（NaN/Inf）和非正值回退到安全默认值，保证虚拟系统不会因配置错误而发散。

### 2.7 逐关节使能

`JointAdmittanceConfig.joints[i].enabled` 可逐关节开关导纳环。未使能的关节其偏移量被清零，不参与导纳计算。全局 `JointAdmittanceConfig.enabled` 控制整个导纳环的总开关。

## 3 阻抗环

**源文件**：`robot_server_lib/dynamics/joint_impedance_controller.{h,cpp}`

### 3.1 阻抗控制律

在含导纳环的配置中，使用 `UpdateTorqueCommandWithoutExternalTorque` 变体（外力矩已由导纳环处理，不在此重复叠加）：

$$
\boldsymbol{\tau}_{\text{impedance}} = \text{diag}(\boldsymbol{K}_p) \cdot \boldsymbol{e}_q + \text{diag}(\boldsymbol{K}_v) \cdot \boldsymbol{e}_{\dot{q}} + \text{diag}(\boldsymbol{K}_p \cdot K_i) \cdot \int \boldsymbol{e}_q \, dt
$$

其中跟踪误差基于导纳修正后的参考位置：

$$
\boldsymbol{e}_q = \boldsymbol{q}_{\text{cmd}} + \boldsymbol{q}_{\text{offset}} - \boldsymbol{q}_{\text{feedback}}
$$

$$
\boldsymbol{e}_{\dot{q}} = \dot{\boldsymbol{q}}_{\text{cmd}} + \dot{\boldsymbol{q}}_{\text{offset}} - \dot{\boldsymbol{q}}_{\text{feedback}}
$$

对应代码（`joint_impedance_controller.cpp:59-75`）：

```cpp
VectorDof q_error = cmd_q - feedback_q;       // cmd_q 已含 q_offset
const VectorDof dq_error = cmd_dq - feedback_dq;
// ... 小误差死区处理 ...
tau_command = stiffness.asDiagonal() * q_error
            + damping.asDiagonal() * dq_error
            + torque_integral;
```

### 3.2 死区

当位置误差小于 $0.005°$（$0.005/180 \times \pi$ rad）时置零，消除传感器分辨率噪声引起的微小力矩抖动。

### 3.3 积分项与条件激活

积分项用于消除稳态误差，但不当激活会导致积分饱和、超调甚至不稳定。本实现引入了**多条件门控**机制（`torque_controller_inverse_dynamic.cpp:97-108`），仅当以下条件同时满足时才允许积分：

| 条件 | 阈值 | 物理含义 |
|------|------|----------|
| 外力矩收敛 | $\|\tau_{\text{ext},i}\| < 0.2$ Nm | 外部干扰已消除 |
| 导纳偏移收敛 | $\|q_{\text{offset},i}\| < 0.003$ rad | 导纳环已稳定 |
| 导纳速度收敛 | $\|\dot{q}_{\text{offset},i}\| < 0.03$ rad/s | 导纳环无动态运动 |
| 指令速度收敛 | $\|\dot{q}_{\text{cmd\_adm},i}\| < 0.03$ rad/s | 规划器已停稳 |
| 反馈速度收敛 | $\|\dot{q}_{\text{fb},i}\| < 0.05$ rad/s | 机器人已停稳 |

积分误差本身也受到限幅（$\pm 0.5$ rad·s），防止积分饱和。

## 4 外力矩估计

**源文件**：`joint_impedance_controller.cpp:83-129`

### 4.1 雅可比转置法

末端力矩传感器测得六维力旋 $\boldsymbol{w} \in \mathbb{R}^6$（力 + 力矩），通过雅可比转置映射到关节空间：

$$
\boldsymbol{\tau}_{\text{ext}} = \boldsymbol{J}^T(\boldsymbol{q}) \cdot \boldsymbol{w}
$$

对于双臂机器人，左右臂分别独立计算，再映射回全身关节向量。

### 4.2 力旋处理

原始传感器读数经过以下处理后才进行雅可比映射（`ProcessExternalWrench`，`joint_impedance_controller.cpp:110-129`）：

1. **死区**：力/力矩分量分别配置死区（`external_wrench_force_deadband_n`、`external_wrench_torque_deadband_nm`），消除传感器零漂。
2. **饱和**：限幅到最大值（`external_wrench_force_saturation_n`、`external_wrench_torque_saturation_nm`），防止异常大力导致关节力矩溢出。
3. **符号**：乘以 `external_wrench_model_sign`（$\pm 1$），适配传感器安装方向。

公式为：

$$
w_{\text{proc}} = \text{sign} \cdot \text{sat}(\text{deadband}(w_{\text{raw}}))
$$

## 5 集成：TorqueControllerInvDynamic

**源文件**：`robot_server_lib/torque_controller/torque_controller_inverse_dynamic.{h,cpp}`

`TorqueControllerInvDynamic` 是力矩控制线程的核心，持有阻抗控制器和导纳控制器的共享指针，在 `ComputeCommand` 中按状态机分派。

### 5.1 Operating 状态（正常运行）完整流程

```
                    力矩传感器 wrench
                         │
                         ▼
              ┌──────────────────────┐
              │ ComputeExternalTorque │  ← J^T · ProcessedWrench
              └─────────┬────────────┘
                        │ τ_ext_raw
           ┌────────────┼────────────────────┐
           │            │                    │
           ▼            ▼                    ▼
     τ_ext 输出   导纳环 Update        τ_ext_direct_gain
     (状态反馈)   → q_offset, dq_offset   直通增益叠加
                        │
                        ▼
              q_cmd_admittance = q_cmd + q_offset
              dq_cmd_admittance = dq_cmd + dq_offset
                        │
                        ▼
              ┌────────────────────────┐
              │ UpdateTorqueCommand     │  ← K_p·e + K_v·ė + K_i·∫e
              │ WithoutExternalTorque  │
              └─────────┬──────────────┘
                        │ τ_joint_impedance
                        ▼
              τ_impedance += torque_ext_direct_gain · τ_ext   (可选)
                        │
                        ▼
              τ_ff = InverseDynamics(q_fb, dq_fb, 0)          ← 重力+科氏+摩擦补偿
                        │
                        ▼
              τ_total = τ_joint_impedance + τ_ff
```

### 5.2 外力矩直通增益

当导纳环使能时，外力矩不直接叠加到阻抗力矩上，而是仅通过导纳环间接影响参考位置。但配置中提供了 `torque_ext_direct_gain` 参数，允许外力矩以可调增益部分直通到力矩输出（`torque_controller_inverse_dynamic.cpp:116-123`）：

- **`joint_admittance.enabled = true`**：$\boldsymbol{\tau} \mathrel{+}= \text{torque\_ext\_direct\_gain} \cdot \boldsymbol{\tau}_{\text{ext}}$（默认增益为 0，纯导纳模式）
- **`joint_admittance.enabled = false`**：$\boldsymbol{\tau} \mathrel{+}= \boldsymbol{\tau}_{\text{ext}}$（增益为 1，退化为纯阻抗+前馈外力模式）

### 5.3 Idle / Decelerating 状态

在这两种非运行状态下：

- 阻抗环以当前位置（Idle）或零速度目标（Decelerating）产生保持力矩。
- 导纳环调用 `DecayToZero`，使偏移量平滑衰减至零。
- 外力矩仍被计算并输出到状态反馈（`output_cmd.torque_ext`），供上层监控。

### 5.4 最终力矩输出

$$
\boldsymbol{\tau}_{\text{total}} = \underbrace{\boldsymbol{\tau}_{\text{impedance}}}_{\text{刚度+阻尼+积分+直通}} + \underbrace{\text{RNEA}(\boldsymbol{q}, \dot{\boldsymbol{q}}, \boldsymbol{0})}_{\text{重力+科氏+摩擦前馈}}
$$

逆动力学前馈使用递归牛顿-欧拉算法（RNEA），在当前关节状态下计算使机器人保持静止所需的力矩，补偿重力、科氏离心力及摩擦力。阻抗力矩只需要补偿偏差部分，大幅降低了阻抗增益的需求。

## 6 配置参数

### 6.1 导纳环配置（`JointAdmittanceConfig`）

定义在 `robot_server_lib/utils/data_structure.h:103-118`，在 `RobotProperty.joint_admittance` 中配置。

**全局参数**：

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `enabled` | bool | false | 导纳环总开关 |
| `torque_ext_direct_gain` | double | 0.0 | 外力矩直通增益（导纳使能时） |

**逐关节参数**（`JointAdmittanceJointConfig`）：

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `enabled` | bool | false | 该关节导纳环开关 |
| `torque_gain` | double | 1.0 | 外力矩输入增益 |
| `virtual_inertia` | double | 1.0 | 虚拟惯量 $M$ |
| `natural_frequency_hz` | double | 1.5 | 固有频率 $f_n$（Hz） |
| `damping_ratio` | double | 1.2 | 阻尼比 $\zeta$ |
| `offset_limit` | double | 0.05 | 位置偏移限幅（rad） |
| `velocity_limit` | double | 0.4 | 速度限幅（rad/s） |
| `acceleration_limit` | double | 5.0 | 加速度限幅（rad/s²） |

### 6.2 阻抗环配置（`RobotProperty`）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `joint_stiffness` | 200.0 | 关节刚度 $K_p$（N·m/rad） |
| `joint_damping` | 20.0 | 关节阻尼 $K_v$（N·m·s/rad） |
| `joint_integral_ratio` | 0.01 | 积分增益比例 $K_i$ |

### 6.3 调参指南

- **增大 `natural_frequency_hz`**：导纳环响应更快，但接近机器人结构频率时可能引起共振。
- **增大 `damping_ratio`**：更平稳无超调，但响应变慢。$\zeta > 1$ 为过阻尼（推荐），$\zeta = 1$ 为临界阻尼，$\zeta < 1$ 为欠阻尼（有振荡）。
- **增大 `offset_limit`**：允许更大的柔顺退让空间，但会降低在极端外力下的位置保持能力。
- **`torque_gain`**：调节外力矩对导纳环的激励强度。大于 1 提高对弱力的灵敏度，小于 1 降低灵敏度。
- **`torque_ext_direct_gain`**：在纯导纳模式（0.0）和纯阻抗模式（1.0）之间连续调节。中间值可兼顾导纳的平滑性与阻抗的快速响应。

## 7 文件索引

| 文件 | 职责 |
|------|------|
| `robot_server_lib/dynamics/joint_admittance_controller.{h,cpp}` | 导纳环：虚拟质量-弹簧-阻尼系统 |
| `robot_server_lib/dynamics/joint_impedance_controller.{h,cpp}` | 阻抗环：刚度-阻尼-积分控制 + 外力矩估计 |
| `robot_server_lib/torque_controller/torque_controller_inverse_dynamic.{h,cpp}` | 集成层：状态机分派，编排导纳+阻抗+逆动力学 |
| `robot_server_lib/dynamics/inverse_dynamics.{h,cpp}` | 逆动力学前馈（RNEA） |
| `robot_server_lib/utils/data_structure.h` | 配置结构体定义（`JointAdmittanceConfig` 等） |
| `robot_server_lib/test/test_joint_admittance_controller.cpp` | 导纳环单元测试 |
