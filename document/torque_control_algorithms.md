# dexe_robot_server 力矩控制算法

本文详细说明 dexe_robot_server 中的关节 PD、关节导纳和笛卡尔阻抗控制，以及三类算法与逆动力学前馈、Planner Manager 和 TorqueControlThread 的组合关系。

## 1. 总体控制链路

力矩控制不是由单一控制器完成，而是由 250 Hz 规划层和 1 kHz 力矩层协同计算：

```text
Planner Manager · 250 Hz
    ├── 生成关节位置、速度、刚度、阻尼和积分比例
    └── 可选：计算笛卡尔阻抗关节力矩
                ↓ PlannerManagerCommand Latest Buffer
TorqueControlThread · 1 kHz
    ├── 读取 JointRawMeasurement
    ├── 计算逆动力学前馈
    ├── 估计外力矩并更新关节导纳偏移
    ├── 根据修正后的参考值计算关节 PD 力矩
    └── 生成 TorqueCtrlCommand
```

核心文件：

| 文件 | 作用 |
|---|---|
| [`robot_server_lib/torque_controller/torque_controller_inverse_dynamic.cpp`](../robot_server_lib/torque_controller/torque_controller_inverse_dynamic.cpp) | 串联逆动力学、关节导纳和关节 PD，并组合笛卡尔阻抗结果 |
| [`robot_server_lib/dynamics/joint_pd_controller.cpp`](../robot_server_lib/dynamics/joint_pd_controller.cpp) | 计算关节 PD 与积分力矩 |
| [`robot_server_lib/dynamics/joint_admittance_controller.cpp`](../robot_server_lib/dynamics/joint_admittance_controller.cpp) | 将外力矩转换为位置和速度偏移 |
| [`robot_server_lib/dynamics/cartesian_impedance_controller.cpp`](../robot_server_lib/dynamics/cartesian_impedance_controller.cpp) | 计算任务空间阻抗、惯性整形和零空间力矩 |
| [`robot_server_lib/dynamics/inverse_dynamics.cpp`](../robot_server_lib/dynamics/inverse_dynamics.cpp) | 组织整机 RNEA、双臂符号动力学和工具补偿 |
| [`robot_server_lib/utils/tool_inertia_loader.cpp`](../robot_server_lib/utils/tool_inertia_loader.cpp) | 从手 URDF 合成默认手指姿态下的固定载荷惯性 |
| [`robot_server_lib/dynamics/tool_compensation.cpp`](../robot_server_lib/dynamics/tool_compensation.cpp) | 通过 RNEA 计算工具附加力矩 |
| [`robot_server_lib/src/planner_manager.cpp`](../robot_server_lib/src/planner_manager.cpp) | 在启用笛卡尔阻抗时计算双臂阻抗力矩并写入规划指令 |
| [`robot_server_lib/src/torque_controller_manager.cpp`](../robot_server_lib/src/torque_controller_manager.cpp) | 管理力矩控制状态，读取规划指令并发布 `TorqueCtrlCommand` 与 `JointMeasurement` |

## 2. 关节 PD 控制

### 2.1 输入与公式

`JointPdController` 接收关节位置误差、速度误差和逐关节增益：

```text
e_q  = q_ref  - q_feedback
e_dq = dq_ref - dq_feedback

τ_PD = Kp · e_q + Kv · e_dq + (Kp ⊙ Ki_ratio) · I_q
```

其中：

- `Kp` 来自 `PlannerManagerCommand::joint_stiffness`。
- `Kv` 来自 `PlannerManagerCommand::joint_damping`。
- `Ki_ratio` 来自 `PlannerManagerCommand::joint_integral_ratio`。
- `I_q` 是控制器内部保存的位置误差累加状态。

配置中的 `joint_pd_control.stiffness`、`damping` 和 `integral_ratio` 会加载到 `RobotProperty`，再由 Planner Manager 写入每帧规划指令，因此运行时可以通过 Set Property 更新这些参数。

### 2.2 积分处理

积分项只对位置误差生效：

1. 当某关节位置误差绝对值小于 `PositionThreshold = 1e-6` 时，该关节本周期不累加积分，减少零点附近噪声引起的漂移。
2. `integral_error_` 按控制周期逐样本累加；当前实现没有额外乘以 `dt`。
3. 每个关节的积分状态限制在 `[-0.5, 0.5]`，避免积分饱和。
4. 力矩控制器进入新 episode 时调用 `Reset()`，将积分状态清零。

### 2.3 不同控制状态下的参考值

| Torque Controller 状态 | 位置参考 | 速度参考 | 作用 |
|---|---|---|---|
| `Idle` | 进入 Idle 时记录的 `idle_q_` 加导纳衰减偏移 | 导纳速度偏移 | 保持进入 Idle 时的位置 |
| `Decelerating` | 当前反馈位置加导纳衰减偏移 | 导纳速度偏移 | 位置误差接近零，主要通过阻尼项减速 |
| `Operating` | 规划位置加导纳偏移 | 规划速度加导纳速度偏移 | 跟踪规划目标并根据外力产生柔顺偏移 |

关节控制最终输出为：

```text
τ_joint = τ_inverse_dynamics + τ_PD
```

该结果保存到 `TorqueCtrlCommand::torque`。在 CST 模式下，[`w1_server.cpp`](../w1_server/src/w1_server.cpp) 会将它复制到共享内存的 `joint_torque_feedforward` 下发。

## 3. 关节导纳控制

### 3.1 外力矩估计

在 `Operating` 状态中，控制器使用模型力矩与电机实测力矩之差估计外力矩：

```text
τ_ext = τ_inverse_dynamics - τ_measured
```

符号方向由当前实现定义。每个关节的 `τ_ext` 先经过死区滤波，以减少动力学模型残差和测量噪声导致的偏移：`|τ_ext| <= d` 时输出 0，否则输出 `sign(τ_ext) × (|τ_ext| - d)`。

两条手臂的 J1-J7 可在 `robot_config.yaml` 当前版本配置（如 `w1_v02.joints`）中，逐关节设置 `torque_ext_deadzone`，与 `torque_limit` 同级。它是直接以 N·m 为单位的非负有限数值，不是比率；默认全部为 `0.5`，`0` 表示不扣除死区。例如在 `LEFT_J5` 的配置中写 `torque_ext_deadzone: 0.2`，只改变左臂 J5 的外力矩死区。

手臂范围由 `joints_primary_names` 和 `joints_secondary_names` 决定。缺少该字段或数值非法时使用原来的 `TorqueExtDeadzone = 0.5 N·m`；非手臂关节始终使用原来的固定值，忽略该字段。参数仅在启动时读取，修改运行配置后需重启服务，不支持热更新。正常启动时使用 `~/dexe_robot_server_data/robot_config.yaml`，应在该文件的当前版本关节列表中调节；旧配置缺少字段时仍按 0.5 N·m 工作。

安装新版默认配置后，首次启动会在用户配置中创建这些字段；已有同版本配置会递归补齐 `w1_v02.joints` / `w1_v03.joints` 下缺少的 `torque_ext_deadzone: 0.5`，保留已经填写的值和其他已有参数。本次未提升 `config_version`（仍为 `v06`）；若真机用户配置版本低于默认版本，仍按原有规则整份替换。

`Idle` 和 `Decelerating` 状态向导纳控制器输入零外力矩，使已有位置、速度偏移在虚拟刚度和阻尼作用下逐渐回零。

### 3.2 虚拟二阶系统

导纳控制器把外力矩转换为关节位置和速度偏移。每个启用关节使用单位虚拟质量：

```text
ddq_offset + D · dq_offset + K · q_offset = τ_ext
```

配置使用自然频率 `fn` 和阻尼比 `ζ`，运行时转换为：

```text
ωn = 2π · fn
K  = ωn²
D  = 2 · ζ · ωn
```

因此：

- 提高 `natural_frequency_hz` 会提高虚拟刚度，受到相同外力时偏移更小、回正更快。
- 提高 `damping_ratio` 会增加阻尼，减少振荡，但响应会更迟缓。
- `enabled=false` 的关节不参与导纳计算，其偏移始终清零。

### 3.3 数值积分与限幅

控制器采用半隐式欧拉积分：

```text
dq_offset ← dq_offset + ddq_offset · dt
q_offset  ← q_offset  + dq_offset  · dt
```

每一步都会对状态限幅：

| 配置项 | 限制对象 | 默认回退值 |
|---|---|---:|
| `offset_limit` | `q_offset` | `0.05 rad` |
| `velocity_limit` | `dq_offset` | `0.4 rad/s` |
| `acceleration_limit` | `ddq_offset` | `5.0 rad/s²` |

当位置偏移达到上下限时，继续向饱和方向发展的速度和加速度会被清零，避免积分状态持续向限位外累积。

导纳输出不直接作为力矩下发，而是修正规划参考：

```text
q_ref  = q_cmd  + q_offset
dq_ref = dq_cmd + dq_offset
```

修正后的参考值再进入关节 PD 控制器，因此整体结构属于级联的“外环导纳 + 内环关节阻抗”。

## 4. 笛卡尔阻抗控制

### 4.1 执行位置

笛卡尔阻抗控制由 Planner Manager 以 250 Hz 分别对主臂和从臂计算。只有满足以下条件时才执行：

- 非仿真模式；
- `enable_cartesian_impedance=true`；
- 对应手臂存在已辨识的动力学参数。

计算结果写入 `PlannerManagerCommand::torque_feedforward`。TorqueControlThread 将该字段视为笛卡尔阻抗力矩，并与 1 kHz 逆动力学前馈组合：

```text
τ_cart_output = τ_cartesian_impedance + τ_inverse_dynamics
```

结果保存到 `TorqueCtrlCommand::torque_impedance`，在 `CartesianImpedance` 运行模式下通过共享内存的 `joint_torque_feedforward` 下发。

### 4.2 位姿与速度误差

控制器分别计算期望和反馈末端位姿：

```text
T_feedback = FK(q_feedback)
T_command  = FK(q_command)
```

平移误差为 `p_command - p_feedback`。旋转误差使用四元数最短路径计算，再转换为反馈末端坐标方向下的轴角向量。平移和旋转误差均限制在 `±0.2`，防止目标突变产生过大控制量。

速度由雅可比矩阵映射：

```text
v_feedback = J(q_feedback) · dq_feedback
v_command  = J(q_command)  · dq_command
e_v        = v_command - v_feedback
```

### 4.3 任务空间弹簧阻尼

惯性整形前的六维阻抗力为：

```text
F₀,translation = K · e_p + D · e_v + Ki · ∑e_p
F₀,rotation    = K · e_R + D · e_ω
```

当前只有平移误差包含积分项，且积分状态带有限幅。阻尼根据虚拟质量、刚度和阻尼比自动计算：

```text
D = damping_ratio · critical_damping_ratio · √(M_virtual ⊙ K)
```

其中 `critical_damping_ratio_` 当前为 `1.5`。`cart_stiffness` 和 `cart_damping_ratio` 可通过配置或 Set Property 更新。

### 4.4 惯性整形与关节力矩

控制器先把阻抗力转换为期望任务空间加速度：

```text
a_task = M_virtual⁻¹ · F₀
```

再使用符号动力学质量矩阵构造笛卡尔空间质量矩阵：

```text
Λ = (J · M(q)⁻¹ · Jᵀ)⁺
F = Λ · (a_task - J̇ · dq)
τ_task = Jᵀ · F
```

上标 `+` 表示伪逆。该惯性整形使笛卡尔响应考虑机器人当前构型下的质量分布。

### 4.5 零空间控制与力矩限制

对于冗余自由度，控制器使用：

```text
N = I - Jᵀ · (Jᵀ)⁺
τ_null = N · (K_null · e_q + D_null · e_dq)
```

最终目标力矩为：

```text
τ_target = τ_task + τ_null
```

输出前执行两级限制：

1. 相对上一周期的单步力矩变化限制为 `±1.0`。
2. 最终关节力矩绝对值限制为 `±25.0 N·m`。

限幅后的结果作为笛卡尔阻抗关节力矩返回 Planner Manager。

## 5. 三类算法的组合关系

| 输出路径 | 导纳 | 关节 PD | 笛卡尔阻抗 | 逆动力学 |
|---|---|---|---|---|
| `TorqueCtrlCommand::torque` | 修正关节参考 | 生成关节反馈力矩 | 不使用 | 相加 |
| `TorqueCtrlCommand::torque_impedance` | 不参与该输出 | 不参与该输出 | 由 Planner Manager 生成 | 相加 |
| `TorqueCtrlCommand::torque_feedforward` | 不参与 | 不参与 | 不参与 | 仅逆动力学前馈 |

运行模式决定 [`w1_server.cpp`](../w1_server/src/w1_server.cpp) 最终选择哪个内部结果复制到共享内存的 `joint_torque_feedforward`。共享内存字段名虽然是 `joint_torque_feedforward`，但在 CST 模式下它承载的是最终生效的关节力矩命令。

### 5.1 工具载荷 RNEA 前馈

手臂辨识时未装手，双臂符号模型计算后再叠加工具载荷力矩：

```text
τ_arm = τ_identified + τ_tool
τ_tool = RNEA(payload_model, q_arm, dq_arm, ddq_arm)
```

`payload_model` 保留手臂七轴运动链，将手臂本体惯性置零，仅在末端加入工具惯性。工具重力、惯性和速度相关项由一次 RNEA 计算，原有雅可比转置工具重力补偿已由此路径替换。模型使用臂基座坐标系下的重力方向，计算范围为固定基座下工具对本臂七轴的附加力矩，不包含躯干运动造成的基座惯性或工具对躯干关节的负载传递。

参数来自 `tool_dynamics`，左右臂由 `tool_primary_enabled` / `tool_secondary_enabled` 独立控制。根据 `left_hand_type` / `right_hand_type` 选择手型，对应的 `left_urdf` / `right_urdf` 路径优先于质量、质心字段。读取手 URDF 后，以默认关节姿态固定手指，包含 mimic 关节的偏移，并将各 link 的惯性合成到手的根坐标系后连接到 EE。当前这条补偿路径不随实时手指姿态更新。

没有 URDF 路径的工具继续读取 `mass_kg`、`com_m`，在 EE 系下构造点质量载荷，不附加绕质心的旋转惯量。工具配置可通过 `./set_w1.sh tool` 编辑，`./set_w1.sh tool --list` 查看，修改后重启服务生效。

## 6. 参数调整与验证

调整控制参数时应遵循：

1. 保存当前配置和回滚值。
2. 先在仿真或离线测试中检查符号、维度、限幅和状态复位。
3. 真机从低刚度、低速度和单臂开始。
4. 先验证逆动力学前馈，再单独启用 PD、导纳或笛卡尔阻抗。
5. 观察力矩峰值、偏移限幅、速度换向、接触响应和解除外力后的回零过程。
6. 参数异常、振荡或持续偏移时立即 QuickStop，并恢复原配置。

需要重点避免：

- 在动力学补偿误差较大时直接减小导纳死区；
- 同时大幅提高刚度并降低阻尼；
- 未验证质量矩阵和机器人版本时启用笛卡尔阻抗；
- 将 CSP 下的力矩前馈误认为最终闭环力矩；
- 忽略控制器进入新 episode 时积分和导纳状态会被重置。
