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

### 3.4 运动学量

每周期算:

$$
\begin{bmatrix} {}^A\!\boldsymbol{v}_{ee} \\ {}^A\!\boldsymbol{\omega} \end{bmatrix}
= {}^A J \cdot {}^P\!\dot{q}
$$

线加速度: ${}^A\!\boldsymbol{v}_{ee}$ 后向差分再做二阶临界阻尼 IIR (`ee_accel_filter_`):

$$
{}^A\!\boldsymbol{a}_{ee, k} = \mathrm{LPF}\!\left(\frac{{}^A\!\boldsymbol{v}_{ee, k} - {}^A\!\boldsymbol{v}_{ee, k-1}}{\Delta t}\right)
$$

角加速度: 直接对 ${}^A\!\boldsymbol{\omega}$ 后向差分再 LPF (`omega_diff_filter_`):

$$
{}^A\!\boldsymbol{\alpha}_{k} = \mathrm{LPF}\!\left(\frac{{}^A\!\boldsymbol{\omega}_{k} - {}^A\!\boldsymbol{\omega}_{k-1}}{\Delta t}\right)
$$

注意: 早期实现曾采用 $\boldsymbol{\alpha} = J_\omega \cdot \ddot{q}$ (其中 $\ddot{q}$ 是 $\dot{q}$ 微分滤波), 这种写法**少了 $\dot{J}_\omega \cdot \dot{q}$ 项** — 当 $\dot{q}$ 不为零且姿态变化时, 会引入与 $\dot{J}_\omega \cdot \dot{q}$ 同量级的偏差. 改为对 $\boldsymbol{\omega}$ 直接微分后, 该修正项隐含在 $\boldsymbol{\omega}$ 自身的时间导数里, 数值上与 $J\ddot{q} + \dot{J}\dot{q}$ 一致, 且与线加速度路径滤波延迟对称. 滤波器从 7 路 (joint_accel) 改为 3 路 (omega).

工具 COM 在 arm-base 系下的位置偏移与惯量旋转:

$$
{}^A\!r_C = {}^A\!R_E \cdot {}^E\!r_C, \qquad
{}^A I_C = {}^A\!R_E \cdot {}^E I_C \cdot \big({}^A\!R_E\big)^T
$$

工具 COM 加速度 (含切向 + 向心 + 平动):

$$
{}^A\!\boldsymbol{a}_C = {}^A\!\boldsymbol{a}_{ee} + {}^A\!\boldsymbol{\alpha} \times {}^A\!r_C + {}^A\!\boldsymbol{\omega} \times \big({}^A\!\boldsymbol{\omega} \times {}^A\!r_C\big)
$$

驱动工具运动所需的合力 / 合力矩 (在 arm-base 系, 力矩取在 EE 原点):

$$
{}^A\!\boldsymbol{F} = m \cdot {}^A\!\boldsymbol{a}_C
$$

$$
{}^A\!\boldsymbol{\tau}_{\!E} = {}^A I_C \cdot {}^A\!\boldsymbol{\alpha} + {}^A\!\boldsymbol{\omega} \times \big({}^A I_C \cdot {}^A\!\boldsymbol{\omega}\big) + {}^A\!r_C \times {}^A\!\boldsymbol{F}
$$

### 3.5 符号约定 (核心)

这一节是 **2026-05-26 修复后的新约定**, 对应 `w1_server.cpp::ApplyToolInertiaCompensationPrimary` 中 `expected_inertia_wrench_ee` 的语义. 历史符号 bug 见第 6 节.

#### 3.5.1 离线辨识脚本的物理模型

`script/tool_inertia_estimate.py:379` 给出 (无外力时) 法兰处的传感器读数模型:

$$
\boldsymbol{F}_{\mathrm{meas}} \;\approx\; m \cdot {}^E\!R_W^{\!T}\, \boldsymbol{g}_W \;-\; m \cdot {}^E\!\boldsymbol{a}_C \;+\; \mathrm{bias}_F
$$

$$
\boldsymbol{M}_{\mathrm{meas}} \;\approx\; -\big({}^E I_C \,{}^E\!\boldsymbol{\alpha} + {}^E\!\boldsymbol{\omega} \times {}^E I_C \,{}^E\!\boldsymbol{\omega}\big) \;+\; {}^E\!r_C \times \boldsymbol{F}_{\mathrm{unbiased}} \;+\; \mathrm{bias}_T
$$

物理解读: 加速工具需要 `+m·a_C` 的合力; 由牛顿第三定律, 工具反作用到传感器上的就是 `-m·a_C`. 重力项符号为 `+m·g` (静止时传感器测得 = +mg), 惯性项符号为 `-`.

#### 3.5.2 C++ 中 expected_inertia_wrench_ee 的定义

定义为 **"传感器读数中由工具惯性产生的那一份分量"** (与 `ApplyToolGravityCompensation` 的 `expected_wrench_ee` 同视角):

$$
\boldsymbol{w}_{\mathrm{exp,inertia}}^E \;=\; \mathrm{sign} \cdot {}^E\!R_A \cdot \begin{bmatrix} -{}^A\!\boldsymbol{F} \\ -{}^A\!\boldsymbol{\tau}_{\!E} \end{bmatrix}
$$

负号来源就是上式中的牛顿第三定律. 然后扣减:

$$
\boldsymbol{w}_{ee} \leftarrow \boldsymbol{w}_{ee} - \boldsymbol{w}_{\mathrm{exp,inertia}}^E
$$

数学上等价于 $\boldsymbol{w}_{ee} \leftarrow \boldsymbol{w}_{ee} + \mathrm{sign}\cdot {}^E\!R_A \cdot [{}^A\!\boldsymbol{F};\,{}^A\!\boldsymbol{\tau}_{\!E}]$, 即把"工具被加速所需的合力"加回 wrench, 抵消掉传感器中工具反作用造成的负号读数. 实际代码保持减法以与重力补偿同结构.

#### 3.5.3 为什么 force_sensor_inertia 字段是 -[F; τ] 而不是 +[F; τ]

发布到 `/feedback/robot_server_state.force_sensor_inertia` 的就是 `expected_inertia_wrench_ee`. 这个值与 `force_sensor` / `raw_force_sensor` **同视角同符号**:
- 工具加速向 +z → COM 加速度向 +z → 工具反作用拉传感器向 -z → `force_sensor_inertia.z < 0` ✓
- 与 `force_sensor` 在没有外力时应该呈现的工具惯性残差**符号一致**, 上层做差就是真实外力.

### 3.6 实现位置

- `RobotServer::ApplyToolInertiaCompensationPrimary` (w1_server.cpp): 公式 + 滤波器更新, 接受可选 `bool* applied`
- `RobotServer::ResetToolInertiaCompensationState`: 清滤波器历史 + prev 状态 + 诊断字段置零
- 调用点: `TorqueControlThreadFunc` 中, 重力补偿 + force_array 注入 + 反馈排序之后, 写下游测量之前. 用 `should_run_inertia` 单一布尔门控所有前置条件, 任一不满足走 Reset 路径.

### 3.7 验证

实测数据 (`script/inertia_analyse/20260526_164600.json`, 无外力 2289 样本):

| 指标 | RMS (N + N·m, 6 维范数) |
|---|---|
| 重力补偿后, 惯量补偿前 | 0.8652 |
| 修复后惯量补偿后 | **0.7293** (↓ 15.7%) |
| 修复前惯量补偿后 (符号反) | 1.0889 (↑ 25.9%) |

修复后 RMS 仍未降到 0, 说明除符号外还有其它残差源 (α 估计的 $\dot{J}\dot{q}$ 项已经修了, 剩下可能是: 编码器速度量化 / 滤波相位延迟与真实运动不完全对齐 / `inertia_com_kg_m2` 标定残差等). 这些是后续 cutoff 调优 + 标定改进的问题, 不再属于符号性 bug.

离线 sanity check: `script/inertia_analyse/check_sign_fix.py` 可重放任意采集 JSON 验证当前符号约定下补偿 RMS 是否下降.

## 4 失效安全策略

补偿"启用了但本周期没成功扣到" 视为失效, 对应臂的 `sensor_data` 整体置零, 让下游 `JointImpedanceController::ComputeExternalTorque` 的 `isZero` 短路使 `torque_ext = 0`. 失效路径包括:

- `manager_state_ ∈ {Init, Error}` 导致 ${}^W\!R_E$ 不可用 (重力补偿失效)
- 反馈失效 / `SortFeedbackByJointName` 失败 (惯量补偿失效)
- 首拍 / Reset 后第一拍 (惯量补偿主动跳过)
- 防御性早退 (size 不匹配 / Jacobian shape 异常)

未启用 (用户没配 `inertia_com_kg_m2`, 或 secondary 无传感器) **不视为失效**, 不触发清零.

代码位置: `TorqueControlThreadFunc` 中 `primary_compensation_failed` / `secondary_compensation_failed` 判定, throttled WARN 日志 1 Hz.

效果: 力矩模式下任一补偿失效都不会把"工具自重 / 惯性反作用"误当外力反馈进控制律. 位置模式不读 sensor_wrench, 失效与否无影响.

## 5 运行时诊断

发布到 `/feedback/robot_server_state` JSON:

| 字段 | 单位 | 含义 |
|---|---|---|
| `force_sensor` (12 维) | N / N·m | primary[0:6] + secondary[6:12], 经过零偏 + 重力补偿 + 惯量补偿 + 失效清零的最终值. **力矩模式下消费的就是这个**. |
| `force_sensor_gravity` (12 维) | N / N·m | primary[0:6] + secondary[6:12], 经过零偏 + 工具重力补偿后、惯量补偿前的值. 不包含 `force_array` 手动注入力. |
| `raw_force_sensor` (12 维) | N / N·m | 完全原始, 连零偏都没扣 |
| `force_sensor_bias` (12 维) | N / N·m | 零偏标定值 (EE 系) |
| `force_sensor_inertia` (6 维) | N / N·m | primary 惯量补偿这一周期实际从 wrench 中扣减的量 (= `expected_inertia_wrench_ee`). 静止 ≈ 0; 失效 / 首拍显式置零. 与 `force_sensor` 同视角同符号. |
| `tool_inertia_filter_cutoff_hz` | Hz | 当前生效的 LPF 截止, 由 yaml 加载 |

监控工具:
- `./set_w1.sh force` 每 100 ms 打印上述字段
- `script/plot_force_sensor.py --force_sensor --force_sensor_inertia` 实时 6 分量曲线对比

## 6 历史: 2026-05-26 符号 bug 复盘

**现象**: 实机定量分析显示重力补偿后 RMS 0.8652, 惯量补偿后 RMS 1.0889 (反而升高), 对模型 `after = before − k·inertia` 拟合得 `k ≈ -1.48`. 符号方向反了.

**根因**: 写代码时把"加速工具需要的合力 $m\boldsymbol{a}_C$"当成了"传感器读数中应该看到的工具惯性分量". 这两者在牛顿第三定律下等大反向. 重力补偿那一支因为公式简单 (直接 $+m\boldsymbol{g}$), 直觉对了; 惯量补偿涉及"运动方程 → 反作用 → 传感器" 的二次符号转换, 我跳过了显式推导.

**为什么没在审查 / 测试阶段发现**:
- 离线辨识脚本里 `F_meas = +mg − m·a_C` 的符号约定是现成的, 但我对齐"参考点 + 坐标基"的语义时没把整个公式抄来核对符号
- reviewer prompt 只让检查"代码 vs 设计是否一致", 没要求"用第一性原理 / 已知物理公式代回去验证"
- 静止 / 慢动测试中 $|\boldsymbol{F}_{\text{inertia}}| \sim 1\,\mathrm{N}$ 叠在 0.5 N 噪声上, 视觉无法区分"叠加" vs "抵消"
- 没做带运动的合成单元测试 (例如纯线加速度沿 +z, 手算 expected 应为 $-m a_z\hat{z}$)

**修复**: `expected_inertia_wrench_ee` 的两个分量赋值取负号; 同时把 α 估计从 $J_\omega \ddot{q}$ 改成 $\dot{\boldsymbol{\omega}}$ 直接微分 (顺手补 $\dot{J}\dot{q}$ 项).

**护栏建议** (尚未实施):
1. 把惯量公式抽成纯函数, 用合成 case 写 GoogleTest (固定姿态 + 已知 a_C → 断言符号)
2. C++ 文件头引用 `tool_inertia_estimate.py:379` 的 F_meas/M_meas 公式作为 ground truth, 注释里强制显式写"传感器视角符号"
3. 任何新补偿模块 (柯氏力 / 空气阻力 / 弹簧元件) 必须先写"传感器读数模型"再写代码

