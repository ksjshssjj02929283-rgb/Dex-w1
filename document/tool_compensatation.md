# 工具补偿（重力 + 惯量）算法设计文档

## 1 概述

末端六维力传感器测到的 wrench 不只包含真实外力，还包含工具自身带来的分量：

1. 工具重力在 EE 系下的投影。
2. 工具运动时产生的惯性反作用。
3. 真实外力或接触力。

工具补偿的目标是先扣掉 1 和 2，让下游关节阻抗控制中的 `torque_ext = J^T * wrench` 尽量只反映真实外力。

补偿在 `w1_server/src/w1_server.cpp` 的 1 kHz 力矩控制线程中执行：

```text
raw_force_sensor
   ↓ 去零偏
zeroed_sensor_data
   ↓ ApplyToolGravityCompensation
gravity compensated wrench
   ↓ 手动 force_array 注入（可选）
   ↓ ApplyToolInertiaCompensationPrimary（仅 primary 臂）
sensor_wrench → JointImpedanceController
```

当前策略：

- primary 臂：重力补偿 + 惯量补偿。
- secondary 臂：只做重力补偿。
- 未启用补偿时，原始零偏后的 wrench 会继续传给下游。
- 补偿启用了但本周期失败时，对应臂 wrench 清零，避免把工具自重或惯性误当成外力。

## 2 工具重力补偿

配置位于 `w1_server/robot_config.yaml`：

```yaml
tool_gravity_compensation:
  com_mm: [-0.15804154, 0.70721872, 50.33428542]
  mass_kg: 1.061
  gravity_magnitude: 9.79
  model_sign: 1
```

字段含义：

| 字段 | 单位 | 含义 |
|---|---|---|
| `mass_kg` | kg | 工具质量 |
| `com_mm` | mm | 工具 COM 相对 EE 原点的位置，表达在 EE 系 |
| `gravity_magnitude` | m/s² | 重力加速度 |
| `model_sign` | `+1` 或 `-1` | 传感器 wrench 符号约定 |

重力向量在世界系中为：

$$
g_W = [0, 0, -g]^T
$$

根据 EE 姿态，将工具重力投影到 EE 系。代码中的旋转矩阵为 `${}^W R_E`，表示从 EE 系到世界系：

$$
F_g^E = \mathrm{sign} \cdot ({}^W R_E)^T \cdot m g_W
$$

工具 COM 引起的力矩为：

$$
\tau_g^E = r_C^E \times F_g^E
$$

最终扣减：

$$
w_{\mathrm{out}} = w_{\mathrm{in}} - [F_g^E,\ \tau_g^E]^T
$$

EE 姿态来自 `GetEERotationWorldFromEE()`。当机器人处于 `Init` 或 `Error` 等姿态不可用状态时，本周期重力补偿不会生效。

## 3 工具惯量补偿

惯量补偿只对 primary 臂启用，配置复用 `tool_gravity_compensation`：

```yaml
tool_gravity_compensation:
  inertia_com_kg_m2:
    [[0.00111446, 0.0001123, 0],
     [0.0001123, 0.00123732, 0],
     [0, 0, 0.00065]]
  inertia_compensation_filter_hz: 10.0
```

字段含义：

| 字段 | 单位 | 含义 |
|---|---|---|
| `inertia_com_kg_m2` | kg·m² | 工具关于 COM 的惯量张量，表达在 EE 系 |
| `inertia_compensation_filter_hz` | Hz | 末端线加速度和角加速度低通滤波截止频率 |

启用条件：

- `mass_kg > 0`
- `com_mm` 合法
- `inertia_com_kg_m2` 是合法 3x3 矩阵
- primary arm-only Pinocchio 模型可用
- primary 7 个关节反馈可按名称正确取到

惯量补偿使用 primary arm-base 作为计算坐标系。这里假设躯干相对静止，因此 arm-only 模型中的 base 可近似看作惯性系。

每周期主要计算：

$$
\begin{bmatrix}v_{ee}^A \\ \omega^A\end{bmatrix} = J^A(q)\dot{q}
$$

线加速度和角加速度由速度差分得到，并经过二阶低通滤波：

$$
a_{ee}^A = \mathrm{LPF}\left(\frac{v_{ee,k}^A - v_{ee,k-1}^A}{dt}\right)
$$

$$
\alpha^A = \mathrm{LPF}\left(\frac{\omega_k^A - \omega_{k-1}^A}{dt}\right)
$$

COM 在 arm-base 系中的偏移，其中 `${}^A R_E` 表示从 EE 系到 arm-base 系：

$$
r_C^A = {}^A R_E r_C^E
$$

COM 加速度：

$$
a_C^A = a_{ee}^A + \alpha^A \times r_C^A + \omega^A \times (\omega^A \times r_C^A)
$$

驱动工具运动所需的合力和 EE 原点处合力矩：

$$
F^A = m a_C^A
$$

$$
\tau_E^A = I_C^A\alpha^A + \omega^A \times (I_C^A\omega^A) + r_C^A \times F^A
$$

传感器读数中由工具惯性产生的是工具对传感器的反作用，因此符号取负：

$$
w_{\mathrm{inertia}}^E =
\mathrm{sign} \cdot
\begin{bmatrix}
-({}^A R_E)^T F^A \\
-({}^A R_E)^T \tau_E^A
\end{bmatrix}
$$

最终扣减：

$$
w_{\mathrm{out}} = w_{\mathrm{in}} - w_{\mathrm{inertia}}^E
$$

发布到反馈中的 `force_sensor_inertia` 就是这里计算出的 `w_inertia`，它和 `force_sensor` 使用同一传感器视角。

## 4 运行流程和安全策略

力矩控制线程中的顺序为：

1. 读取 primary / secondary 力传感器原始值和零偏。
2. 对有效 wrench 做工具重力补偿。
3. 读取并排序机器人关节反馈。
4. 对 primary 臂做惯量补偿；首拍只初始化滤波器，不做扣减。
5. 组装 `sensor_wrench` 写入下游控制器。

如果补偿启用了但本周期未能成功执行，对应臂的 `sensor_data` 会清零。常见原因包括：

- 当前状态下 EE 姿态不可用。
- 反馈共享内存无有效数据。
- 关节名称或模型维度不匹配。
- 首拍或刚 reset 后惯量滤波器还没有历史数据。

这种策略偏保守：补偿失效时宁愿让 `torque_ext = 0`，也不把未扣除的工具重力或惯性送入阻抗控制。

## 5 标定和验证

### 5.1 重力参数

`mass_kg` 建议用称重结果，`com_mm` 可用：

```text
script/force_sensor/tool_com_estimate.py
```

验证方法：

1. 机械臂保持静止，切换多个姿态。
2. 查看补偿后的 `force_sensor`。
3. 理想情况下三轴力应接近 0，力矩也应接近 0。

如果某些姿态下残差接近工具重量，优先检查：

- `mass_kg`
- `com_mm`
- `model_sign`
- 力传感器零偏是否在 EE 姿态有效后完成

### 5.2 惯量参数

`inertia_com_kg_m2` 可用：

```text
script/force_sensor/tool_inertia_estimate.py
```

动态验证时，建议使用无外部接触的往复动作，对比惯量补偿前后的 wrench RMS。补偿后 RMS 应下降；如果反而上升，优先检查：

- 惯量张量是否表达在 EE 系下
- `model_sign` 是否正确
- `inertia_compensation_filter_hz` 是否过高或过低
- 关节速度反馈是否有明显量化噪声

## 6 文件索引

| 文件 | 作用 |
|---|---|
| `w1_server/src/w1_server.cpp` | 补偿配置加载、重力补偿、惯量补偿、控制线程集成 |
| `w1_server/robot_config.yaml` | 工具补偿参数 |
| `script/force_sensor/tool_com_estimate.py` | 工具质量中心估计 |
| `script/force_sensor/tool_inertia_estimate.py` | 工具惯量估计 |
| `script/force_sensor/force_analyse.py` | 力数据分析辅助脚本 |
