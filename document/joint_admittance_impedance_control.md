# 含导纳环的关节阻抗控制算法设计文档

## 1 概述

本控制器用于力控模式下的关节空间柔顺控制。整体思路是：

1. 六维力传感器 wrench 经过死区、饱和和符号处理后，通过雅可比转置映射为关节外力矩 `torque_ext`。
2. 导纳环把 `torque_ext` 转换成关节参考位置偏移 `q_offset` 和速度偏移 `dq_offset`。
3. 阻抗环跟踪修正后的参考位置 `q_cmd + q_offset`。
4. 逆动力学前馈补偿重力、科氏力和摩擦力。

控制周期在 `TorqueControllerInvDynamic::ComputeCommand` 中完成，主要代码位于：

- `robot_server_lib/dynamics/joint_admittance_controller.cpp`
- `robot_server_lib/dynamics/joint_impedance_controller.cpp`
- `robot_server_lib/torque_controller/torque_controller_inverse_dynamic.cpp`

简化流程如下：

```text
sensor_wrench
   ↓ deadband / saturation / sign
torque_ext = J^T * wrench
   ↓
JointAdmittanceController
   ↓ q_offset, dq_offset
q_ref_adm = q_cmd + q_offset
dq_ref_adm = dq_cmd + dq_offset
   ↓
JointImpedanceController
   ↓
tau_total = tau_impedance + InverseDynamics(q, dq, 0)
```

导纳环的作用是让外力先变成位置退让量，而不是直接全部叠加到力矩输出上。这样阻抗环仍可以保持较高跟踪刚度，柔顺性主要由导纳参数决定。

## 2 外力矩估计

外力矩由 `JointImpedanceController::ComputeExternalTorque` 计算：

$$
\tau_{\mathrm{ext}} = J^T(q) \cdot w
$$

其中 `w` 是力传感器的六维 wrench。进入雅可比映射前会先做处理：

$$
w_{\mathrm{proc}} = \mathrm{sign} \cdot \mathrm{saturate}(\mathrm{deadband}(w_{\mathrm{raw}}))
$$

相关配置在 `w1_server/robot_config.yaml` 的 `force_sensor_external_wrench`：

| 字段 | 含义 |
|---|---|
| `model_sign` | 传感器符号约定，当前配置为 `-1` |
| `force_deadband_n` | 力死区，单位 N |
| `force_saturation_n` | 力饱和，单位 N |
| `torque_deadband_nm` | 力矩死区，单位 N·m |
| `torque_saturation_nm` | 力矩饱和，单位 N·m |

左右臂分别使用各自的 arm-only Pinocchio 模型计算雅可比，再映射回全身关节向量。

## 3 导纳环

导纳环为每个关节模拟一个二阶虚拟系统：

$$
M_i \ddot{q}_{\mathrm{offset},i}
+ B_i \dot{q}_{\mathrm{offset},i}
+ K_i q_{\mathrm{offset},i}
= g_i \tau_{\mathrm{ext},i}
$$

其中：

| 符号 | 配置字段 | 含义 |
|---|---|---|
| $g_i$ | `torque_gain` | 外力矩输入增益 |
| $M_i$ | `virtual_inertia` | 虚拟惯量 |
| $K_i$ | 由 `natural_frequency_hz` 推导 | 虚拟刚度 |
| $B_i$ | 由 `damping_ratio` 推导 | 虚拟阻尼 |

刚度和阻尼不直接配置，而是由固有频率和阻尼比计算：

$$
\omega_n = 2 \pi f_n
$$

$$
K = M \omega_n^2,\quad B = 2 \zeta M \omega_n
$$

默认阻尼比为 `1.2`，属于过阻尼设置，优先避免外力突变后的振荡。

导纳状态使用显式欧拉积分：

$$
\ddot{q}_{\mathrm{offset}} =
\frac{g\tau_{\mathrm{ext}} - B\dot{q}_{\mathrm{offset}} - Kq_{\mathrm{offset}}}{M}
$$

$$
\dot{q}_{\mathrm{offset}} \leftarrow \dot{q}_{\mathrm{offset}} + \ddot{q}_{\mathrm{offset}} dt
$$

$$
q_{\mathrm{offset}} \leftarrow q_{\mathrm{offset}} + \dot{q}_{\mathrm{offset}} dt
$$

每次积分后都会对位置、速度、加速度限幅：

| 状态 | 配置字段 | 默认值 |
|---|---|---|
| `q_offset` | `offset_limit` | 0.05 rad |
| `dq_offset` | `velocity_limit` | 0.4 rad/s |
| `ddq_offset` | `acceleration_limit` | 5.0 rad/s² |

当位置偏移到达限幅边界时，继续向外积分的速度和加速度会被清零，避免偏移长期顶在限幅上。

## 4 阻抗环和力矩输出

导纳使能后，阻抗环使用修正后的参考量：

$$
q_{\mathrm{ref}} = q_{\mathrm{cmd}} + q_{\mathrm{offset}}
$$

$$
\dot{q}_{\mathrm{ref}} = \dot{q}_{\mathrm{cmd}} + \dot{q}_{\mathrm{offset}}
$$

阻抗控制律为：

$$
\tau_{\mathrm{imp}} =
K_p(q_{\mathrm{ref}} - q)
+ K_v(\dot{q}_{\mathrm{ref}} - \dot{q})
+ K_p K_i \int(q_{\mathrm{ref}} - q)dt
$$

位置误差小于 `0.005 deg` 时会置零，用于减少小误差抖动。

积分项只在系统基本静止、外力和导纳偏移都已收敛时启用。当前门限为：

| 条件 | 阈值 |
|---|---|
| `abs(torque_ext)` | `< 0.2 N·m` |
| `abs(q_offset)` | `< 0.003 rad` |
| `abs(dq_offset)` | `< 0.03 rad/s` |
| `abs(cmd_dq_admittance)` | `< 0.03 rad/s` |
| `abs(feedback_dq)` | `< 0.05 rad/s` |

最终力矩：

$$
\tau_{\mathrm{total}} =
\tau_{\mathrm{imp}} + \tau_{\mathrm{ff}}
$$

其中：

$$
\tau_{\mathrm{ff}} = \mathrm{InverseDynamics}(q,\dot{q},0)
$$

当 `joint_admittance.enabled = true` 时，外力矩默认只进入导纳环。若需要保留一部分外力直通，可以设置：

$$
\tau_{\mathrm{imp}} \leftarrow \tau_{\mathrm{imp}} + \mathrm{torque\_ext\_direct\_gain}\cdot\tau_{\mathrm{ext}}
$$

当前默认 `torque_ext_direct_gain = 0.0`。如果关闭导纳环，控制器会退回到原来的阻抗控制方式，直接叠加 `torque_ext`。

## 5 状态机行为

### 5.1 Operating

正常运行时执行完整链路：

1. 计算 `torque_ext`。
2. 导纳环更新 `q_offset` 和 `dq_offset`。
3. 阻抗环跟踪 `q_cmd + q_offset`。
4. 叠加逆动力学前馈输出最终关节力矩。

### 5.2 Idle / Decelerating

非运行状态下不继续用外力驱动导纳环，而是调用 `DecayToZero(dt)`，让导纳偏移通过虚拟阻尼平滑回零。这样可以避免状态切换时参考位置突然跳变。

`torque_ext` 仍会计算并写入反馈，方便上层监控外力。

## 6 配置参数

导纳配置位于 `w1_server/robot_config.yaml` 的 `joint_admittance`：

```yaml
joint_admittance:
  enabled: true
  torque_ext_direct_gain: 0.0
  default:
    torque_gain: -0.9
    virtual_inertia: 1.0
    natural_frequency_hz: 1.5
    damping_ratio: 1.2
    offset_limit: 0.05
    velocity_limit: 0.2
    acceleration_limit: 1.3
```

`default` 会先应用到所有关节，`groups` 可以覆盖某些关节组。组配置中列出的关节会自动使能导纳。

主要调参方向：

- `torque_gain`：决定外力矩转换为位置偏移的方向和强度。当前配置用负号表示顺应外力方向移动。
- `natural_frequency_hz`：越大响应越快，但过大时更容易激发结构振动。
- `damping_ratio`：越大越平稳，响应也越慢；建议先使用 `1.0 ~ 1.5`。
- `virtual_inertia`：越大越不容易被外力推动。
- `offset_limit`：限制最大退让角度，防止外力异常时偏移过大。
- `torque_ext_direct_gain`：一般保持 `0.0`；只有需要更直接的力矩响应时再小幅增加。

## 7 验证建议

1. 静止无接触时，`torque_ext` 应接近零；如果长期偏大，优先检查力传感器零偏、工具补偿和 `model_sign`。
2. 人手轻推关节或末端时，`joint_admittance_q_offset` 应随外力方向变化，松手后逐渐回零。
3. `q_cmd + q_offset` 不应出现跳变；如果有跳变，检查状态切换和 `DecayToZero` 行为。
4. 调参时先固定阻抗参数，只调整导纳参数；确认柔顺性合适后再微调阻抗刚度和阻尼。
