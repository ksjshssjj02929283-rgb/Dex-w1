# EI 输入整形

本文说明 dexe_robot_server 当前 `EIInputShaper` 的算法、参数及其在关节位置和差速底盘控制链路中的实际用法。

## 1. 使用位置

### 1.1 关节位置指令

真机为每个关节创建一个独立的三脉冲 EI 整形器。位置指令处理顺序为：

```text
PlannerManagerCommand::position（250 Hz）
    → LinearInterpolator（插值到 1 kHz）
    → ButterworthFilter
    → EIInputShaper
    → ControlSharedMemory::joint_position
```

EI 整形处理 Butterworth 滤波后的**关节位置指令**，通过延迟脉冲叠加降低指定固有频率附近的残余振动。它对 CSP 模式的位置闭环直接生效；CST 模式下位置字段不参与底层控制。

### 1.2 差速底盘

同一个类也用于差速底盘左右轮速度整形：

- 双通道 `Apply(left, right)` 分别保存左右轮历史；
- 当前底盘整形器同样使用 EI 类型；
- 仅差速模式且 `enable_chassis_shaper=true` 时启用；
- 全向底盘电机控制直接透传，不经过该整形器。

关键文件：

| 文件 | 作用 |
|---|---|
| [`robot_server_lib/utils/ei_input_shaper.cpp`](../robot_server_lib/utils/ei_input_shaper.cpp) | EI、Two-Hump EI、斜坡限制及历史缓冲实现 |
| [`w1_server/src/w1_server.cpp`](../w1_server/src/w1_server.cpp) | 创建关节与底盘整形器，并接入 1 kHz 控制链路 |
| [`robot_server_lib/utils/utils.cpp`](../robot_server_lib/utils/utils.cpp) | 读取逐关节开关和固有频率 |
| [`w1_server/robot_config.yaml`](../w1_server/robot_config.yaml) | 保存关节 `input_shaper` 和底盘整形开关 |

## 2. 三脉冲 EI 算法

当前关节和底盘实例都使用 `EIInputShaper::EI`。对于输入 `u[k]`，输出为：

```text
y[k] = A1u[k] + A2u[k-T2] + A3u[k-T3]
```

三项权重满足：

```text
A1 + A2 + A3 = 1
```

因此稳定直流输入的稳态增益为 1。

设配置的固有频率为 `fn`、采样周期为 `Ts`：

```text
Tvib = 1 / fn
T3 = round(Tvib / Ts)
```

`A1`、`A2`、`A3` 和中间脉冲延迟 `T2` 根据振动容忍度 `V` 与阻尼比 `ζ` 的 EI 拟合公式计算。当前类内参数固定为：

```text
ζ = 0.05
V = 0.05
Ts = 0.001 s
```

`fn` 越低，完整脉冲序列越长，引入的整形延迟也越大。关节 `fn` 从各关节 YAML 配置读取；差速底盘当前在代码中使用 `1.67 Hz`。

## 3. 斜坡限制与缓冲区

在脉冲卷积之前，输入先经过变化率限制：

```text
Δmax = max_ramp_rate · Ts
r[k] = clamp(u[k], r[k-1] - Δmax, r[k-1] + Δmax)
```

关节位置整形器当前使用：

```text
max_ramp_rate = 10π rad/s
Ts = 0.001 s
Δmax ≈ 0.0314 rad/周期
```

差速底盘使用由轮径和最大线加速度换算得到的轮速变化率。

整形器使用固定长度 `T3 + 1` 的历史队列。首次调用时用当前斜坡输入填满缓冲区并直接返回当前值，避免启动时从零逐渐爬升。

## 4. 支持的类型

`EIInputShaper` 类实现三种类型：

| 类型 | 行为 |
|---|---|
| `NONE` | 不进行脉冲整形，仅保留斜坡限制 |
| `EI` | 三脉冲 Extra Insensitive 整形 |
| `TWO_HUMP_EI` | 四脉冲 Two-Hump EI，使用更长历史缓冲 |

当前 `w1_server` 的关节和差速底盘实例均固定选择 `EI`；`TWO_HUMP_EI` 目前只作为类能力和单元测试覆盖，并未由 YAML 选择。

## 5. 初始化与复位

以下情况会复位关节整形器：

- 该关节 `input_shaper.enabled=false`；
- 当前周期没有有效规划指令或力矩控制结果；

差速底盘未启用整形或退出差速模式时，也会复位对应整形状态。

`Reset()` 会清空单通道及双通道缓冲区、卷积初始化标志和斜坡输出。再次启用时从当前输入重新初始化，不继续叠加旧运动的延迟脉冲。

## 6. 配置与运行时开关

关节配置示例：

```yaml
joints:
  - name: LEFT_J1
    input_shaper:
      enabled: true
      frequency: 6.7
```

| 字段 | 含义 |
|---|---|
| `enabled` | 是否对该关节的位置指令启用 EI 整形 |
| `frequency` | 目标机械模态的固有频率，单位 Hz；服务启动时用于计算脉冲和延迟 |

`EnableInputShaper` Set Property 支持统一开关或长度为 20 的逐关节开关数组。运行时只更新 `enabled`，不会重算系数，因此修改 `frequency` 后需要重启服务。

辅助命令：

```bash
./set_w1.sh zvd true
./set_w1.sh zvd false
./script/set_input_shaper.sh read
```

命令名沿用 `zvd`，但当前关节实现实际选择的是 EI，而不是 ZVD。脚本会修改用户 YAML 的逐关节开关，并通过 ROS 2 Set Property 同步当前进程。

## 7. 调参与验证

固有频率应来自实测振动模态，而不是仅凭运动观感选择。验证时应关注：

1. 运动停止后的残余振动幅值和衰减时间。
2. 原始目标、Butterworth 输出和 EI 输出之间的时间延迟。
3. 不同负载和姿态下共振频率变化对整形效果的影响。
4. 关闭或重新启用整形时的位置连续性。
5. 斜坡限制与 EI 延迟叠加后是否影响轨迹到位时间。

若目标模态频率配置错误，整形可能无法抵消振动，甚至放大其他频段的响应。修改参数后应先低速、单关节验证，再逐步扩大运动范围。
