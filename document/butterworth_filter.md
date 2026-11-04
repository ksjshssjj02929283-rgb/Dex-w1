# Butterworth 位置指令滤波

本文说明 dexe_robot_server 当前 `ButterworthFilter` 的计算方法、配置来源及其在 1 kHz 真机控制链路中的实际位置。

## 1. 使用位置

`JointReferenceProcessor` 为每个关节维护一个独立滤波器。有效规划指令到达 TorqueControlThread 后，在力矩计算前按以下顺序处理参考轨迹：

```text
PlannerManagerCommand::cmd.position（250 Hz）
    → LinearInterpolator（插值到 1 kHz）
    → ButterworthFilter
    → EIInputShaper
    → 位置参考 q_ref，并后向差分生成 dq_ref、ddq_ref
        ├── CSP：下发同一份 q_ref
        └── Torque CST：参考轨迹进入力矩控制器
```

该滤波器处理**关节位置参考**，不是关节速度反馈。CSP 直接使用处理后的位置；Torque CST 在 Server 内使用处理后的位置、速度计算 PD/积分力矩，并使用处理后的加速度计算逆动力学前馈，再通过 `joint_torque_feedforward` 下发。

参考速度、加速度由最终位置按固定 1 ms 周期连续后向差分得到，首帧和复位后的首帧置零。它们随位置一起包含滤波和 EI 延迟；关闭平滑滤波时，加速度会反映线性插值拐点。

关键文件：

| 文件 | 作用 |
|---|---|
| [`robot_server_lib/utils/butterworth_filter.cpp`](../robot_server_lib/utils/butterworth_filter.cpp) | 二阶低通滤波器的系数计算、单步滤波和复位 |
| [`robot_server_lib/utils/joint_reference_processor.cpp`](../robot_server_lib/utils/joint_reference_processor.cpp) | 组合关节插值、滤波、整形及参考速度和加速度计算 |
| [`w1_server/src/w1_server.cpp`](../w1_server/src/w1_server.cpp) | 在 1 kHz 力矩计算前调用参考处理，并组装 CSP/CST 输出 |
| [`robot_server_lib/utils/utils.cpp`](../robot_server_lib/utils/utils.cpp) | 从 YAML 读取逐关节开关和截止频率 |
| [`w1_server/robot_config.yaml`](../w1_server/robot_config.yaml) | 保存各关节 `butterworth_filter` 配置 |

## 2. 当前算法

### 2.1 连续模型

当前代码使用阻尼比 `ζ = 1.0` 的二阶临界阻尼低通模型：

```text
H(s) = ωc² / (s² + 2ζωc s + ωc²)
ζ = 1.0
ωc = 2πfc
```

类名保留为 `ButterworthFilter`，但当前阻尼比是 1.0，而经典二阶 Butterworth 的阻尼比是 `1/√2`。当前实现更强调无超调的平滑响应。

采样率固定为 `fs = 1000 Hz`，与 TorqueControlThread 周期一致。构造时先进行频率预畸变：

```text
T = 1 / fs
ωpre = (2 / T) · tan(ωc T / 2)
```

再计算离散二阶 IIR 系数。

### 2.2 差分方程

运行时使用：

```text
y[n] = b0x[n] + b1x[n-1] + b2x[n-2]
       - a1y[n-1] - a2y[n-2]
```

其中 `a0 = 1`，系数由关节配置的截止频率在构造阶段计算，而不是使用固定的 3 Hz 常量。默认构造函数使用 3 Hz，但真机实际按各关节 YAML 参数构造。

截止频率越低，位置指令越平滑、相位滞后越明显；截止频率越高，跟随更快，但对高频分量的抑制更弱。

## 3. 初始化与复位

第一次调用 `Apply(input)` 时，输入与输出历史全部填为当前输入，并直接返回该值，从而避免从零状态启动造成位置跳变。

以下情况会调用 `Reset()`：

- 该关节的滤波开关关闭；
- 规划指令或力矩控制结果缺失；
- 机器人未 Ready，或进入 Sleep / QuickStop。

后两类情况同时复位参考处理的插值与差分历史。复位后下一次启用会重新以当前输入初始化，不继续使用旧轨迹的滤波历史。切回 Position 的临时保持阶段保留滤波历史，并将逆动力学参考加速度置零；Drag 模式同样使用零参考加速度进行动力学补偿。

## 4. 配置与运行时开关

每个关节分别配置：

```yaml
joints:
  - name: LEFT_J1
    butterworth_filter:
      enabled: true
      frequency: 3
```

| 字段 | 含义 |
|---|---|
| `enabled` | 是否对该关节的位置指令启用滤波 |
| `frequency` | 截止频率，单位 Hz；在服务启动时用于构造滤波器 |

`EnableButterworthFilter` Set Property 支持传入一个统一开关，或传入长度为 20 的逐关节开关数组。运行时修改只改变 `enabled`，不会重新构造滤波器，因此修改 `frequency` 后需要重启服务。

辅助命令：

```bash
./set_w1.sh filter true
./set_w1.sh filter false
./script/set_butterworth_filter.sh read
```

[`set_w1.sh`](../set_w1.sh) 的 `filter` 命令会修改用户 YAML 中的逐关节开关，并通过 ROS 2 Set Property 同步当前进程。

## 5. 调参与验证

调整截止频率时应同时观察：

1. 插值前规划位置、滤波前位置和滤波后位置的相位差。
2. 运动起停阶段的位置连续性及到位时间。
3. Butterworth 滤波与 EI 输入整形串联后的总延迟。
4. 关闭再开启滤波时，首次输出是否与当前指令连续。

滤波和输入整形都会引入延迟，不应只通过持续降低截止频率来抑制机械振动；已知共振频率优先由 EI 输入整形针对性处理。
