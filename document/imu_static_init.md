# IMU 静止初始化：原理、斜坡可行性与重力参数误差分析

## 1. IMU 静止初始化原理

### 1.1 传感器量测模型

设世界坐标系（重力坐标系）Z 轴向上，IMU 静止时的量测模型为：

**加速度计**：
```
a_meas = R_GtoI * [0, 0, g]^T + b_acc + n_acc
```

**陀螺仪**（静止时 ω_true = 0）：
```
ω_meas = b_gyr + n_gyr
```

其中：
- `R_GtoI`：世界系（G）到 IMU 系（I）的旋转矩阵
- `g`：重力加速度幅值（标量，配置参数）
- `b_acc`、`b_gyr`：加速度计和陀螺零偏
- `n_acc`、`n_gyr`：白噪声（N 帧均值后趋近于 0）

### 1.2 可估计量（N 帧静止均值）

对 N 帧静止数据取平均后，噪声消除，得到：

```
acc_avg = R_GtoI * [0, 0, g]^T + b_acc   ≈  R_GtoI.col(2) * g  （零偏小量近似）
gyr_avg = b_gyr
```

可直接估计：

| 量 | 估计方法 | 备注 |
|----|---------|------|
| 陀螺零偏 `b_gyr` | `gyr_avg` | 直接均值 |
| 重力方向（roll+pitch） | `acc_avg.normalized()` = R_GtoI.col(2) | 只确定 2 个自由度 |
| 加速度零偏 `b_acc` | `acc_avg - R_GtoI * [0,0,g]^T` | 依赖 R_GtoI 和 g |

**不可观量（Yaw 角）**：绕重力轴的旋转无法通过加速度计确定，初始化时固定 yaw = 0，只估计 roll 和 pitch。

### 1.3 Roll 和 Pitch 的计算

从加速度均值 `[ax, ay, az]` 计算安装角：

```
roll  = atan2(ay, az)
pitch = atan2(-ax, sqrt(ay² + az²))
```

以此构造 R_GtoI（R_GtoI.col(2) = acc_avg.normalized()，yaw=0 约束下用格拉姆-施密特正交化完整构造）：

```
z = acc_avg.normalized()           // 世界 Z 轴在 IMU 系的投影
e = UnitX (或 UnitY，避免与 z 共线)
x = (e - z*(z·e)).normalized()
y = z × x
x = y × z                          // 重新正交化，消除误差
R.col(0)=x, R.col(1)=y, R.col(2)=z
R_GtoI = SVD 极分解 → SO(3)
```

### 1.4 静止判据

收集 N 帧（N = `imu_max_buff_size_`，100 Hz 下 500 帧 = 5 秒）数据，满足以下条件才执行初始化：

```
wheel_stopped == true                       （编码器轮速静止，第二信源）
acc_std.maxCoeff() < acc_excite_threshold   （加速度标准差，默认 0.04 m/s²）
gyr_std.maxCoeff() < gyro_excite_threshold  （角速度标准差，默认 0.004 rad/s）
| ||acc_avg|| - g_preset | < acc_magnitude_tolerance   （默认 0.2 m/s²）
```

> ⚠️ **不要把 `gyr_avg` 写进静止判据**（本文档 v1 曾要求 `|gyr_avg| < 0.001 rad/s`，是错的）。
>
> 静止时 `gyr_avg` **恒等于陀螺零偏 `b_gyr`**，也就是本流程下一步要估计的量（见 §1.2）。
> 拿待估量当判据量，会让同一个判据对「真在转」和「零偏大」给出相同结论，产生两类错误：
>
> | 器件零偏 | 实际状态 | 旧判据结论 | 后果 |
> |---------|---------|-----------|------|
> | > 0.001 rad/s（0.057°/s）| 真静止 | 拒绝 | **永远无法完成初始化**，非 EKF 模式下里程计一帧不发 |
> | < 0.001 rad/s | 真运动 | 通过 | 拿运动数据估出错误零偏与错误 `R_ItoG`，此后 yaw 持续漂 |
>
> 数值验证见项目脚本 `scripts/imu_static_init_sim.py`：0.001 rad/s 是旧判据的实际门槛，
> 典型工业级 MEMS 零偏（0.002~0.02 rad/s）全部落在拒绝区。
>
> **静止的信息在方差里，不在均值里。** 而恒定角速度、恒定线加速度、匀速直线三种工况
> 的方差与模值都与静止无异（IMU 侧不可区分），必须靠编码器轮速拆开——这就是
> `wheel_stopped` 存在的原因。
>
> `gyr_avg` 的量级仍有价值，但用途是**器件健康告警**：确认静止后若
> `|gyr_avg| > gyro_bias_warn_threshold`（默认 0.01 rad/s）则输出警告，**不阻断**初始化。

---

## 2. 斜坡初始化可行性分析

### 2.1 斜坡对静止检查的影响

在倾角为 θ 的斜面上静止时：

```
acc_avg ≈ [g·sin(θ), 0, g·cos(θ)]^T  （沿斜面前向倾斜）
||acc_avg|| ≈ g
```

**关键结论：加速度模长与斜面倾角无关。**`||acc_avg|| - g_preset ≈ 0`，静止检查**不会因斜坡失败**。

### 2.2 斜坡对 R_ItoG 的影响

斜坡上估计的 R_ItoG 将"斜坡法向"对齐到世界 Z 轴，而非真实竖直方向。旋转后的世界系角速度：

```
ωz_world = R_ItoG[2,:] · ω_imu = cos(θ)·ωz_imu - sin(θ)·ωx_imu
```

与平地相比多了 `-sin(θ)·ωx_imu` 项。当机器人在平地行驶时，路面颠簸导致 `ωx_imu ≠ 0`，此项会污染 yaw 积分。

### 2.3 可行性结论

| 场景 | 初始化成功 | yaw 积分误差量级 | 建议 |
|------|-----------|----------------|------|
| 平地初始化，平地行驶 | ✅ | 极小（仅安装误差） | ✅ 推荐 |
| 斜坡初始化，斜坡行驶（角度恒定） | ✅ | 自洽，误差小 | ⚠️ 可接受 |
| 斜坡初始化，之后转到平地行驶 | ✅ | ~sin(θ) ≈ 1.7%/° | ❌ 应避免 |

**策略**：不阻断斜坡初始化（不会加重问题），但当倾角 > 5° 时输出警告，提示用户在平坦地面重新初始化。
同时把结果记入 `is_last_init_on_slope_` / `last_init_tilt_deg_`，供下游查询——只打日志无法让上层
判断本次初始化的可信度。

实测量级（`scripts/imu_static_init_sim.py`，平地行驶颠簸 ωx = 0.1 rad/s、真实 yaw 速率为 0）：

| 初始化倾角 | 伪 yaw 速率 |
|-----------|------------|
| 2° | 11.9 °/min |
| 5° | 29.7 °/min |
| 8° | 48.2 °/min |
| 15° | 89.1 °/min |

---

## 3. 重力参数 g_preset 误差对角度估计的影响

### 3.1 g_preset 与 R_ItoG 的关系

R_ItoG 的估计**只依赖 `acc_avg` 的方向**（`acc_avg.normalized()`），与重力大小**无关**。

因此 g_preset 的误差 **不会影响 R_ItoG 的精度**，也不会影响 yaw 积分。

### 3.2 g_preset 影响加速度零偏

```
b_acc = acc_avg - R_GtoI * [0, 0, g_preset]^T
```

当 g_preset 有偏差 Δg 时：

```
Δb_acc = R_GtoI * [0, 0, Δg]^T   →  ||Δb_acc|| = Δg
```

对于 g_preset = 9.7887 vs 9.81：

```
Δg = 9.81 - 9.7887 = 0.0213 m/s²
等效加速度误差 = 0.0213 m/s²  ≈  0.22% g
等效倾角误差  = arcsin(Δg/g) ≈ 0.13°
```

**结论：g_preset 的选取（9.7887 或 9.81）对 yaw 角积分影响极小，对线加速度输出的影响 < 0.03 m/s²，可忽略不计。建议使用当地实测值（可通过 GPS 坐标查重力加速度表）。**

---

## 4. 固定重力参数的实现方案

### 4.1 设计原则

1. **固定 g_preset**：使用配置参数 `gravity_magnitude_`，不从数据重估重力幅值
2. **仅估计 roll/pitch**：yaw = 0，从 `acc_avg` 方向构造 `R_GtoI_`
3. **陀螺零偏**：静止均值 `gyr_avg`
4. **加速度零偏**：`acc_avg - R_GtoI_ * [0,0,g_fixed]^T`（使用固定 g_fixed）
5. **不修改 gravity_vector_**：`gravity_vector_` 由 `SetGravityMagnitude()` 写入且保持不变

### 4.2 原始代码问题

| 问题 | 原代码 | 影响 |
|------|--------|------|
| 重估重力大小 | `g_estimated = acc_avg.norm()` 并 `gravity_vector_ = g_estimated` | 将波动的实测值覆盖预设值，破坏 `GetRefinedImuData` 去重力一致性 |
| 加速度零偏用估计值 | `acc_bias = acc_avg - R_GtoI_ * g_estimated` | 与 `gravity_vector_` 不自洽（被覆盖后一致，但语义混乱） |
| 缺少斜坡告警 | 无倾角检查 | 斜坡初始化后平地行驶 yaw 漂移无提示 |

### 4.3 GetRefinedImuData 输出说明

初始化完成后，将原始 IMU 数据变换到"水平参考系"（以初始化时的重力方向为 Z 轴）：

```
// 加速度（去零偏后旋转到世界系，保留重力分量）：
acc_G = R_ItoG_ * (acc_raw - b_acc)

// 角速度（去零偏后旋转到世界系）：
gyr_G = R_ItoG_ * (gyr_raw - b_gyr)
```

其中 `gyr_G.z` 即为绕（初始化时确定的）重力方向的 yaw 角速度，供 `IntegrateImuOnce()` 积分用。

---

## 5. 代码修改说明

**修改文件**：`w1_server/src/imu_processor.cpp`，函数 `ComputeImuBias()`

**修改范围（原 `// ==================== 重估重力方向和大小 ====================` 段）**：

- 删除变量 `g_estimated` 及相关的 `gravity_vector_` 更新
- 删除 `RCLCPP_INFO` 的重力估计打印
- 增加 roll/pitch 计算（用于日志）
- 增加倾角 > 5° 时的 `LOG_WARN` 斜坡告警
- 加速度零偏计算改为使用固定 `gravity_magnitude_`
- 更新 `LOG_WARN` 输出格式，包含 roll/pitch 信息
