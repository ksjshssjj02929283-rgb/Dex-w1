# 底盘里程计 EKF 融合设计文档

## 1. 概述

本模块在差速底盘上，将**编码器里程计（odom）**与**惯性测量单元（IMU）**通过扩展卡尔曼滤波（EKF）进行融合，输出协方差完备的里程计消息，并提供**悬空/打滑检测**能力。

### 1.1 融合策略

| `odom_mode` | 行为 | 位置来源 | 航向来源 |
|------|----|------|------|
| `encoder_only` | 直接积分 | 编码器 | 编码器 ω 积分 |
| `imu_yaw` | 直接积分 | 编码器 | 陀螺积分 |
| `ekf`（默认） | **EKF 融合**：IMU 驱动过程模型，编码器 (v, ω) 做量测与检测 | 编码器量测 | IMU 驱动 |
| `ekf_no_imu` | **EKF 无 IMU**：匀速预测 + 编码器 2 维量测；`ekf` 在 IMU 掉线时自动退化到此态 | 编码器量测 | 编码器 ω |

> 原 `use_imu_yaw` / `use_ekf_fusion` 两个布尔已合并为单参数 `odom_mode`（2026-09-04）。IMU 在任何模式下都只提供航向/角速度。

---

## 2. EKF 状态定义

### 2.1 状态向量（5 维）

$$
\mathbf{x}_k = \begin{bmatrix} p_x \\ p_y \\ \theta \\ v \\ \omega \end{bmatrix}
$$

| 变量 | 含义 | 单位 |
|------|------|------|
| $p_x, p_y$ | odom 坐标系下位置 | m |
| $\theta$ | 航向角（绕 Z 轴），范围 $[-\pi, \pi]$ | rad |
| $v$ | 机体前向线速度 | m/s |
| $\omega$ | 机体角速度（yaw rate） | rad/s |

### 2.2 协方差矩阵

$$
P_k \in \mathbb{R}^{5 \times 5}
$$

---

## 3. 过程模型（预测步）

### 3.1 IMU 驱动模式（`imu_available = true`）

IMU 提供：陀螺仪角速度 $\omega_z^{imu}$（已去偏、已旋转至水平系）。

**中点航向积分**（减小非线性误差）：

$$
\theta_{mid} = \theta_k + \tfrac{1}{2} \omega_z^{imu} \cdot \Delta t
$$

**状态传播**：

$$
\hat{\mathbf{x}}_{k+1|k} = \begin{bmatrix}
p_{x,k} + v_k \cos\theta_{mid} \cdot \Delta t \\
p_{y,k} + v_k \sin\theta_{mid} \cdot \Delta t \\
\theta_k + \omega_z^{imu} \cdot \Delta t \\
v_k \\
\omega_z^{imu}
\end{bmatrix}
$$

> **说明**：$v$ 设为匀速（由编码器量测更新），$\omega$ 直接来自 IMU（高带宽、低噪声）。

### 3.2 纯编码器模式（`imu_available = false`）

匀速运动假设：

$$
\hat{\mathbf{x}}_{k+1|k} = \begin{bmatrix}
p_{x,k} + v_k \cos\theta_{mid} \cdot \Delta t \\
p_{y,k} + v_k \sin\theta_{mid} \cdot \Delta t \\
\theta_k + \omega_k \cdot \Delta t \\
v_k \\
\omega_k
\end{bmatrix}
$$

其中 $\theta_{mid} = \theta_k + \tfrac{1}{2}\omega_k \cdot \Delta t$。

### 3.3 过程模型雅可比矩阵 $F_k$

$$
F_k = \frac{\partial f}{\partial \mathbf{x}}\bigg|_{\mathbf{x}_k}
$$

**IMU 模式**（$\partial\theta / \partial\omega_k = 0$，$\partial\omega_{k+1} / \partial\omega_k = 0$）：

$$
F_k^{imu} = \begin{bmatrix}
1 & 0 & -v_k \sin\theta_{mid} \cdot \Delta t & \cos\theta_{mid} \cdot \Delta t & 0 \\
0 & 1 & v_k \cos\theta_{mid} \cdot \Delta t  & \sin\theta_{mid} \cdot \Delta t & 0 \\
0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0
\end{bmatrix}
$$

**纯编码器模式**（$\partial\theta / \partial\omega_k = \Delta t$）：

$$
F_k^{enc} = \begin{bmatrix}
1 & 0 & -v_k \sin\theta_{mid} \cdot \Delta t & \cos\theta_{mid} \cdot \Delta t & 0 \\
0 & 1 & v_k \cos\theta_{mid} \cdot \Delta t  & \sin\theta_{mid} \cdot \Delta t & 0 \\
0 & 0 & 1 & 0 & \Delta t \\
0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 1
\end{bmatrix}
$$

### 3.4 过程噪声 $Q$

$$
Q = \mathrm{diag}\left[\sigma_{qxy}^2,\; \sigma_{qxy}^2,\; \sigma_{q\theta}^2,\; \sigma_{qv}^2,\; \sigma_{q\omega}^2\right]
$$

| 参数 | 符号 | 典型值 | 说明 |
|------|------|--------|------|
| `ekf_q_xy` | $\sigma_{qxy}^2$ | `1e-4` m² | 位置过程扩散（速度不确定性×dt² 量级） |
| `ekf_q_theta` | $\sigma_{q\theta}^2$ | `1e-3` rad² | 无 IMU 时航向扩散 |
| `ekf_q_v` | $\sigma_{qv}^2$ | `0.04` (m/s)² | 线速度扩散（每步约 0.2 m/s std） |
| `ekf_q_omega` | $\sigma_{q\omega}^2$ | `0.01` (rad/s)² | 无 IMU 时角速度扩散 |

**协方差预测**：

$$
P_{k+1|k} = F_k P_k F_k^T + Q
$$

---

## 4. 量测模型（更新步）

### 4.1 编码器量测

$$
\mathbf{z}_k^{enc} = \begin{bmatrix} v^{enc} \\ \omega^{enc} \end{bmatrix}
$$

**量测函数**（线性）：

$$
h(\mathbf{x}) = \begin{bmatrix} v \\ \omega \end{bmatrix}, \quad
H^{enc} = \begin{bmatrix}
0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 1
\end{bmatrix} \in \mathbb{R}^{2 \times 5}
$$

### 4.2 量测噪声 $R$

正常行驶时：

$$
R^{enc} = \mathrm{diag}\left[\sigma_{rv}^2,\; \sigma_{r\omega}^2\right]
$$

悬空/打滑检测触发后（放大量测噪声，使 EKF 忽略不可信的编码器）：

$$
R^{aerial} = \mathrm{diag}\left[\sigma_{rv,air}^2,\; \sigma_{r\omega,air}^2\right]
$$

### 4.3 EKF 更新方程

**新息（innovation）**：
$$
\mathbf{y} = \mathbf{z}^{enc} - H^{enc} \hat{\mathbf{x}}_{k+1|k}
$$

**新息协方差**：
$$
S = H^{enc} P_{k+1|k} (H^{enc})^T + R
$$

**卡尔曼增益**：
$$
K = P_{k+1|k} (H^{enc})^T S^{-1}
$$

**状态更新**：
$$
\mathbf{x}_{k+1} = \hat{\mathbf{x}}_{k+1|k} + K \mathbf{y}
$$

**协方差更新（Joseph 形式，数值稳定）**：
$$
P_{k+1} = (I - K H^{enc}) P_{k+1|k} (I - K H^{enc})^T + K R K^T
$$

> Joseph 形式确保 $P$ 保持对称正定，避免数值误差导致协方差退化。

---

## 5. 悬空/打滑检测

### 5.1 原理

当底盘悬空（轮子转但机体不移动）时，编码器报告的速度与 EKF 预测（基于 IMU 运动学）不一致。这在新息向量 $\mathbf{y}$ 上表现为**统计异常**。

采用 **Mahalanobis 距离检验**：

$$
D_{maha}^2 = \mathbf{y}^T S^{-1} \mathbf{y}
$$

在正常工况下，$D_{maha}^2$ 服从自由度为 2 的卡方分布：

$$
D_{maha}^2 \sim \chi^2(2)
$$

| 阈值 | 置信度 | 含义 |
|------|--------|------|
| $\chi^2(2, 95\%) = 5.99$ | 95% | 较敏感，5% 正常样本会误判 |
| $\chi^2(2, 99\%) = 9.21$ | 99% | **推荐默认值** |
| $\chi^2(2, 99.9\%) = 13.82$ | 99.9% | 宽松，仅检测严重异常 |

### 5.2 处理逻辑

```
if D_maha² > threshold:
    is_aerial = true
    R ← R_aerial  （放大量测噪声）
    odom covariance ← large  （发布大协方差告知上层不可信）
else:
    is_aerial = false
    R ← R_enc  （正常量测噪声）
```

### 5.3 局限性

- **无 IMU 时**：EKF 采用匀速预测，与编码器量测几乎一致，Mahalanobis 距离始终小，**无法可靠检测悬空**
- 悬空检测依赖 IMU 提供独立的运动先验

---

## 6. 完整流程图

```
[IMU 回调 ~100Hz]
    │
    ├── AddImuData() → 静止初始化 → GetRefinedImuData()
    └── 存入 curr_imu_（refined，含 wz_world、acc_world）

[Timer 回调 100Hz]
    │
    ├── 读编码器 → v_enc = speed_avg * odom_factor
    │             omega_enc = (v_r - v_l) / wheel_track
    │
    ├── [odom_mode = ekf | ekf_no_imu]
    │       │
    │       ├── 读 wz_imu = curr_imu_→angular_velocity.z * imu_factor
    │       │
    │       ├── EKF::Step(dt, v_enc, omega_enc, wz_imu, imu_available)
    │       │       ├── Predict(dt, wz_imu, imu_available)
    │       │       └── Update(v_enc, omega_enc) → aerial flag
    │       │
    │       └── 读 EKF 状态 → (odom_x, odom_y, theta, v, omega)
    │
    └── [odom_mode = encoder_only | imu_yaw]
            └── 现有直接积分逻辑（原代码不变）

[PublishOdometry()]
    ├── 发布 /feedback/odom（含协方差矩阵）
    └── 发布 odom → base_footprint TF
```

---

## 7. 协方差发布策略

### `pose.covariance`（6×6 展开，取 $p_x, p_y, \theta$）：

正常：
$$
P_{pose} = \mathrm{diag}[P[0,0],\; P[1,1],\; P[2,2]]
$$

悬空时上层应降低 odom 权重，`robot_localization` 会自动根据协方差加权。

### robot_localization EKF 配置参考（`ekf_odom.yaml`）：

```yaml
# 融合 /feedback/odom（线速度、angular_z）和 /imu/chassis/refined（wz_world）
odom0: /feedback/odom
odom0_config: [false, false, false,
               false, false, false,
               true,  false, false,
               false, false, true,
               false, false, false]

imu0: /imu/chassis/refined
imu0_config: [false, false, false,
              false, false, false,
              false, false, false,
              false, false, true,
              false, false, false]
```

---

## 8. 参数一览

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `odom_mode` | `ekf` | 里程计模式四选一：`encoder_only` / `imu_yaw` / `ekf` / `ekf_no_imu`，可热切换 |
| `ekf_q_xy` | `1e-4` | 位置过程噪声 (m²) |
| `ekf_q_theta` | `1e-3` | 航向过程噪声 (rad²) |
| `ekf_q_v` | `0.04` | 线速度过程噪声 ((m/s)²) |
| `ekf_q_omega` | `0.01` | 角速度过程噪声 ((rad/s)²) |
| `ekf_r_v` | `0.05` | 编码器线速度噪声 ((m/s)²) |
| `ekf_r_omega` | `0.01` | 编码器角速度噪声 ((rad/s)²) |
| `ekf_r_v_aerial` | `100.0` | 悬空时编码器线速度噪声（放大） |
| `ekf_r_omega_aerial` | `100.0` | 悬空时编码器角速度噪声（放大）|
| `ekf_aerial_chi2_threshold` | `9.21` | 悬空检测 Mahalanobis 阈值 |
| `ekf_p0_xy` | `0.01` | 初始位置协方差对角 |
| `ekf_p0_theta` | `0.01` | 初始航向协方差对角 |
| `ekf_p0_v` | `1.0` | 初始线速度协方差对角 |
| `ekf_p0_omega` | `1.0` | 初始角速度协方差对角 |

## 9. IMU 健康状态机与诊断外发

IMU 可用性（`IsImuAvailable()`）由 `imu_health.h` 的 `ImuHealthMonitor` 判定，取代原先单一的超时判据：

| 状态 | 判据 | 参数 |
|---|---|---|
| invalid | 六轴 / stamp 非有限值连续 ≥ N 帧（入口丢弃）；或 stamp 与节点时钟差 < −1 s / > 1e6 s | `imu_invalid_frames` |
| frozen | 六轴原始值连续 N 帧完全相同 | `imu_frozen_frames` |
| timeout | 最近一帧 stamp 距当前 > T | `imu_timeout` |
| noisy | 陀螺 z 帧间差分 1 s 滑窗 std > σ | `imu_gyro_noise_max` |
| uninitialized | 静止初始化未完成（> `imu_init_timeout` 升 ERROR；从未收到样本同此宽限） | `imu_init_timeout` |

- 多条同时满足取最严重；非 ok 即不可用 → EKF 走无 IMU 分支（§4 量测只用轮速）。
- 数据类失效解除后连续 `imu_recover_frames` 帧正常才回 ok（迟滞）。
- 噪声用**帧间差分**而非原始值 std：真实转动的差分 ≈ ω̇·dt 且平滑，噪声的差分随机跳变，两者可分。
- 评估在 `onTimer()` 最前独立执行（`UpdateImuHealth()`），不依赖底盘反馈有效。
- 外发：`diagnostic_updater` → `/diagnostics`，status `chassis_node: IMU`（含 `effective_mode` / `init_*` 进度与拒绝原因 / `gyro_diff_std_rad_s`）与 `chassis_node: odom feedback`（fail-fast 状态）。
- 单测：`test/test_imu_health.cpp`。

## 10. 全向底盘 6 维 EKF（`omni_chassis_ekf.{h,cpp}`）

全向（舵轮）底盘用独立的 `OmniChassisEkf`，状态 (x, y, θ, v_x, v_y, ω)，与 §2–§7 的差速设计同构、共用 `ChassisEkf::Params`：

- 预测：x += (v_x cosθ_m − v_y sinθ_m)·dt，y += (v_x sinθ_m + v_y cosθ_m)·dt；θ / ω 与差速相同（IMU 驱动时 F(2,5)=F(5,5)=0）
- 雅可比：F(0,2)=(−v_x s − v_y c)dt，F(0,3)=c·dt，F(0,4)=−s·dt；F(1,2)=(v_x c − v_y s)dt，F(1,3)=s·dt，F(1,4)=c·dt
- 量测：[v_x, v_y, ω]（四舵轮反解，梯形均值）；检测 3 维新息，阈值 `aerial_chi2_threshold_3d`（χ²(3) 99% = 11.34）；IMU 模式更新只取 v_x、v_y 两行；ZUPT 三维零速
- 新增参数：`q_vy` `r_vy` `r_vy_aerial` `p0_vy` `aerial_chi2_threshold_3d`；其余与差速共用
- v_y ≡ 0 时退化为差速 EKF（x / θ / v_x / ω 协方差逐位一致，见 `test/test_omni_chassis_ekf.cpp`）
- 节点侧：`IntegrateOmniOdomOnce` 在 `use_ekf_fusion_` 时调 `omni_ekf_->Step`；`PublishOdometry` 填 P(4,4) 到 `twist.covariance[7]`；`ApplyOdomMode` 按底盘类型灌初值 / 接续协方差
