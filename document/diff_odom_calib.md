# 差速底盘里程计精细建模与标定方案

## 1. 标准差速底盘运动学模型

### 1.1 轮速到机体速度

设左轮角速度 $\omega_L$（rad/s）、右轮角速度 $\omega_R$（rad/s），左右轮半径相同 $r$（m），有效轴距 $b_\text{eff}$（m）：

$$v_L = \omega_L \cdot r, \quad v_R = \omega_R \cdot r$$

$$v = \frac{v_R + v_L}{2} = \frac{r(\omega_R + \omega_L)}{2}, \quad \omega = \frac{v_R - v_L}{b_\text{eff}} = \frac{r(\omega_R - \omega_L)}{b_\text{eff}}$$

编码器原始值（RPM）轮 rad/s：

$$\omega_{L/R} = n_{L/R}[\text{RPM}] \times \frac{2\pi}{60}$$

### 1.2 Jacobian 参数化

将 $(v, \omega)$ 写成矩阵形式：

$$\begin{bmatrix} v \\ \omega \end{bmatrix} = \begin{bmatrix} J_{11} & J_{12} \\ J_{21} & J_{22} \end{bmatrix} \begin{bmatrix} \omega_L \\ \omega_R \end{bmatrix}$$

左右轮半径相同时：

$$J_{11} = J_{12} = \frac{r}{2},\quad J_{21} = -\frac{r}{b_\text{eff}},\quad J_{22} = \frac{r}{b_\text{eff}}$$

反解物理参数：

$$r = 2 J_{11} = 2 J_{12}, \quad b_\text{eff} = \frac{r}{J_{22}} = -\frac{r}{J_{21}}$$

标定时只需参数化为 $(r, b_\text{eff})$ 共 **2 个自由度**，可進一步简化标定方程组。

> 注：$J_{21}$ 为负数（左轮正转使 $\omega$ 为负）。

---

## 2. 参数物理含义

| 参数 | 含义 | 默认值 | 备注 |
|------|------|--------|------|
| `wheel_radius` | 轮半径 $r$ (m) | 0.0865 | 来自 173mm 直径，左右相同 |
| `wheel_track` | 几何轴距 (m) | 0.439 | 卷尺测量 |
| `wheel_track_eff` | 有效轴距 $b_\text{eff}$ (m) | = `wheel_track` | 需标定，通常略大于几何值 |

> $b_\text{eff}$ 通常比几何轴距偏大 3–10%，原因包括：轮胎弹性形变、轮接地点外移、地面打滑。

---

## 3. 标定可观性分析

### 3.1 问题设定

设 2D 激光输出 SE(2) 里程计相对位姿 $\mathbf{T}^L_{t \to t+1}$（激光坐标系），
机器人编码器给出机体相对位姿 $\mathbf{T}^B_{t \to t+1}(r_L, r_R, b_\text{eff})$，
激光外参 $\boldsymbol{\xi} = (x_L, y_L, \theta_L)$（base_link 到 lidar 的 SE(2) 变换）。

三者满足：

$$\mathbf{T}^L = \boldsymbol{\xi}^{-1} \cdot \mathbf{T}^B \cdot \boldsymbol{\xi}$$

需要辨识的参数总计 **6 个**：$(r_L, r_R, b_\text{eff}, x_L, y_L, \theta_L)$。

### 3.2 同时标定的可行性

**结论：理论上可以同时标定，但需满足严格的激励条件。**

#### 可观性分析

**1. 旋转方向（$\theta_L$）**

对纯直线运动 $\mathbf{T}^B = (\Delta s, 0, 0)$：

$$\mathbf{T}^L_\text{translation} = R(-\theta_L) \cdot \Delta \mathbf{p}$$

方向取决于 $\theta_L$；多方向直线运动可唯一辨识 $\theta_L$（**条件**：机器人朝向不同方向行驶）。

**2. 位置偏置（$x_L, y_L$）**

对纯旋转运动 $\mathbf{T}^B = (0, 0, \Delta\theta)$：

$$\mathbf{T}^L_\text{translation} = (I - R(\Delta\theta)) \cdot \mathbf{t}_L$$

观察到的激光平移量与 $\mathbf{t}_L = (x_L, y_L)$ 直接相关（**条件**：激光不在旋转中心）。

**3. 轮半径比 $r_R / r_L$**

直线行驶时，若 $r_R \neq r_L$，机器人会轻微偏转。激光里程计可观测到这个偏转，从而约束 $r_R / r_L$（**条件**：足够远的直线段）。

**4. 比例尺（绝对轮半径）**

激光里程计测量物理距离（米），因此绝对尺度可观。纯比例缩放 $(r_L, r_R, b_\text{eff}) \to (\lambda r_L, \lambda r_R, \lambda b_\text{eff})$ 会同时缩放激光移动距离预测值，与激光实测不一致，**比例尺可观**。

#### 不可分离的耦合（关键）

虽然理论上可观，但存在以下**强耦合**：

| 耦合 | 原因 |
|------|------|
| $\theta_L$ 与每步的 $\mathbf{T}^B$ 方向 | 旋转外参改变激光测量的"前进方向"基准 |
| $b_\text{eff}$ 与 $(x_L, y_L)$ | 旋转量误差会被激光偏置部分补偿 |
| $r_L, r_R$ 与 $b_\text{eff}$ | 三个参数只有两个自由度（$J_{21}, J_{22}$），需要外部约束（如 $b_\text{eff}$ 的先验） |

**实践中出现的问题**：
- 非凸优化，容易陷入局部最小值
- 激光里程计本身有漂移，污染标定数据
- 需要数量充足且激励充分的运动序列

### 3.3 推荐方案：分步标定

#### 方案 A：先固定外参，再标定轮参（推荐）

**步骤 1：机械测量 + 验证激光外参 $\boldsymbol{\xi}$**

通过 CAD 或卷尺获得 $(x_L, y_L, \theta_L)$ 初值，用以下方法验证：
- 机器人原地旋转 360°，激光扫描到固定障碍物
- 若 $\theta_L$ 正确，激光轨迹应为以 $(x_L, y_L)$ 为圆心的圆弧

**步骤 2：用激光 SE(2) 里程计作为"真值"，最小二乘标定轮参**

已知 $\boldsymbol{\xi}$，将激光相对位姿转换到机体系：

$$\mathbf{T}^B_k = \boldsymbol{\xi} \cdot \mathbf{T}^L_k \cdot \boldsymbol{\xi}^{-1}$$

从 $\mathbf{T}^B_k$ 提取 $(v_k^\text{gt}, \omega_k^\text{gt})$，联立线性方程：

**线速度方程（N组数据）：**

$$\underbrace{\begin{bmatrix} (\omega_{L,1}+\omega_{R,1})/2 \\ \vdots \\ (\omega_{L,N}+\omega_{R,N})/2 \end{bmatrix}}_{\mathbf{a}_v} r = \underbrace{\begin{bmatrix} v_1^\text{gt} \\ \vdots \\ v_N^\text{gt} \end{bmatrix}}_{\mathbf{b}_v}, \quad \hat{r} = \frac{\mathbf{a}_v^T \mathbf{b}_v}{\mathbf{a}_v^T \mathbf{a}_v}$$

**角速度方程（N组数据）：**

$$\underbrace{\begin{bmatrix} \omega_{R,1}-\omega_{L,1} \\ \vdots \\ \omega_{R,N}-\omega_{L,N} \end{bmatrix}}_{\mathbf{a}_\omega} \frac{r}{b_\text{eff}} = \underbrace{\begin{bmatrix} \omega_1^\text{gt} \\ \vdots \\ \omega_N^\text{gt} \end{bmatrix}}_{\mathbf{b}_\omega}, \quad \hat{b}_\text{eff} = \frac{\hat{r} \cdot \mathbf{a}_\omega^T \mathbf{a}_\omega}{\mathbf{a}_\omega^T \mathbf{b}_\omega}$$

两个独立的一元最小二乘闭式解出 $\hat r$、$\hat b_\text{eff}$，写入 YAML 即可。

> **激励条件**：数据需包含：
> - 不同速度的直线段（约束 $r$ 的绝对值）
> - 不同半径的圆弧（约束 $b_\text{eff}$）
> - 排除纯静止数据

#### 方案 B：同时标定外参和轮参（激光外参未知时）

最小化以下目标函数（Gauss-Newton / LM 迭代）：

$$\min_{\boldsymbol{\theta}} \sum_{k} \left\| \text{Log}\left(\boldsymbol{\xi}(\boldsymbol{\theta})^{-1} \cdot \mathbf{T}^B_k(\boldsymbol{\theta}) \cdot \boldsymbol{\xi}(\boldsymbol{\theta}) \cdot (\mathbf{T}^L_k)^{-1}\right) \right\|^2$$

其中 $\boldsymbol{\theta} = (r, b_\text{eff}, x_L, y_L, \theta_L)$（共 **5 个参数**），$\text{Log}(\cdot)$ 为 SE(2) 对数映射。

**注意**：
- 初始值质量至关重要（建议先用方案 A 粗标定）
- 激光里程计累积误差会污染结果，建议用短序列（每段 < 5 m）

---

### 3.4 针对"前进后退 + S形"运动的可观性分析

#### 每步观测方程

对 SE(2) 外参 $\boldsymbol{\xi}=(x_L, y_L, \theta_L)$，每步观测方程为：

$$\Delta\mathbf{p}^L = \underbrace{(I - R(\Delta\theta^B)) \cdot \mathbf{t}_L}_{\text{旋转臂项}} + R(-\theta_L) \cdot \Delta\mathbf{p}^B$$

其中 $\mathbf{t}_L = (x_L, y_L)^T$，$\Delta\mathbf{p}^B$ 由编码器计算。旋转臂项的展开：

$$(I - R(\Delta\theta)) \mathbf{t}_L \approx \Delta\theta \begin{bmatrix} -y_L \\ x_L \end{bmatrix} + O(\Delta\theta^2)$$

**$(x_L, y_L)$ 的可观性完全依赖于 $\Delta\theta^B$ 是否足够大。**

#### 各运动段信息贡献分析

| 运动段 | $\theta_L$ | $r_L+r_R$ | $r_L-r_R$ 与 $b_\text{eff}$ | $(x_L,y_L)$ |
|--------|-----------|----------|---------------------------|------------|
| 直线前进 | ✅ 强 | ✅ 强 | ❌ 同速无差，不激励 | ❌ $\Delta\theta^B \approx 0$，旋转臂消失 |
| 直线后退 | ✅ 补充反向消歧 | ✅ | ❌ | ❌ |
| S形左转段 | ✅ | ✅ | ✅ 提供 $\omega>0$ 约束 | ⚠️ 弱（小转角 $\Delta\theta \approx 0.1$–$0.3$ rad） |
| S形右转段 | ✅ | ✅ | ✅ 与左转联立可分离 $r_L-r_R$ | ⚠️ 弱（同上） |

#### 关键问题：S形的旋转量不足以观测 $(x_L, y_L)$

S形前进时，每段转角 $\Delta\theta^B \approx 0.1$–$0.3$ rad（约 6°–17°）。旋转臂量级约为：

$$|\text{旋转臂}| \approx \Delta\theta \cdot |\mathbf{t}_L| \approx 0.2 \times 0.1 \text{ m} = 0.02 \text{ m}$$

而激光里程计一步噪声约 1–3 cm，信噪比极低，$(x_L, y_L)$ 实际上**不可可靠辨识**。

#### 结论：前进后退 + S形的标定能力

| 参数 | 可观性 | 说明 |
|------|--------|------|
| $r_L + r_R$（线速度尺度） | ✅ 强 | 直线段距离约束 |
| $r_L - r_R$（轮径差） | ✅ 较强 | S形左右段联立分离 |
| $b_\text{eff}$（有效轴距） | ✅ 较强 | S形 $\omega/v$ 比约束 |
| $\theta_L$（激光朝向） | ✅ 强 | 前进+后退补充 |
| $x_L, y_L$（激光位置） | ❌ 不可靠 | 旋转臂量级≈噪声，信噪比差 |

**因此：仅凭前进后退 + S形，可以可靠地同时标定 $(r_L, r_R, b_\text{eff}, \theta_L)$ 共 4 个参数，但无法可靠标定激光安装位置 $(x_L, y_L)$。**

#### 补救方案：增加原地旋转

加入原地旋转（$v \approx 0$，$\omega$ 大）后，旋转臂项主导：

$$\Delta\mathbf{p}^L \approx (I - R(\Delta\theta)) \cdot \mathbf{t}_L, \quad \Delta\theta \to 2\pi$$

积分一整圈：$\Delta\mathbf{p}^L = (I - I) \cdot \mathbf{t}_L = 0$（激光起点应归零），**任何非零平移残差直接反映 $\theta_L$ 误差下的测量噪声**，而半圈时旋转臂量级为 $2|\mathbf{t}_L|$，远超噪声，$(x_L, y_L)$ 变为**强可观**。

**推荐运动序列（按优先级排序）：**

```
1. 原地旋转 ±360° × 2次          → 强约束 (x_L, y_L)
2. 直线前进后退各 5m × 2次        → 强约束 r_L+r_R, θ_L
3. S形前进（曲率半径 0.5m/1.0m）  → 约束 r_L-r_R, b_eff
```

加入原地旋转后，6个参数**均可同时可靠标定**，推荐直接使用方案 B（非线性最小二乘联合优化）。

**步骤 1：机械测量 + 验证激光外参 $\boldsymbol{\xi}$**

通过 CAD 或卷尺获得 $(x_L, y_L, \theta_L)$ 初值，用以下方法验证：
- 机器人原地旋转 360°，激光扫描到固定障碍物
- 若 $\theta_L$ 正确，激光轨迹应为以 $(x_L, y_L)$ 为圆心的圆弧

**步骤 2：用激光 SE(2) 里程计作为"真值"，最小二乘标定轮参**

已知 $\boldsymbol{\xi}$，将激光相对位姿转换到机体系：

$$\mathbf{T}^B_k = \boldsymbol{\xi} \cdot \mathbf{T}^L_k \cdot \boldsymbol{\xi}^{-1}$$

从 $\mathbf{T}^B_k$ 提取 $(v_k^\text{gt}, \omega_k^\text{gt})$，联立线性方程：

**线速度方程（N组数据）：**

$$\underbrace{\begin{bmatrix} \omega_{L,1}/2 & \omega_{R,1}/2 \\ \vdots & \vdots \\ \omega_{L,N}/2 & \omega_{R,N}/2 \end{bmatrix}}_{\mathbf{A}_v} \begin{bmatrix} r_L \\ r_R \end{bmatrix} = \underbrace{\begin{bmatrix} v_1^\text{gt} \\ \vdots \\ v_N^\text{gt} \end{bmatrix}}_{\mathbf{b}_v}$$

**角速度方程（N组数据）：**

$$\underbrace{\begin{bmatrix} -\omega_{L,1} & \omega_{R,1} \\ \vdots & \vdots \\ -\omega_{L,N} & \omega_{R,N} \end{bmatrix}}_{\mathbf{A}_\omega} \begin{bmatrix} J_{21} \\ J_{22} \end{bmatrix} = \underbrace{\begin{bmatrix} \omega_1^\text{gt} \\ \vdots \\ \omega_N^\text{gt} \end{bmatrix}}_{\mathbf{b}_\omega}$$

两个独立的线性最小二乘问题，闭式解：

---

## 4. 标定实施流程（针对 W1）

```
1. 测量 wheel_track（卷尺），初始化 wheel_track_eff = wheel_track
2. 机械测量激光外参 (x_L, y_L, θ_L)
3. 采集标定数据（场地：空旷走廊）：
   a. 原地旋转 ±360° × 2次           → 强约束 (x_L, y_L)
   b. 直线前进后退各 5m × 2次         → 强约束 r, θ_L
   c. S形前进（曲率半径 0.5m/1.0m）   → 约束 b_eff
4. 对齐激光里程计与编码器时间戳
5. 运行方案 A 最小二乘求解 r、b_eff
6. 验证：回放标定数据，对比激光轨迹与编码器轨迹残差
7. 将结果写入 chassis_params.yaml
```

---

## 5. 代码参数对照表

| YAML 参数 | 物理量 | 默认值 |
|-----------|--------|--------|
| `wheel_radius` | $r$ (m)，左右相同 | 0.0865 |
| `wheel_track` | 几何轴距 (m) | 0.4390 |
| `wheel_track_eff` | $b_\text{eff}$ (m) | = wheel_track |

节点内计算：

```cpp
// RPM → m/s
v_l = left_rpm  * (2π/60) * wheel_radius
v_r = right_rpm * (2π/60) * wheel_radius
// 机体速度
v     = (v_r + v_l) / 2
omega = (v_r - v_l) / wheel_track_eff
```

## 附：尺度参数的分工（2026-09-03 定稿）

三个尺度参数只对应两个可观测量：线尺度 $k_v = r\cdot f_{odom}$，角尺度 $k_\omega = f_{odom}\cdot r / b_{eff}$。多出的一个自由度靠**约定**消除：

| 参数 | 角色 | 谁写 |
|---|---|---|
| `wheel_radius` $r$ | 物理常量，钉在名义值 | 人工，不参与标定 |
| `wheel_track_eff` $b_{eff}$ | 物理常量，钉在名义值 | 人工；本文 §4 的手动标定法仅作理论参考 |
| `odom_factor` $f_{odom}$ | **线尺度标定量** | `calib_odom_factor`：激光位移 / 里程计位移 |
| `imu_factor` | **角尺度标定量**（IMU 通路） | `calib_odom_factor`：激光 yaw / IMU yaw |

`odom_factor` 乘在**轮速层**（$v_l, v_r$ 各乘一次），而非只乘合成线速度：轮径误差对 $v$ 与 $\omega_{enc}$ 是同一个比例，只修 $v$ 会迫使 $b_{eff}$ 吸收轮径误差。`ChassisMotorFeedback.speed_mps` 与积分所用轮速为同一量，亦含该系数。

**已知缺口**：编码器角尺度中的 $b_{eff}$ 无标定项。当前 $\omega$ 以 IMU 为主，编码器 $\omega$ 仅用于 IMU 失效回退与 EKF 新息；若需补齐，形态与 `imu_factor` 标定相同（原地转 N 圈，激光 yaw / 编码器 yaw）。
