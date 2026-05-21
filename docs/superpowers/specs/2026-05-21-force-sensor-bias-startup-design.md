# 力传感器零位校准启动流程重构 — 设计文档

日期：2026-05-21
状态：待实现

## 背景

当前实现存在时序错误：

- `RobotServer` 构造函数中调用 `primary_sensor_->Init(460800)`，该方法同步阻塞跑零位校准（500ms 采样 + 最多 2s 等待，见 `robot_server_lib/utils/sri_force_sensor.cpp:379-441`）。
- 校准期间，每帧调用 `ComputeToolGravityWrenchEE()` 来从原始读数中减去工具重力，依赖 `manager_info_.ee_pose_world_measurement` 提供的 EE 旋转 R_world_from_EE。
- 但 `Run()` 在 `Init()` 之后才被调用，`PlannerManager` 线程此时还未启动，`ee_pose_world_measurement` 为零向量；`GetEERotationWorldFromEE()` 在长度不足时返回单位矩阵 I（见 `w1_server/src/w1_server.cpp:169-183`）。
- 结果：校准期间用 R = I 减重力，残留误差被吸收进零偏。如果机器人启动姿态每次不同，零偏质量会随姿态飘移。

## 目标

把零偏结算推迟到 EE 位姿首次有效之后，确保减重力时使用的是真实姿态。期望的启动顺序：

1. 力传感器启动 + 开始记录原始（已映射到 EE 系的）样本
2. 关节反馈到位 → PlannerManager 写入 `ee_pose_world_measurement`
3. 用真实 R 从样本均值中减去工具重力 → 得到纯零偏
4. 运行时按实时 R 做工具重力补偿
5. 正常运动控制

## 设计概要

把"延迟结算"逻辑封装在 `SriForceSensor` 内部。`Init()` 不再阻塞等待校准；新增 `TryFinalizeBias()` 由控制环每个 tick 主动调用，传感器自己判断条件并完成结算。

### 设计原则

- **传感器自包含**：所有校准状态（累加器、采样数、计时、结算条件）都在 `SriForceSensor` 内，对外只暴露"已就绪"与"未就绪"两种状态
- **静止前提**：校准窗口期间机器人静止——这是机械臂上电后由调用方保证的现场约束
- **无限等待**：EE 位姿迟迟不来时，零偏永远不就绪，`primary_sensor_ready_` 保持 false，力相关功能不可用，但不阻塞系统其他部分
- **配置不变**：`tool_gravity_compensation` yaml 段、`force_sensor_bias` JSON 字段、补偿运行时行为均维持原状

## 状态机

```
[Init]
  开串口 + 启读线程 + 清空累加器 + 重置 mZeroOffsetValid=false
  立即返回（不阻塞）
   │
   ▼
[Sampling]  ← 读取线程持续累加 raw_ee 到 mRawSumEE，递增 mCalibrationSampleCount
  │
  │ 主线程每 tick 调 TryFinalizeBias()
  ▼
[CheckConditions]  采样数 ≥ MIN_SAMPLES 且 EE 旋转 provider 返回 valid R
  │
  │ 是 → 进入 Finalize；否 → 继续 Sampling
  ▼
[Finalize]
  bias_ee = (mRawSumEE / mCalibrationSampleCount) - tool_gravity_wrench_ee(R_now)
  mZeroOffset[i] = bias_ee[i]
  mZeroOffsetValid = true
  mIsCalibrating = false
  停止累加（可选：保留计数用于状态展示）
   │
   ▼
[Ready]
  PeekLatest 返回 raw_ee - mZeroOffset
  控制环按实时 R 做 ApplyToolGravityCompensation
  primary_sensor_ready_ 翻为 true
  JSON 中 force_sensor_bias 字段反映纯零偏
```

`MIN_SAMPLES` 取 500ms × 数据率（SRI 力传感器 1 kHz → 500 帧）。

## 接口变化

### `SriForceSensor`

| 接口 | 变化 |
|------|------|
| `Init(baudrate)` | 不再阻塞等待校准；只做串口打开 + 启读线程 + 清零累加器；立即返回。返回值仅表示串口/线程是否启动成功 |
| `SetToolGravityCompensationForZeroCalibration(...)` | provider 从 `std::function<Matrix3d()>` 改为 `std::function<std::optional<Matrix3d>()>`。`nullopt` 表示 EE 位姿未就绪 |
| **新增** `bool TryFinalizeBias()` | 主线程每 tick 调用；条件满足→结算返回 true；之前已 finalize 或条件未满足→返回 false。线程安全（与读线程的累加并发） |
| `PeekLatest(out)` | 在 `mZeroOffsetValid == false` 时返回 false（让 `primary_wrench_valid` 自然为 false） |
| `PeekLatestRaw(out)` | 不变。raw 流随时可读 |
| `HasValidZeroOffset()` | 等价于 `mZeroOffsetValid`。语义略变：现在意味着"含真实 R 减重力后的纯零偏" |
| `IsCalibrating()` | 语义变为"已开始累加但未完成 finalize" |
| `GetZeroOffsetSampleCount()` | 返回 finalize 时使用的采样数；finalize 前返回当前累加数 |

### `RobotServer::GetEERotationWorldFromEE`

签名改为：

```cpp
std::optional<Eigen::Matrix3d> GetEERotationWorldFromEE(int arm_index) const;
```

`ee_pose_world_measurement.size() < offset+7` 时返回 `nullopt`，否则返回真实 R。`ApplyToolGravityCompensation()` 中如果当前没有 valid R，按"不做补偿"处理（透传 wrench）—— 但实际上零偏未就绪时 PeekLatest 已经返回 false，控制环根本走不到 ApplyToolGravityCompensation 这一步，所以这里的 fallback 只是保险。

### `RobotServer::TorqueControlThreadFunc`

新增一行（在读取传感器数据之前）：

```cpp
if (primary_sensor_) {
    primary_sensor_->TryFinalizeBias();
}
```

`primary_sensor_ready_` 的语义改为镜像 `primary_sensor_->HasValidZeroOffset()`，每 tick 刷新。`primary_wrench_valid` 判定保持原样：`PeekLatest` 在零偏未就绪时返回 false 已经够。

## 数据流

```
读线程（~1kHz）
  raw frame
    └─► MapSriRawToEE → wrench_ee_raw
            ├─► raw_buffer_ (PeekLatestRaw 用)
            ├─► [若 mIsCalibrating] mRawSumEE += wrench_ee_raw; mCalibrationSampleCount++
            └─► [若 mZeroOffsetValid] buffer_ ← wrench_ee_raw - mZeroOffset

主线程（torque control 1kHz）
  TryFinalizeBias()
    └─► 检查 mIsCalibrating && samples ≥ MIN && provider() == valid R
          └─► 加锁 → finalize → 解锁
  PeekLatest(zeroed_sensor_data_primary)  // valid 才进入下一步
  ApplyToolGravityCompensation(zeroed, R_now)
  ...
```

## 并发控制

- `mIsCalibrating`、`mZeroOffsetValid`、`mZeroOffset[]` 用 `std::atomic` 或单独 mutex 保护
- `mRawSumEE` / `mCalibrationSampleCount` 在累加（读线程）与读取并 finalize（主线程）之间需要互斥
- 选用 `std::mutex` + 短临界区：读线程累加时上锁；主线程 finalize 时上锁读累加器与计数，计算后写入 `mZeroOffset`、置 `mZeroOffsetValid=true`、`mIsCalibrating=false`
- `PeekLatest` 中读取 `mZeroOffset` 用 atomic flag + memory order 即可（finalize 一次性写入后不再变）

## 配置兼容

`tool_gravity_compensation.enabled = false` 时：
- 不需要等 EE 位姿，TryFinalizeBias 在累加 ≥ MIN_SAMPLES 后立即结算
- `bias = mean(raw_ee)`，不减重力
- 行为退化为"经典零位校准"

实现上：当 `mToolGravityCompensationForZeroEnabled == false`，TryFinalizeBias 跳过 R provider 检查。

## 测试计划

### 单元测试 / 桌面级
1. **静态 R = I + 重力补偿启用**：构造一个固定 raw 流（模拟 1kg 工具静止竖直挂在 EE 末端），provider 返回 I，500 帧后调 TryFinalizeBias，断言 bias_z ≈ raw_z + 9.79（即重力被减掉）
2. **R provider 一直 nullopt**：累加 1 秒后 TryFinalizeBias 仍返回 false，`mZeroOffsetValid == false`
3. **重力补偿 disabled**：500 帧后立刻 finalize，bias = mean(raw)
4. **finalize 后 PeekLatest 行为**：finalize 前返回 false；finalize 后返回 true 且为 raw - bias
5. **采样期间 provider 返回 nullopt，finalize 时刻才有效**：模拟"传感器先启动、EE 位姿后到位"——验证累加期间 provider 的返回值不影响 bias 结果，最终 bias 是用 finalize 时刻的 R 减重力得到的

### 集成 / 现场
1. 启动机器人，观察 LOG，确认时序：
   `Sensor Init returned` → `EE pose first valid (t=...)` → `Bias finalized (samples=..., bias=...)` → `primary_sensor_ready_` 翻 true
2. 用机械臂在不同初始姿态启动多次，断言纯零偏数值稳定（与姿态无关），不像旧实现那样飘
3. 校准结束后让机械臂手动姿态变化，观察 robot_server_state JSON 的 `force_sensor_bias` 字段保持稳定（纯零偏不应随姿态变）
4. 校准期间力相关功能应不可用（force_subscriber_ 注入的 `sensor_force_` 加成在 `primary_wrench_valid==false` 时 skip）

## 不影响的部分

- yaml 配置 `tool_gravity_compensation`（mass_kg / com_mm / gravity_magnitude / model_sign / enabled）
- 运行时 `ApplyToolGravityCompensation()` 的实现
- robot_server_state JSON schema（`force_sensor_bias` 字段已存在）
- `ForceArrayCallback` / `sensor_force_` 注入路径
- secondary_sensor_ 路径（目前未使用，但同样改造接口以保持对称）

## 风险

1. **校准期间机器人移动**：累加器混入动态分量。设计假设静止；现场约束。可选添加"采样方差监控"作为 future work，方差超阈值则告警。
2. **EE 位姿 valid 但姿态在累加期间显著变化**：finalize 用单一 R 减重力会有误差。同上，依赖静止假设。
3. **JSON 中 force_sensor_bias 在未就绪期间为 0 向量**：旧代码也是这种行为；监控/可视化端应配合 `force_sensor_bias_valid` 字段（如已有）使用。

## 文件改动清单

- `robot_server_lib/utils/sri_force_sensor.h` — 接口变更
- `robot_server_lib/utils/sri_force_sensor.cpp` — Init 不阻塞、TryFinalizeBias、并发保护、PeekLatest invalid 时返回 false
- `w1_server/src/w1_server.h` — `GetEERotationWorldFromEE` 返回类型
- `w1_server/src/w1_server.cpp` — 构造函数中 lambda 签名、TorqueControlThreadFunc 调 TryFinalizeBias、`primary_sensor_ready_` 语义
