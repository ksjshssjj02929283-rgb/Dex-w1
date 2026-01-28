# Butterworth 低通滤波器算法文档

## 1. 概述

Butterworth 滤波器是一种信号处理中的经典低通滤波器，以其平坦的通带频率响应而闻名。本实现采用二阶 Butterworth 低通滤波器，截止频率为 3Hz，用于机器人控制系统中的位置命令平滑。

## 2. 算法原理

### 2.1 数字滤波器结构

采用直接 II 型（Direct Form II）双二阶（Biquad）滤波器结构：

```
y[n] = b0·x[n] + b1·x[n-1] + b2·x[n-2] - a1·y[n-1] - a2·y[n-2]
```

其中：
- `x[n]` 是输入信号
- `y[n]` 是输出信号
- `b0, b1, b2` 是前馈系数
- `a1, a2` 是反馈系数（注意：a0 = 1）

### 2.2 滤波器参数

**设计参数：**
- 滤波器类型：Butterworth 低通
- 阶数：2
- 截止频率：3 Hz
- 采样频率：1000 Hz（1 kHz）

**系数值：**
```cpp
b = {8.765554875401464e-05, 0.00017531109750802928, 8.765554875401464e-05}
a = {1.0, -1.9733442497812987, 0.9736948719763148}
```

这些系数通过双线性变换（Bilinear Transform）从模拟滤波器设计得到。

### 2.3 频率响应特性

**幅频响应：**
- 通带（0-3Hz）：增益 ≈ 0 dB（增益 = 1）
- 截止频率（3Hz）：增益 ≈ -3 dB（增益 = 0.707）
- 阻带（>3Hz）：以 -40 dB/decade 衰减（二阶滤波器）

**相频响应：**
- 引入相位延迟，延迟量随频率增加而增加
- 在截止频率处相位延迟约 -90°

## 3. 实现细节

### 3.1 类结构

```cpp
class ButterworthFilter {
public:
    ButterworthFilter();           // 构造函数
    double Apply(double input);    // 应用滤波
    void Reset();                  // 重置状态

private:
    BiquadFilter filter_;                              // 滤波器系数
    std::array<double, FILTER_ORDER + 1> inputHistory_;   // 输入历史
    std::array<double, FILTER_ORDER + 1> outputHistory_;  // 输出历史
    bool initialized_;                                 // 初始化标志
};
```

### 3.2 初始化策略

首次调用 `Apply()` 时：
```cpp
if (!initialized_) {
    // 用第一个输入值填充所有历史缓冲区
    for (int i = 0; i <= FILTER_ORDER; ++i) {
        inputHistory_[i] = input;
        outputHistory_[i] = input;
    }
    initialized_ = true;
    return input;  // 第一个样本直接输出
}
```

**设计理由：**
- 避免启动瞬态：如果历史缓冲区初始化为 0，会产生大的瞬态响应
- 快速收敛：用输入值填充使滤波器立即进入合理状态
- 无跳变：第一个输出等于输入，保证连续性

### 3.3 滤波计算

```cpp
// 更新输入历史（FIFO）
for (int i = FILTER_ORDER; i > 0; --i) {
    inputHistory_[i] = inputHistory_[i - 1];
    outputHistory_[i] = outputHistory_[i - 1];
}
inputHistory_[0] = input;

// 计算输出
double y = 0.0;
for (int i = 0; i <= FILTER_ORDER; ++i) {
    y += filter_.b[i] * inputHistory_[i];
}
for (int i = 1; i <= FILTER_ORDER; ++i) {
    y -= filter_.a[i] * outputHistory_[i];
}
outputHistory_[0] = y;
return y;
```

### 3.4 重置功能

```cpp
void Reset() {
    if (!initialized_) return;
    initialized_ = false;
    inputHistory_.fill(0.0);
    outputHistory_.fill(0.0);
}
```

重置后，下次 `Apply()` 调用会重新初始化。

## 4. 性能特性

### 4.1 计算复杂度

- **时间复杂度**：O(1)（固定的 5 次乘法和 4 次加法）
- **空间复杂度**：O(1)（固定大小的历史缓冲区）
- **执行时间**：< 1 μs（在现代处理器上）

### 4.2 数值稳定性

- 使用 `std::array` 避免动态内存分配
- 双精度浮点运算（`double`）保证精度
- 直接 II 型结构在低阶滤波器中数值稳定

### 4.3 实时性能

- 无堆分配，适合实时控制循环
- 确定性执行时间
- 可在 1 kHz 控制循环中使用（占用 < 0.1% CPU）

## 5. 应用场景

### 5.1 在机器人控制中的作用

**主要用途：**
1. **位置命令平滑**：过滤运动规划器输出的位置命令中的高频噪声
2. **振动抑制**：减少控制命令中的高频分量，避免激发机械共振
3. **传感器信号滤波**：平滑传感器读数（如力传感器）

**典型应用位置：**
- w1_server 控制循环中的位置命令滤波（第 322 行）
- 力传感器数据预处理
- 速度估计中的噪声抑制

### 5.2 与 ZVD Input Shaper 的对比

| 特性 | Butterworth Filter | ZVD Input Shaper |
|------|-------------------|------------------|
| **目标** | 抑制高频噪声 | 抑制残余振动 |
| **原理** | 频域滤波 | 时域整形 |
| **延迟** | 小（约 10-20 ms） | 大（一个振动周期，如 200 ms） |
| **频率选择性** | 平滑衰减 | 针对特定频率 |
| **参数调整** | 固定（3 Hz） | 需要系统辨识 |
| **适用场景** | 通用噪声抑制 | 特定频率振动抑制 |

### 5.3 级联使用建议

**方案 A：仅 Butterworth**（推荐用于一般场景）
```cpp
position_filtered = butterworth_filter->Apply(position_command);
```
- 优点：延迟小，响应快
- 缺点：无法针对性抑制振动

**方案 B：仅 ZVD**（推荐用于振动严重场景）
```cpp
position_shaped = zvd_shaper->Apply(position_command);
```
- 优点：振动抑制效果好
- 缺点：延迟大，不滤除噪声

**方案 C：Butterworth + ZVD 级联**（推荐用于振动和噪声都存在的场景）
```cpp
position_filtered = butterworth_filter->Apply(position_command);
position_shaped = zvd_shaper->Apply(position_filtered);
```
- 优点：同时抑制噪声和振动
- 缺点：延迟累积

## 6. 设计权衡

### 6.1 截止频率选择（3 Hz）

**选择理由：**
- 机器人运动频率通常 < 2 Hz
- 3 Hz 截止频率保留运动命令，滤除高频噪声
- 避免过度滤波导致响应迟缓

**权衡：**
- 更低截止频率：更强的噪声抑制，但延迟更大
- 更高截止频率：延迟更小，但噪声抑制效果减弱

### 6.2 滤波器阶数（2 阶）

**选择理由：**
- 2 阶提供 -40 dB/decade 衰减，足够抑制高频噪声
- 计算量小，适合实时控制
- 相位延迟适中

**权衡：**
- 更高阶数：更陡峭的衰减，但相位延迟更大，计算量更大
- 更低阶数：计算更简单，但衰减不足

### 6.3 初始化策略

**当前策略：用输入值填充历史**

**优点：**
- 无启动瞬态
- 快速收敛
- 输出连续

**替代方案：**
- 历史清零：会产生瞬态，不推荐
- 预热期：需要额外逻辑，增加复杂度

## 7. 验证与测试

### 7.1 单元测试覆盖

实现了 9 个单元测试：
1. ✅ 初始化测试
2. ✅ 直流信号响应（稳态增益 = 1）
3. ✅ 零输入响应
4. ✅ 单位阶跃响应
5. ✅ 阶跃响应特性（单调性）
6. ✅ 重置功能
7. ✅ 高频噪声抑制（50 Hz 衰减 > 90%）
8. ✅ 低频信号通过（1 Hz 衰减 < 20%）
9. ✅ 输出连续性

### 7.2 频率响应验证

通过正弦波测试验证频率响应：
- 1 Hz 信号：通过（幅值保持 > 80%）
- 50 Hz 信号：衰减（幅值衰减 > 90%）

### 7.3 实际应用验证

在 w1_server 控制系统中的实际表现：
- 有效抑制位置命令中的高频噪声
- 不影响正常运动轨迹
- 无明显延迟感

## 8. 使用指南

### 8.1 基本使用

```cpp
#include "butterworth_filter.h"

// 创建滤波器
ButterworthFilter filter;

// 在控制循环中使用
for (int i = 0; i < num_samples; ++i) {
    double raw_position = get_position_command();
    double filtered_position = filter.Apply(raw_position);
    send_to_robot(filtered_position);
}
```

### 8.2 模式切换时重置

```cpp
if (control_mode_changed) {
    filter.Reset();
}
```

### 8.3 注意事项

1. **不要频繁重置**：重置会导致瞬态，仅在必要时（如模式切换）使用
2. **保持采样率一致**：滤波器设计基于 1 kHz 采样率
3. **避免输入跳变**：大的输入跳变会产生瞬态响应

## 9. 文件位置

- 头文件: `robot_server_lib/utils/butterworth_filter.h`
- 实现文件: `robot_server_lib/utils/butterworth_filter.cpp`
- 单元测试: `robot_server_lib/test/test_butterworth_filter.cpp`

## 10. 参考资料

- Butterworth, S. (1930). "On the Theory of Filter Amplifiers"
- Oppenheim, A. V., & Schafer, R. W. (2009). "Discrete-Time Signal Processing"
- 双线性变换方法：将模拟滤波器转换为数字滤波器的标准方法

## 11. 未来改进方向

### 11.1 可配置截止频率

当前截止频率固定为 3 Hz，可以扩展为：
```cpp
ButterworthFilter(double cutoff_frequency_hz, double sampling_rate_hz);
```

### 11.2 自适应滤波

根据运动速度自动调整截止频率：
- 高速运动：提高截止频率，减少延迟
- 低速运动：降低截止频率，增强噪声抑制

### 11.3 更高阶滤波器

对于噪声特别严重的场景，可以实现 4 阶或 6 阶 Butterworth 滤波器。

---

**文档版本**: 1.0
**最后更新**: 2026-01-28
**作者**: Zhang Yinan
