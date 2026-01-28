# ZVD Input Shaper 算法文档

## 1. 概述

ZVD (Zero Vibration Derivative) Input Shaper 是一种前馈控制技术，用于抑制机器人关节运动中的残余振动。通过将输入命令与一系列脉冲进行卷积，可以消除系统的一阶和二阶振动导数，从而显著减少振动幅值和稳定时间。

## 2. 算法原理

### 2.1 输入整形基本概念

**输入整形（Input Shaping）** 的核心思想是：将原始输入命令与一系列精心设计的脉冲进行卷积，使得系统的振动响应相互抵消。

**数学表达：**
```
u_shaped(t) = Σ A_i · u(t - T_i)
```

其中：
- `u(t)` 是原始输入命令
- `u_shaped(t)` 是整形后的输入命令
- `A_i` 是脉冲幅值
- `T_i` 是脉冲时间延迟

### 2.2 ZVD 算法

ZVD 使用三个脉冲来消除振动的零阶、一阶和二阶导数：

```
output(t) = A1·input(t) + A2·input(t-T2) + A3·input(t-T3)
```

**系数计算公式：**

1. 计算 K 因子：
   ```
   K = exp(-ζπ/√(1-ζ²))
   ```
   其中 ζ 是阻尼比

2. 计算归一化因子：
   ```
   J_ZVD = 1 + 2K + K²
   ```

3. 计算脉冲幅值：
   ```
   A1 = 1/J_ZVD
   A2 = 2K/J_ZVD
   A3 = K²/J_ZVD
   ```

4. 计算时间延迟：
   ```
   T_d = round(T_vibration / T_sampling)
   T2 = T_d/2
   T3 = T_d
   ```
   其中 T_vibration = 1/f_natural（振动周期）

**关键特性：**
- 系数归一化：A1 + A2 + A3 = 1（保证稳态增益为 1）
- 对称性：A2 = 2·√(A1·A3)
- 延迟：T_d = 一个振动周期

### 2.3 振动抑制机理

ZVD 通过三个脉冲的相位关系来抵消振动：

1. **第一个脉冲（A1）**：在 t=0 时刻激发振动
2. **第二个脉冲（A2）**：在 t=T2 时刻（半个周期后）激发反相振动
3. **第三个脉冲（A3）**：在 t=T3 时刻（一个周期后）进一步抵消残余振动

三个脉冲的振动响应叠加后，振动的幅值、速度和加速度都趋于零。

## 3. 实现细节

### 3.1 类结构

```cpp
class ZVDInputShaper {
public:
    ZVDInputShaper(double natural_frequency_hz,
                   double damping_ratio,
                   double sampling_time_sec = 0.001);

    double Apply(double input);
    void Reset();
    void Configure(double natural_frequency_hz, double damping_ratio);

    int GetDelaySamples() const;
    double GetDelaySeconds() const;
    bool IsInitialized() const;

private:
    void CalculateCoefficients();

    double A1_, A2_, A3_;           // 脉冲幅值
    int T2_, T3_, T_d_;             // 时间延迟（采样点数）
    double natural_frequency_hz_;   // 自然频率
    double damping_ratio_;          // 阻尼比
    double sampling_time_;          // 采样周期
    double K_;                      // ZVD 计算因子
    std::deque<double> input_history_;  // 输入历史缓冲区
    bool initialized_;              // 初始化标志
};
```

### 3.2 系数计算实现

```cpp
void ZVDInputShaper::CalculateCoefficients() {
    // 计算 K 因子
    double sqrt_term = std::sqrt(1.0 - damping_ratio_ * damping_ratio_);
    K_ = std::exp(-damping_ratio_ * M_PI / sqrt_term);

    // 计算归一化因子
    double J_ZVD = 1.0 + 2.0 * K_ + K_ * K_;

    // 计算脉冲幅值
    A1_ = 1.0 / J_ZVD;
    A2_ = 2.0 * K_ / J_ZVD;
    A3_ = K_ * K_ / J_ZVD;

    // 计算时间延迟
    double vibration_period = 1.0 / natural_frequency_hz_;
    T_d_ = static_cast<int>(std::round(vibration_period / sampling_time_));
    T2_ = T_d_ / 2;
    T3_ = T_d_;

    // 调整缓冲区大小
    input_history_.resize(T3_ + 1, 0.0);
}
```

### 3.3 信号整形实现

```cpp
double ZVDInputShaper::Apply(double input) {
    // 首次调用：用输入值填充缓冲区
    if (!initialized_) {
        std::fill(input_history_.begin(), input_history_.end(), input);
        initialized_ = true;
        return input;
    }

    // 更新 FIFO 缓冲区
    input_history_.pop_front();
    input_history_.push_back(input);

    // 应用 ZVD 整形公式
    double output = A1_ * input_history_[T3_] +           // 当前输入
                    A2_ * input_history_[T3_ - T2_] +     // 延迟 T2
                    A3_ * input_history_[0];              // 延迟 T3

    return output;
}
```

### 3.4 初始化策略

**首次调用处理：**
- 用第一个输入值填充整个历史缓冲区
- 第一个输出直接等于输入（避免瞬态）
- 后续调用正常应用整形公式

**设计理由：**
- 避免启动瞬态：如果缓冲区初始化为 0，会产生大的瞬态响应
- 快速收敛：用输入值填充使整形器立即进入合理状态
- 无跳变：第一个输出等于输入，保证连续性

### 3.5 重置功能

```cpp
void ZVDInputShaper::Reset() {
    if (!initialized_) return;
    initialized_ = false;
    std::fill(input_history_.begin(), input_history_.end(), 0.0);
}
```

重置后，下次 `Apply()` 调用会重新初始化。

## 4. 参数配置

### 4.1 自然频率（natural_frequency_hz）

**定义**：系统的共振频率（Hz）

**如何确定：**
1. **系统辨识**：通过阶跃响应或频率扫描测试
2. **经验值**：
   - 机器人手臂：3-10 Hz
   - 轻型机械臂：5-15 Hz
   - 重型机械臂：2-5 Hz

**影响：**
- 决定延迟大小：T_d = 1/f_natural / T_sampling
- 例如：5 Hz @ 1 kHz = 200 采样点 = 0.2 秒延迟

**注意事项：**
- ⚠️ 错误的频率会导致振动恶化而非抑制
- ⚠️ 必须通过实际测试确定，不能随意猜测

### 4.2 阻尼比（damping_ratio）

**定义**：系统的阻尼特性（无量纲，0-1）

**典型值：**
- 0.01-0.05：轻阻尼系统（振动明显）
- 0.05-0.1：中等阻尼系统（常见）
- 0.1-0.3：重阻尼系统（振动较小）

**影响：**
- 决定 K 因子和脉冲幅值分布
- 较小的阻尼比对应更强的振动抑制效果

**推荐值：**
- 初始值：0.05
- 调整范围：0.03-0.1

### 4.3 采样周期（sampling_time_sec）

**定义**：控制循环的采样周期（秒）

**典型值：**
- 0.001 s（1 kHz）：标准控制频率
- 0.0005 s（2 kHz）：高速控制
- 0.002 s（500 Hz）：低速控制

**注意事项：**
- 必须与实际控制循环频率匹配
- 影响延迟的绝对时间值

## 5. 性能特性

### 5.1 延迟特性

**延迟计算：**
```
T_delay = 1 / f_natural
```

**示例：**
- 5 Hz @ 1 kHz：200 采样点 = 0.2 秒
- 10 Hz @ 1 kHz：100 采样点 = 0.1 秒
- 3 Hz @ 1 kHz：333 采样点 = 0.333 秒

**影响：**
- 延迟会影响系统的响应速度
- 需要在振动抑制和响应速度之间权衡

### 5.2 振动抑制效果

**理论性能：**
- 振动幅值减少：> 90%
- 稳定时间减少：> 50%
- 超调量减少：> 80%

**实际性能：**
- 取决于参数准确性（特别是自然频率）
- 频率误差 ±10% 时仍有良好效果
- 频率误差 > 20% 时效果显著下降

### 5.3 计算复杂度

- **时间复杂度**：O(1)（3 次乘法和 2 次加法）
- **空间复杂度**：O(T_d)（历史缓冲区大小）
- **执行时间**：< 5 μs（在现代处理器上）

### 5.4 数值稳定性

- 使用 `std::deque` 管理历史缓冲区
- 双精度浮点运算（`double`）保证精度
- 系数归一化保证数值稳定性

## 6. 应用场景

### 6.1 在机器人控制中的作用

**主要用途：**
1. **残余振动抑制**：消除运动结束后的振动
2. **快速稳定**：减少到达目标位置的稳定时间
3. **精确定位**：提高最终位置精度

**典型应用场景：**
- 点到点运动后的振动抑制
- 高速运动中的振动控制
- 精密装配任务中的稳定性提升

### 6.2 与 Butterworth Filter 的对比

| 特性 | ZVD Input Shaper | Butterworth Filter |
|------|------------------|-------------------|
| **目标** | 抑制残余振动 | 抑制高频噪声 |
| **原理** | 时域整形 | 频域滤波 |
| **延迟** | 大（一个振动周期） | 小（约 10-20 ms） |
| **频率选择性** | 针对特定频率 | 平滑衰减 |
| **参数调整** | 需要系统辨识 | 固定（3 Hz） |
| **适用场景** | 特定频率振动抑制 | 通用噪声抑制 |
| **鲁棒性** | 对频率误差敏感 | 鲁棒性好 |

### 6.3 使用建议

**何时使用 ZVD：**
- ✅ 振动是主要问题
- ✅ 已知系统自然频率
- ✅ 可以接受延迟
- ✅ 需要快速稳定

**何时不使用 ZVD：**
- ❌ 延迟不可接受
- ❌ 自然频率未知或变化
- ❌ 振动不明显
- ❌ 需要实时响应

## 7. 参数调优指南

### 7.1 系统辨识方法

**方法 1：阶跃响应测试**
1. 给系统一个阶跃输入
2. 记录输出响应
3. 测量振动周期 T_vibration
4. 计算自然频率：f_natural = 1 / T_vibration
5. 从振动衰减估计阻尼比

**方法 2：频率扫描测试**
1. 输入不同频率的正弦信号
2. 测量输出幅值
3. 找到共振峰对应的频率
4. 该频率即为自然频率

### 7.2 参数调整流程

**步骤 1：初始配置**
```cpp
// 保守的初始值
ZVDInputShaper shaper(5.0, 0.05, 0.001);
```

**步骤 2：测试和观察**
- 执行点到点运动
- 观察振动情况
- 记录稳定时间

**步骤 3：调整自然频率**
- 如果振动未减少：调整频率 ±20%
- 如果振动恶化：频率设置错误，需要重新辨识
- 如果振动减少但未消除：微调频率 ±5%

**步骤 4：调整阻尼比**
- 增大阻尼比：更保守，鲁棒性更好
- 减小阻尼比：更激进，抑制效果更强

### 7.3 常见问题排查

**问题 1：振动没有减少**
- 原因：自然频率设置错误
- 解决：重新进行系统辨识

**问题 2：振动恶化**
- 原因：自然频率严重偏离实际值
- 解决：禁用 ZVD，重新辨识

**问题 3：延迟过大**
- 原因：自然频率过低
- 解决：确认频率是否正确，或考虑不使用 ZVD

**问题 4：效果不稳定**
- 原因：系统自然频率随负载变化
- 解决：使用自适应参数或保守的阻尼比

## 8. 验证与测试

### 8.1 单元测试覆盖

实现了 9 个单元测试：
1. ✅ 系数计算验证（与 MATLAB 参考一致）
2. ✅ 延迟计算验证
3. ✅ 单位阶跃响应（稳态值 = 1.0）
4. ✅ 零输入响应
5. ✅ 斜坡响应（恒定延迟）
6. ✅ 重置功能
7. ✅ 重新配置功能
8. ✅ 不同频率参数测试
9. ✅ 初始化状态测试

### 8.2 系数验证结果

使用参数 (5 Hz, 0.05, 0.001s)：
- K = 0.854468 ✓
- A1 = 0.290778 ✓
- A2 = 0.496921 ✓
- A3 = 0.212301 ✓
- A1 + A2 + A3 = 1.0 ✓
- T_d = 200 采样点 (0.2 秒) ✓

### 8.3 响应特性验证

- ✅ 单位阶跃稳态值 = 1.0（无稳态误差）
- ✅ 斜坡响应有恒定延迟（无相位失真）
- ✅ 系数归一化（增益 = 1）

## 9. 使用指南

### 9.1 基本使用

```cpp
#include "zvd_input_shaper.h"

// 创建 shaper（5Hz 自然频率，0.05 阻尼比，1ms 采样周期）
ZVDInputShaper shaper(5.0, 0.05, 0.001);

// 在控制循环中应用整形
for (int i = 0; i < num_samples; ++i) {
    double input = get_position_command();
    double output = shaper.Apply(input);
    send_to_robot(output);
}
```

### 9.2 查询延迟信息

```cpp
int delay_samples = shaper.GetDelaySamples();  // 例如: 200
double delay_seconds = shaper.GetDelaySeconds();  // 例如: 0.2
std::cout << "ZVD 延迟: " << delay_seconds << " 秒" << std::endl;
```

### 9.3 模式切换时重置

```cpp
if (control_mode_changed) {
    shaper.Reset();
}
```

### 9.4 运行时重新配置

```cpp
// 根据系统辨识结果调整参数
shaper.Configure(6.0, 0.08);  // 新的自然频率和阻尼比
```

### 9.5 与 Butterworth 级联使用

```cpp
// 先应用 Butterworth 滤除噪声
double filtered = butterworth_filter.Apply(position_command);

// 再应用 ZVD 抑制振动
double shaped = zvd_shaper.Apply(filtered);

send_to_robot(shaped);
```

## 10. 设计权衡

### 10.1 延迟 vs 振动抑制

**权衡：**
- 更高的自然频率：延迟更小，但可能不匹配实际振动频率
- 更低的自然频率：延迟更大，但如果匹配实际频率则效果更好

**建议：**
- 优先保证频率准确性
- 延迟是必要的代价

### 10.2 鲁棒性 vs 性能

**权衡：**
- 较大的阻尼比：对频率误差更鲁棒，但抑制效果稍弱
- 较小的阻尼比：抑制效果更强，但对频率误差更敏感

**建议：**
- 初期使用较大阻尼比（0.08-0.1）
- 确认效果后可以减小阻尼比（0.03-0.05）

### 10.3 缓冲区实现

**当前实现：std::deque**

**优点：**
- 实现简单
- 自动内存管理
- 动态大小调整

**替代方案：**
- `std::array`：无堆分配，但需编译时确定大小
- 循环缓冲区：最优性能，但实现复杂

**建议：**
- 初期使用 `std::deque`
- 如性能分析显示瓶颈再优化

## 11. 文件位置

- 头文件: `robot_server_lib/utils/zvd_input_shaper.h`
- 实现文件: `robot_server_lib/utils/zvd_input_shaper.cpp`
- 单元测试: `robot_server_lib/test/test_zvd_input_shaper.cpp`

## 12. 参考资料

### 12.1 原始论文

- Singer, N. C., & Seering, W. P. (1990). "Preshaping command inputs to reduce system vibration". Journal of Dynamic Systems, Measurement, and Control, 112(1), 76-82.

### 12.2 相关文献

- Singhose, W. E., Seering, W. P., & Singer, N. C. (1994). "Residual vibration reduction using vector diagrams to generate shaped inputs". Journal of Mechanical Design, 116(2), 654-659.

## 13. 未来改进方向

### 13.1 自适应参数

根据负载或运动状态自动调整参数：
```cpp
void AdaptParameters(double current_load);
```

### 13.2 多模态振动抑制

如果系统有多个共振频率，可以级联多个 ZVD shaper：
```cpp
ZVDInputShaper shaper1(5.0, 0.05, 0.001);   // 第一个共振频率
ZVDInputShaper shaper2(12.0, 0.05, 0.001);  // 第二个共振频率
```

### 13.3 在线系统辨识

实时估计系统自然频率，自动更新 ZVD 参数。

### 13.4 ZVD 变体

- **ZV (Zero Vibration)**：仅两个脉冲，延迟更小但鲁棒性较差
- **ZVDD (Zero Vibration Double Derivative)**：四个脉冲，鲁棒性更好但延迟更大
- **EI (Extra Insensitive)**：对频率误差更鲁棒

---

**文档版本**: 1.0
**最后更新**: 2026-01-28
**作者**: Zhang Yinan
