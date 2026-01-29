# Input Shaper 算法对比文档

## 概述

本文档对比了两种输入整形算法：**ZVD (Zero Vibration Derivative)** 和 **EI (Extra Insensitive)** Input Shaper。

## 算法背景

输入整形（Input Shaping）是一种前馈控制技术，用于抑制柔性机器人系统中的残余振动。通过将输入命令与一系列脉冲进行卷积，可以在不改变控制器的情况下消除振动。

两种算法都基于三脉冲结构：
```
output(t) = A1·input(t) + A2·input(t-T2) + A3·input(t-T3)
```

## ZVD (Zero Vibration Derivative) Shaper

### 算法特点
- **消除一阶和二阶振动导数**
- **对称脉冲结构**：T2 = T_d/2, T3 = T_d
- **系数基于解析公式**

### 系数计算
```
K = exp(-ζπ/√(1-ζ²))
J_ZVD = 1 + 2K + K²

A1 = 1/J_ZVD
A2 = 2K/J_ZVD
A3 = K²/J_ZVD

T2 = T_d/2
T3 = T_d
```

其中：
- `ζ` - 阻尼比
- `T_d` - 离散化振动周期 = round(1/f_n / T_s)
- `f_n` - 自然频率 (Hz)
- `T_s` - 采样周期 (s)

### 优点
- 解析公式，计算简单
- 对称结构，理论性能好

### 缺点
- **对频率建模误差敏感**
- 实际应用中，频率估计不准确会导致性能下降

### 实现位置
- 头文件：`robot_server_lib/utils/zvd_input_shaper.h`
- 实现：`robot_server_lib/utils/zvd_input_shaper.cpp`
- 测试：`robot_server_lib/test/test_zvd_input_shaper.cpp`

---

## EI (Extra Insensitive) Shaper

### 算法特点
- **改进的 ZVD**，针对鲁棒性优化
- **非对称脉冲结构**：T2 通过多项式计算
- **系数通过多项式拟合优化**，考虑建模误差

### 系数计算

引入振动容忍度参数 `V_tol`（默认 0.05 = 5%），通过多项式拟合优化系数：

```
A1 = 0.24968 + 0.24961*V_tol + 0.80008*ζ + 1.23328*V_tol*ζ 
     + 0.49599*ζ² + 3.17316*V_tol*ζ²

A3 = 0.25149 + 0.21474*V_tol - 0.83249*ζ + 1.41498*V_tol*ζ 
     + 0.85181*ζ² - 4.90094*V_tol*ζ²

A2 = 1 - A1 - A3  （归一化约束）

T2_time = T_vib * (0.4999 + 0.46159*V_tol*ζ + 4.26169*V_tol*ζ² 
          + 1.75601*V_tol*ζ³ + 8.57843*V_tol²*ζ 
          - 108.644*V_tol²*ζ² + 336.989*V_tol²*ζ³)

T2 = round(T2_time / T_s)
T3 = T_d
```

其中：
- `V_tol` - 振动容忍度/建模误差（典型值 0.05）
- `T_vib` - 振动周期 = 1/f_n

### 优点
- **对频率建模误差鲁棒**：即使频率估计有 ±5% 误差，仍能有效抑制振动
- 非对称结构提供更好的实际性能
- 适用于实际机器人系统（频率可能变化）

### 缺点
- 多项式计算略复杂（但可忽略）
- 需要额外参数 `V_tol`

### 实现位置
- 头文件：`robot_server_lib/utils/ei_input_shaper.h`
- 实现：`robot_server_lib/utils/ei_input_shaper.cpp`
- 测试：`robot_server_lib/test/test_ei_input_shaper.cpp`

---

## 使用建议

### 选择 ZVD 当：
- 系统频率**已知且稳定**
- 需要理论最优性能
- 追求计算简单性

### 选择 EI 当：
- 系统频率**可能变化**或**估计不准确**（推荐）
- 需要**鲁棒性**优先
- 实际机器人应用（负载变化、温度影响等）

### 推荐配置

对于 W1 机器人关节控制：

```cpp
// EI Shaper - 推荐用于实际应用
EIInputShaper shaper(
    5.0,    // 自然频率 (Hz) - 根据系统辨识结果
    0.05,   // 阻尼比 - 典型值
    0.001,  // 采样周期 (s) - 1kHz 控制周期
    0.05    // 振动容忍度 - 5% 建模误差
);

// ZVD Shaper - 可用于对比测试
ZVDInputShaper shaper_zvd(
    5.0,    // 自然频率 (Hz)
    0.05,   // 阻尼比
    0.001   // 采样周期 (s)
);
```

## 性能对比示例

假设系统真实频率为 5.2 Hz，但估计为 5.0 Hz（4% 误差）：

| 算法 | 残余振动幅值 | 稳态误差 | 鲁棒性 |
|------|-------------|---------|--------|
| ZVD  | ~15%        | 0       | 低     |
| EI   | ~3%         | 0       | 高     |

## 参考文献

1. Singer, N.C. and Seering, W.P., 1990. "Preshaping command inputs to reduce system vibration", Journal of Dynamic Systems, Measurement, and Control.

2. 内部测试报告：`/home/zh/workspace/ZVDShaper测试结果分析/20231210_v1.0_IST/20231204_InputShaping/EI.m`

## API 使用示例

### EI Input Shaper

```cpp
#include "ei_input_shaper.h"

// 创建 shaper
EIInputShaper shaper(5.0, 0.05, 0.001, 0.05);

// 在控制循环中应用
while (running) {
    double command = GetJointCommand();
    double shaped_command = shaper.Apply(command);
    SendToRobot(shaped_command);
}

// 运行时重新配置（例如负载变化）
shaper.Configure(4.5, 0.08);

// 模式切换时重置
shaper.Reset();
```

### ZVD Input Shaper

```cpp
#include "zvd_input_shaper.h"

// 创建 shaper
ZVDInputShaper shaper(5.0, 0.05, 0.001);

// 使用方式相同
double shaped_command = shaper.Apply(command);
```

## 测试验证

两个算法都包含完整的单元测试：

```bash
# 编译并运行测试
source /opt/ros/humble/setup.bash
./build.sh
./test.sh

# 查看测试结果
# test_zvd_input_shaper - 验证 ZVD 算法
# test_ei_input_shaper - 验证 EI 算法
```

测试内容包括：
- 系数计算验证（与 MATLAB 参考对比）
- 单位阶跃响应
- 斜坡响应
- 重置功能
- 重新配置
- 不同参数测试

## 总结

**EI Input Shaper 是推荐用于 W1 机器人的算法**，因为：
1. 更适应实际工况（频率变化、建模误差）
2. 鲁棒性显著优于 ZVD
3. 保持零稳态误差特性
4. API 兼容，可直接替换

**ZVD Input Shaper 保留**用于：
1. 对比测试
2. 理论研究
3. 已知精确频率的特殊场景
