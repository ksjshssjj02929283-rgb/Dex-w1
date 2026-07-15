# Joint Analyzer 使用指南

Joint Analyzer 是一个集成化的机器人关节性能分析工具，支持 100Hz 实时控制指令下发与 1ms (1000Hz) 高精度数据采集。它通过 ROS 2 进行基础通信，并利用 System V 共享内存 (Shared Memory) 实现极低延迟的反馈监控。

## 1. 启动说明

在终端执行以下脚本启动工具：

```bash
cd dexe_robot_server
./set_w1.sh analyzer
```

*注：若需在模拟环境下运行，请添加 `--sim` 参数。*

## 2. 功能布局

界面分为左右两大区域：

### 左侧：控制与采集区 (Control & Acquisition)
*   **Data Acquisition (数据采集)**:
    *   **Record Path**: 设置数据保存路径（默认为 `../log/`）。
    *   **Trigger (触发机制)**: 支持“边沿触发”、“增量触发”或“无触发”。适用于捕捉电机启动或突发误差时的波形。
    *   **Duration**: 录制时长（秒）。
*   **Joint Control (关节控制)**:
    *   **Position Step**: 设置 +/- 按钮的步进步长（单位：rad）。
    *   **+/- 按钮**: 短按按步长移动；长按则以 100Hz 频率持续连续移动。
    *   **Advanced Controls (高级控制开关)**:
        *   **Ruckig**: 轨迹插值开关。
        *   **Filter**: 巴特沃斯滤波器开关。
        *   **ZVD**: 输入整形器（消抖）开关。
        *   **Drag Mode**: 拖动示教模式开关。
    *   **Clear Error**: 一键清除所有驱动器报警。

### 右侧：高精度波形分析区 (Plotting)
*   提供四个同步的时间序列图表：
    1.  **Position**: 指令值 vs 反馈值。
    2.  **Position Error**: 跟踪误差（反馈 - 指令）。
    3.  **Velocity**: 实时角速度记录。
    4.  **Torque**: 电机力矩反馈。
*   **工具栏**: 支持缩放、平移和存图。鼠标悬停在波形上可查看到 **1ms (0.001s)** 精度的时间戳及浮点数值。
