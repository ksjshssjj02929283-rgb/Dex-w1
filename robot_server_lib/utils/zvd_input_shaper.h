#pragma once

#include <cmath>
#include <deque>

/**
 * @brief ZVD (Zero Vibration Derivative) Input Shaper
 *
 * 实现 ZVD 输入整形算法，用于抑制机器人关节运动中的残余振动。
 * 通过将输入命令与三个脉冲进行卷积，消除系统的一阶和二阶振动导数。
 *
 * 算法原理：
 *   output(t) = A1·input(t) + A2·input(t-T2) + A3·input(t-T3)
 *
 * 其中：
 *   K = exp(-ζπ/√(1-ζ²))
 *   J_ZVD = 1 + 2K + K²
 *   A1 = 1/J_ZVD, A2 = 2K/J_ZVD, A3 = K²/J_ZVD
 *   T_d = round(振动周期/采样周期)
 *   T2 = T_d/2, T3 = T_d
 *
 * 参考：1990_2303_Preshaping command inputs to reduce system vibration
 */
class ZVDInputShaper {
public:
    /**
     * @brief 构造函数
     * @param natural_frequency_hz 系统自然频率 (Hz)
     * @param damping_ratio 阻尼比 (无量纲, 0-1)
     * @param sampling_time_sec 采样周期 (秒)
     */
    ZVDInputShaper(double natural_frequency_hz, double damping_ratio, double sampling_time_sec = 0.001);

    /**
     * @brief 应用 ZVD 整形到单个输入样本
     * @param input 输入信号
     * @return 整形后的输出信号
     */
    double Apply(double input);

    /**
     * @brief 重置内部状态（用于模式切换）
     * 将缓冲区清零，下次 Apply 时重新初始化
     */
    void Reset();

    /**
     * @brief 运行时重新配置参数
     * @param natural_frequency_hz 系统自然频率 (Hz)
     * @param damping_ratio 阻尼比 (无量纲, 0-1)
     */
    void Configure(double natural_frequency_hz, double damping_ratio);

    /**
     * @brief 查询当前延迟（采样点数）
     * @return 延迟的采样点数
     */
    int GetDelaySamples() const;

    /**
     * @brief 查询当前延迟（秒）
     * @return 延迟时间（秒）
     */
    double GetDelaySeconds() const;

    /**
     * @brief 查询是否已初始化
     * @return true 如果已初始化
     */
    bool IsInitialized() const;

private:
    /**
     * @brief 计算 ZVD 系数和时间延迟
     */
    void CalculateCoefficients();

    // ZVD 脉冲幅值
    double A1_;
    double A2_;
    double A3_;

    // 时间延迟（采样点数）
    int T2_;
    int T3_;
    int T_d_;

    // 系统参数
    double natural_frequency_hz_;
    double damping_ratio_;
    double sampling_time_;

    // ZVD 计算因子
    double K_;

    // 输入历史缓冲区
    std::deque<double> input_history_;

    // 初始化状态
    bool initialized_;
};
