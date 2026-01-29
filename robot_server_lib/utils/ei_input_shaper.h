#pragma once

#include <cmath>
#include <deque>

/**
 * @brief EI (Extra Insensitive) Input Shaper
 *
 * 实现 EI 输入整形算法，是 ZVD 的改进版本，通过多项式优化系数提高对建模误差的鲁棒性。
 * EI shaper 对频率建模误差的容忍度更高，适用于实际机器人控制场景。
 *
 * 算法原理：
 *   output(t) = A1·input(t) + A2·input(t-T2) + A3·input(t-T3)
 *
 * 其中系数通过多项式拟合优化：
 *   A1 = 0.24968 + 0.24961*V_tol + 0.80008*ζ + 1.23328*V_tol*ζ
 *        + 0.49599*ζ² + 3.17316*V_tol*ζ²
 *   A3 = 0.25149 + 0.21474*V_tol - 0.83249*ζ + 1.41498*V_tol*ζ
 *        + 0.85181*ζ² - 4.90094*V_tol*ζ²
 *   A2 = 1 - A1 - A3
 *
 *   T2_time = T_vib * (0.4999 + 0.46159*V_tol*ζ + 4.26169*V_tol*ζ²
 *             + 1.75601*V_tol*ζ³ + 8.57843*V_tol²*ζ
 *             - 108.644*V_tol²*ζ² + 336.989*V_tol²*ζ³)
 *   T2 = round(T2_time / T_s)
 *   T3 = T_d
 *
 * 参数：
 *   V_tol - 振动容忍度/建模误差 (默认 0.05，即 5%)
 *   ζ - 阻尼比
 *   T_vib - 振动周期 = 1/f_n
 *   T_s - 采样周期
 *   T_d - 离散化振动周期 = round(T_vib/T_s)
 *
 * 参考：
 *   - Singer, N.C. and Seering, W.P., 1990. "Preshaping command inputs
 *     to reduce system vibration"
 *   - 内部测试报告：20231210_v1.0_IST/20231204_InputShaping/EI.m
 */
class EIInputShaper {
public:
    /**
     * @brief 构造函数
     * @param natural_frequency_hz 系统自然频率 (Hz)
     * @param damping_ratio 阻尼比 (无量纲, 0-1)
     * @param sampling_time_sec 采样周期 (秒)
     * @param vibration_tolerance 振动容忍度/建模误差 (默认 0.05 = 5%)
     */
    EIInputShaper(double natural_frequency_hz,
                  double damping_ratio,
                  double sampling_time_sec = 0.001,
                  double vibration_tolerance = 0.05);

    /**
     * @brief 应用 EI 整形到单个输入样本
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
     * @param vibration_tolerance 振动容忍度 (可选，默认保持不变)
     */
    void Configure(double natural_frequency_hz, double damping_ratio, double vibration_tolerance = -1.0);

    /**
     * @brief 查询当前延迟（采样点数）
     * @return 延迟的采样点数（T3）
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

    /**
     * @brief 获取当前系数（用于调试）
     */
    void GetCoefficients(double& A1, double& A2, double& A3, int& T2, int& T3) const;

private:
    /**
     * @brief 计算 EI 系数和时间延迟
     */
    void CalculateCoefficients();

    // EI 脉冲幅值
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
    double vibration_tolerance_;

    // 输入历史缓冲区
    std::deque<double> input_history_;

    // 初始化状态
    bool initialized_;
};
