#pragma once
// 里程计运行模式：单参数四选一，替代原 use_imu_yaw / use_ekf_fusion 两个布尔的隐式组合。
// 不依赖 ROS，可在 gtest 中单独测试。
//
// IMU 在任何模式下都只提供航向/角速度，位置始终来自编码器——
// 平面运动下加速度二次积分撑不过 1 s，「纯 IMU」在底盘上没有定义。

#include <optional>
#include <string>

namespace chassis_node {

enum class OdomMode {
    EncoderOnly,  // 纯编码器：v 与 θ 都来自编码器
    ImuYaw,       // IMU 航向：v 来自编码器，θ 由陀螺积分（非 EKF 直接积分）
    Ekf,          // EKF 融合：IMU 驱动预测，编码器 (v, ω) 做量测与检测
    EkfNoImu,     // EKF 无 IMU：匀速假设预测 + 编码器 2 维更新——IMU 掉线时 EKF 自动退化到的状态
};

inline const char* OdomModeName(OdomMode m) {
    switch (m) {
        case OdomMode::EncoderOnly: return "encoder_only";
        case OdomMode::ImuYaw:      return "imu_yaw";
        case OdomMode::Ekf:         return "ekf";
        case OdomMode::EkfNoImu:    return "ekf_no_imu";
    }
    return "unknown";
}

inline std::optional<OdomMode> ParseOdomMode(const std::string& s) {
    if (s == "encoder_only") return OdomMode::EncoderOnly;
    if (s == "imu_yaw")      return OdomMode::ImuYaw;
    if (s == "ekf")          return OdomMode::Ekf;
    if (s == "ekf_no_imu")   return OdomMode::EkfNoImu;
    return std::nullopt;
}

inline constexpr const char* kOdomModeChoices = "encoder_only | imu_yaw | ekf | ekf_no_imu";

/// 该模式是否消费 IMU（航向/角速度）
inline bool ModeUsesImu(OdomMode m) { return m == OdomMode::ImuYaw || m == OdomMode::Ekf; }
/// 该模式是否走 EKF 路径
inline bool ModeUsesEkf(OdomMode m) { return m == OdomMode::Ekf || m == OdomMode::EkfNoImu; }

}  // namespace chassis_node
