/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <spdlog/fmt/fmt.h>

#include <cstdint>
#include <vector>

#include "dexe_rp/common/CommonHeader.h"

namespace DexeRP {
namespace Kinematics {

struct ArmConfig {
public:
    enum class Direction : int { Pos = 1, Neg = -1 };

    static constexpr Direction Pos = Direction::Pos;  // can access ArmConfig::Pos
    static constexpr Direction Neg = Direction::Neg;  // can access ArmConfig::Neg

    ArmConfig() : shoulder_{Direction::Pos}, elbow_{Direction::Pos}, wrist_{Direction::Pos} {}
    constexpr ArmConfig(Direction s, Direction e, Direction w) : shoulder_{s}, elbow_{e}, wrist_{w} {}
    virtual ~ArmConfig() = default;

    inline double Shoulder() const { return static_cast<double>(shoulder_); }
    inline double Elbow() const { return static_cast<double>(elbow_); }
    inline double Wrist() const { return static_cast<double>(wrist_); }

protected:
    Direction shoulder_;
    Direction elbow_;
    Direction wrist_;
};

inline std::ostream& operator<<(std::ostream& os, const ArmConfig& cfg) {
    os << "ArmConfig(" << cfg.Shoulder() << ", " << cfg.Elbow() << ", " << cfg.Wrist() << ")";
    return os;
}

}  // namespace Kinematics
}  // namespace DexeRP

// fmt::formatter specialization for ArmConfig
template <>
struct fmt::formatter<DexeRP::Kinematics::ArmConfig> {
    // Parse format specifications
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const DexeRP::Kinematics::ArmConfig& cfg, FormatContext& ctx) {
        return fmt::format_to(ctx.out(), "ArmConfig({}, {}, {})", cfg.Shoulder(), cfg.Elbow(), cfg.Wrist());
    }
};
