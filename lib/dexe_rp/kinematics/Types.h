/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
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

    static constexpr Direction Pos = Direction::Pos;  // 可访问ArmConfig::Pos
    static constexpr Direction Neg = Direction::Neg;  // 可访问ArmConfig::Neg

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

struct SRSConfig : public ArmConfig {
public:
    SRSConfig(Direction s, Direction e, Direction w, double arm_angle) : ArmConfig{s, e, w}, arm_angle{arm_angle} {}

    SRSConfig(const CVec7d& qpos, double arm_angle = 0.) {
        shoulder_ = qpos(1) > 0 ? Pos : Neg;
        elbow_ = qpos(3) > 0 ? Pos : Neg;
        wrist_ = qpos(5) > 0 ? Pos : Neg;
        this->arm_angle = arm_angle;
    }

    virtual ~SRSConfig() = default;

public:
    double arm_angle;
};

inline std::ostream& operator<<(std::ostream& os, const SRSConfig& cfg) {
    os << "SRSConfig(" << cfg.Shoulder() << ", " << cfg.Elbow() << ", " << cfg.Wrist() << ", " << cfg.arm_angle << ")";
    return os;
}

struct SRSKineParams {
    double bs, se, ew, wt;
    Mat4d T_b_ob;                   // base frame to srs_base frame
    Mat4d T_oe_e;                   // srs_ee frame to ee frame
    std::array<double, 7> offsets;  // joint offsets of SRS-model
    std::array<int, 7> axis_dirs;   // TODO: 暂时未使用
    CVec7d qpos_lb, qpos_ub;
    Mat<double, 7, 4> dh;

    SRSKineParams()
        : bs{},
          se{},
          ew{},
          wt{},
          T_b_ob{Mat4d::Identity()},
          T_oe_e{Mat4d::Identity()},
          offsets{},
          axis_dirs{1, 1, 1, 1, 1, 1, 1},
          qpos_lb{CVec7d::Zero()},
          qpos_ub{CVec7d::Zero()},
          dh(MatXd::Zero(7, 4)) {}

    SRSKineParams(double bs,
                  double se,
                  double ew,
                  double wt,
                  const CVec7d& qpos_lb,
                  const CVec7d& qpos_ub,
                  const std::array<double, 7>& offsets = {0, 0, 0, 0, 0, 0, 0},
                  const std::array<int, 7>& axis_dirs = {1, 1, 1, 1, 1, 1, 1},
                  const Mat4d& T_b_ob = Mat4d::Identity(),
                  const Mat4d& T_oe_e = Mat4d::Identity());
    virtual ~SRSKineParams() = default;
};

struct W1V02KineParams : public SRSKineParams {
    bool is_left;
    bool is_anthropmorphic;

    W1V02KineParams(bool is_left, bool is_anthropmorphic);
    virtual ~W1V02KineParams() = default;
};

}  // namespace Kinematics
}  // namespace DexeRP

// fmt::formatter模板特化
template <>
struct fmt::formatter<DexeRP::Kinematics::ArmConfig> {
    // 忽略格式解析
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const DexeRP::Kinematics::ArmConfig& cfg, FormatContext& ctx) {
        return fmt::format_to(ctx.out(), "ArmConfig({}, {}, {})", cfg.Shoulder(), cfg.Elbow(), cfg.Wrist());
    }
};

template <>
struct fmt::formatter<DexeRP::Kinematics::SRSConfig> {
    // 忽略格式解析
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const DexeRP::Kinematics::SRSConfig& cfg, FormatContext& ctx) {
        return fmt::format_to(ctx.out(), "SRSConfig({}, {}, {}, {})", cfg.Shoulder(), cfg.Elbow(), cfg.Wrist(),
                              cfg.arm_angle);
    }
};
