/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <fstream>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "dexe_rp/kinematics/Types.h"

namespace DexeRP {
namespace Kinematics {

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
    // SRS available through papers:
    // https://www.sciencedirect.com/science/article/abs/pii/S0094114X17306559

    double bs, se, ew, wt;    // link lengths
    CVec7d qpos_lb, qpos_ub;  // joint position limits

    std::array<double, 7> offsets;  // joint offsets of SRS-model
    std::array<int, 7> axis_dirs;   // TODO: axis directions of SRS-model(unused)
    Mat4d T_b_ob;                   // base frame to srs_base frame
    Mat4d T_oe_e;                   // srs_ee frame to ee frame

    Mat<double, 7, 4> mdh;  // MDH parameters of SRS-model

    SRSKineParams()
        : bs{0.},
          se{0.},
          ew{0.},
          wt{0.},
          qpos_lb{CVec7d::Zero()},
          qpos_ub{CVec7d::Zero()},
          offsets{0., 0., 0., 0., 0., 0., 0.},
          axis_dirs{1, 1, 1, 1, 1, 1, 1},
          T_b_ob{Mat4d::Identity()},
          T_oe_e{Mat4d::Identity()},
          mdh(MatXd::Zero(7, 4)) {}

    SRSKineParams(double bs,
                  double se,
                  double ew,
                  double wt,
                  const CVec7d& qpos_lb,
                  const CVec7d& qpos_ub,
                  const std::array<double, 7>& offsets = {0., 0., 0., 0., 0., 0., 0.},
                  const std::array<int, 7>& axis_dirs = {1, 1, 1, 1, 1, 1, 1},
                  const Mat4d& T_b_ob = Mat4d::Identity(),
                  const Mat4d& T_oe_e = Mat4d::Identity());

    virtual ~SRSKineParams() = default;
};

struct W1V02KineParams : public SRSKineParams {
    bool is_left;
    bool is_anthropomorphic;

    W1V02KineParams(bool is_left, bool is_anthropomorphic);

    W1V02KineParams(bool is_left, bool is_anthropomorphic, double bs, double se, double ew, double wt);
    virtual ~W1V02KineParams() = default;
};

}  // namespace Kinematics
}  // namespace DexeRP

template <>
struct fmt::formatter<DexeRP::Kinematics::SRSConfig> {
    // Parse format specifications
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const DexeRP::Kinematics::SRSConfig& cfg, FormatContext& ctx) {
        return fmt::format_to(ctx.out(), "SRSConfig({}, {}, {}, {})", cfg.Shoulder(), cfg.Elbow(), cfg.Wrist(),
                              cfg.arm_angle);
    }
};
