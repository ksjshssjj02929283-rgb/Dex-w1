/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <fmt/format.h>

#include <nlohmann/json.hpp>

#include "dexe_rp/common/CommonHeader.h"
#include "dexe_rp/kinematics/Types.h"

namespace DexeRP {
namespace Kinematics {

struct SRSConfig {
    ArmConfig cfg{ArmConfig::Direction::Pos, ArmConfig::Direction::Pos, ArmConfig::Direction::Pos};
    // support: arm_angle
    double redundancy_parameter{0.0};
};

// clang-format off

// KUKA iiwa14 robot as reference model
struct SRSGeometryParams {
public:
    double d_01, d_24, d_46, d_67;

private:
    Mat<double, 7, 4> mdh_;           // MDH parameters
    Mat<double, 3, 7> Jw_home_;       // angular velocity Jacobian at home configuration

public:
    SRSGeometryParams(double d_01, double d_24, double d_46, double d_67) :
        d_01(d_01), d_24(d_24), d_46(d_46), d_67(d_67) {
        //      a    alpha   d   theta
        mdh_ << 0.0,    0.0, d_01, 0.0,
               0.0, -kPi/2,  0.0, 0.0,
               0.0,  kPi/2, d_24, 0.0,
               0.0,  kPi/2,  0.0, 0.0,
               0.0, -kPi/2, d_46, 0.0,
               0.0, -kPi/2,  0.0, 0.0,
               0.0,  kPi/2, d_67, 0.0;
        // base frame -> x -> front, z -> up
        // Jw = [w1, w2, w3, w4, w5, w6, w7]
        Jw_home_ << 0.0,  0.0, 0.0, 0.0, 0.0,  0.0, 0.0,
                   0.0, -1.0, 0.0, 1.0, 0.0, -1.0, 0.0,
                   1.0,  0.0, 1.0, 0.0, 1.0,  0.0, 1.0;
    }

    static SRSGeometryParams FromJson(const nlohmann::json& j) {
        return SRSGeometryParams(j.value("d_01", 0.0),
                                 j.value("d_24", 0.0),
                                 j.value("d_46", 0.0),
                                 j.value("d_67", 0.0));
    }

    nlohmann::json ToJson() const {
        return {{"d_01", d_01}, {"d_24", d_24}, {"d_46", d_46}, {"d_67", d_67}};
    }

    const auto& mdh() const { return mdh_; }
    const auto& Jw_home() const { return Jw_home_; }
};

using SRSKineParams = ArmKineParams<7, SRSGeometryParams>;
using SRSKineParamsRegistry = KineParamsRegistry<SRSKineParams>;

struct SRSKineParamsAutoInit {
    SRSKineParamsAutoInit() {
        SRSKineParamsRegistry::Register(SRSKineParams("KUKA", "IIWA14", 
            ArmCommonParams<7>(
                Deg2Rad(CVec7d{-170.0, -120.0, -170.0, -120.0, -170.0, -120.0, -175.0}),
                Deg2Rad(CVec7d{170.0, 120.0, 170.0, 120.0, 170.0, 120.0, 175.0}),
                Deg2Rad(CVec7d{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
                std::array<int, 7>{1, 1, 1, 1, 1, 1, 1},
                Mat4d::Identity(),
                (Mat4d() << 1,0,0,0,
                            0,1,0,0,
                            0,0,1,0.045,
                            0,0,0,1).finished()
            ),
            SRSGeometryParams(0.36, 0.42, 0.4, 0.081)));

        // -------------- W1 V026 --------------
        SRSKineParamsRegistry::Register(SRSKineParams("DexForce", "W1V026Left",
            ArmCommonParams<7>(
                Deg2Rad(CVec7d{-170.0, -120.0, -170.0, -135.0, -170.0, -45.0, -90.0}),
                Deg2Rad(CVec7d{170.0, 90.0, 170.0, 90.0, 170.0, 45.0, 60.0}),
                Deg2Rad(CVec7d{0.0, 0.0, -90.0, 0.0, 0.0, 90.0, 0.0}),
                std::array<int, 7>{1, -1, 1, 1, 1, -1, 1},
                (Mat4d() << -1, 0, 0, 0,
                             0,-1, 0, 0,
                             0, 0, 1, 0,
                             0, 0, 0, 1).finished(),
                (Mat4d() << 0, 0, -1, -0.066,
                            0, 1, 0, 0,
                            1, 0, 0, 0,
                            0, 0, 0, 1).finished()
            ),
            SRSGeometryParams(0.1025, 0.26, 0.264, 0.0)));

        SRSKineParamsRegistry::Register(SRSKineParams("DexForce", "W1V026Right",
            ArmCommonParams<7>(
                Deg2Rad(CVec7d{-170.0, -90.0, -170.0, -90.0, -170.0, -45.0, -60.0}),
                Deg2Rad(CVec7d{170.0, 120.0, 170.0, 135.0, 170.0, 45.0, 90.0}),
                Deg2Rad(CVec7d{0.0, 0.0, 90.0, 0.0, 0.0, 90.0, 0.0}),
                std::array<int, 7>{1, 1, 1, -1, 1, 1, 1},
                Mat4d::Identity(),
                (Mat4d() << 0, 0,-1, -0.066,
                            0, 1, 0, 0,
                            1, 0, 0, 0,
                            0, 0, 0, 1).finished()
            ),
            SRSGeometryParams(0.1025, 0.26, 0.264, 0.0)
        ));

        // Notes: V021-V025 the same kine params
        // -------------- W1 V025 --------------
        SRSKineParamsRegistry::Register(SRSKineParams("DexForce", "W1V025Left",
            ArmCommonParams<7>(
                Deg2Rad(CVec7d{-170.0, -120.0, -170.0, -135.0, -170.0, -45.0, -90.0}),
                Deg2Rad(CVec7d{170.0, 90.0, 170.0, 90.0, 170.0, 45.0, 60.0}),
                Deg2Rad(CVec7d{0.0, 0.0, -90.0, 0.0, 0.0, 90.0, 0.0}),
                std::array<int, 7>{1, -1, 1, 1, 1, -1, 1},
                (Mat4d() << -1, 0, 0, 0,
                             0,-1, 0, 0,
                             0, 0, 1, 0,
                             0, 0, 0, 1).finished(),
                (Mat4d() << 0, 0, -1, -0.066,
                            0, 1, 0, 0,
                            1, 0, 0, 0,
                            0, 0, 0, 1).finished()
            ),
            SRSGeometryParams(0.1025, 0.26, 0.264, 0.0)));

        SRSKineParamsRegistry::Register(SRSKineParams("DexForce", "W1V025Right",
            ArmCommonParams<7>(
                Deg2Rad(CVec7d{-170.0, -90.0, -170.0, -90.0, -170.0, -45.0, -60.0}),
                Deg2Rad(CVec7d{170.0, 120.0, 170.0, 135.0, 170.0, 45.0, 90.0}),
                Deg2Rad(CVec7d{0.0, 0.0, 90.0, 0.0, 0.0, 90.0, 0.0}),
                std::array<int, 7>{1, 1, 1, -1, 1, 1, 1},
                Mat4d::Identity(),
                (Mat4d() << 0, 0,-1, -0.066,
                            0, 1, 0, 0,
                            1, 0, 0, 0,
                            0, 0, 0, 1).finished()
            ),
            SRSGeometryParams(0.1025, 0.26, 0.264, 0.0)
        ));

        // -------------- W1 V024 --------------
        SRSKineParamsRegistry::Register(SRSKineParams("DexForce", "W1V024Left",
            ArmCommonParams<7>(
                Deg2Rad(CVec7d{-170.0, -120.0, -170.0, -135.0, -170.0, -45.0, -90.0}),
                Deg2Rad(CVec7d{170.0, 90.0, 170.0, 90.0, 170.0, 45.0, 60.0}),
                Deg2Rad(CVec7d{0.0, 0.0, -90.0, 0.0, 0.0, 90.0, 0.0}),
                std::array<int, 7>{1, -1, 1, 1, 1, -1, 1},
                (Mat4d() << -1, 0, 0, 0,
                             0,-1, 0, 0,
                             0, 0, 1, 0,
                             0, 0, 0, 1).finished(),
                (Mat4d() << 0, 0, -1, -0.066,
                            0, 1, 0, 0,
                            1, 0, 0, 0,
                            0, 0, 0, 1).finished()
            ),
            SRSGeometryParams(0.1025, 0.26, 0.264, 0.0)));

        SRSKineParamsRegistry::Register(SRSKineParams("DexForce", "W1V024Right",
            ArmCommonParams<7>(
                Deg2Rad(CVec7d{-170.0, -90.0, -170.0, -90.0, -170.0, -45.0, -60.0}),
                Deg2Rad(CVec7d{170.0, 120.0, 170.0, 135.0, 170.0, 45.0, 90.0}),
                Deg2Rad(CVec7d{0.0, 0.0, 90.0, 0.0, 0.0, 90.0, 0.0}),
                std::array<int, 7>{1, 1, 1, -1, 1, 1, 1},
                Mat4d::Identity(),
                (Mat4d() << 0, 0,-1, -0.066,
                            0, 1, 0, 0,
                            1, 0, 0, 0,
                            0, 0, 0, 1).finished()
            ),
            SRSGeometryParams(0.1025, 0.26, 0.264, 0.0)
        ));

        SRSKineParamsRegistry::Register(SRSKineParams("DexForce", "W1V024ILeft",
            ArmCommonParams<7>(
                Deg2Rad(CVec7d{-170.0, -120.0, -170.0, -135.0, -170.0, -90.0, -170.0}),
                Deg2Rad(CVec7d{170.0, 90.0, 170.0, 90.0, 170.0, 90.0, 170.0}),
                Deg2Rad(CVec7d{0.0, 0.0, -90.0, 0.0, 0.0, 0.0, 0.0}),
                std::array<int, 7>{1, -1, 1, 1, 1, -1, 1},
                (Mat4d() << -1, 0, 0, 0,
                             0,-1, 0, 0,
                             0, 0, 1, 0,
                             0, 0, 0, 1).finished(),
                (Mat4d() << -1, 0, 0, 0,
                            0, -1, 0, 0,
                            0, 0, 1, 0.066,
                            0, 0, 0, 1).finished()
            ),
            SRSGeometryParams(0.1025, 0.26, 0.264, 0.0)
        ));

        SRSKineParamsRegistry::Register(SRSKineParams("DexForce", "W1V024IRight",
            ArmCommonParams<7>(
                Deg2Rad(CVec7d{-170.0, -90.0, -170.0, -90.0, -170.0, -90.0, -170.0}),
                Deg2Rad(CVec7d{170.0, 120.0, 170.0, 135.0, 170.0, 90.0, 170.0}),
                Deg2Rad(CVec7d{0.0, 0.0, 90.0, 0.0, 0.0, 0.0, 0.0}),
                std::array<int, 7>{1, 1, 1, -1, 1, 1, 1},
                Mat4d::Identity(),
                (Mat4d() << -1, 0, 0, 0,
                            0, -1, 0, 0,
                            0, 0, 1, 0.066,
                            0, 0, 0, 1).finished()
            ),
            SRSGeometryParams(0.1025, 0.26, 0.264, 0.0)
        ));

        // -------------- W1 V023 --------------
        SRSKineParamsRegistry::Register(SRSKineParams("DexForce", "W1V023Left",
            ArmCommonParams<7>(
                Deg2Rad(CVec7d{-170.0, -120.0, -170.0, -135.0, -170.0, -45.0, -90.0}),
                Deg2Rad(CVec7d{170.0, 90.0, 170.0, 90.0, 170.0, 45.0, 60.0}),
                Deg2Rad(CVec7d{0.0, 0.0, -90.0, 0.0, 0.0, 90.0, 0.0}),
                std::array<int, 7>{1, -1, 1, 1, 1, -1, 1},
                (Mat4d() << -1, 0, 0, 0,
                             0,-1, 0, 0,
                             0, 0, 1, 0,
                             0, 0, 0, 1).finished(),
                (Mat4d() << 0, 0, -1, -0.066,
                            0, 1, 0, 0,
                            1, 0, 0, 0,
                            0, 0, 0, 1).finished()
            ),
            SRSGeometryParams(0.1025, 0.26, 0.264, 0.0)));

        SRSKineParamsRegistry::Register(SRSKineParams("DexForce", "W1V023Right",
            ArmCommonParams<7>(
                Deg2Rad(CVec7d{-170.0, -90.0, -170.0, -90.0, -170.0, -45.0, -60.0}),
                Deg2Rad(CVec7d{170.0, 120.0, 170.0, 135.0, 170.0, 45.0, 90.0}),
                Deg2Rad(CVec7d{0.0, 0.0, 90.0, 0.0, 0.0, 90.0, 0.0}),
                std::array<int, 7>{1, 1, 1, -1, 1, 1, 1},
                Mat4d::Identity(),
                (Mat4d() << 0, 0,-1, -0.066,
                            0, 1, 0, 0,
                            1, 0, 0, 0,
                            0, 0, 0, 1).finished()
            ),
            SRSGeometryParams(0.1025, 0.26, 0.264, 0.0)
        ));

        SRSKineParamsRegistry::Register(SRSKineParams("DexForce", "W1V023ILeft",
            ArmCommonParams<7>(
                Deg2Rad(CVec7d{-170.0, -120.0, -170.0, -135.0, -170.0, -90.0, -170.0}),
                Deg2Rad(CVec7d{170.0, 90.0, 170.0, 90.0, 170.0, 90.0, 170.0}),
                Deg2Rad(CVec7d{0.0, 0.0, -90.0, 0.0, 0.0, 0.0, 0.0}),
                std::array<int, 7>{1, -1, 1, 1, 1, -1, 1},
                (Mat4d() << -1, 0, 0, 0,
                             0,-1, 0, 0,
                             0, 0, 1, 0,
                             0, 0, 0, 1).finished(),
                (Mat4d() << -1, 0, 0, 0,
                            0, -1, 0, 0,
                            0, 0, 1, 0.066,
                            0, 0, 0, 1).finished()
            ),
            SRSGeometryParams(0.1025, 0.26, 0.264, 0.0)
        ));

        SRSKineParamsRegistry::Register(SRSKineParams("DexForce", "W1V023IRight",
            ArmCommonParams<7>(
                Deg2Rad(CVec7d{-170.0, -90.0, -170.0, -90.0, -170.0, -90.0, -170.0}),
                Deg2Rad(CVec7d{170.0, 120.0, 170.0, 135.0, 170.0, 90.0, 170.0}),
                Deg2Rad(CVec7d{0.0, 0.0, 90.0, 0.0, 0.0, 0.0, 0.0}),
                std::array<int, 7>{1, 1, 1, -1, 1, 1, 1},
                Mat4d::Identity(),
                (Mat4d() << -1, 0, 0, 0,
                            0, -1, 0, 0,
                            0, 0, 1, 0.066,
                            0, 0, 0, 1).finished()
            ),
            SRSGeometryParams(0.1025, 0.26, 0.264, 0.0)
        ));

        // -------------- W1 V022 --------------
        SRSKineParamsRegistry::Register(SRSKineParams("DexForce", "W1V022Left",
            ArmCommonParams<7>(
                Deg2Rad(CVec7d{-170.0, -120.0, -170.0, -135.0, -170.0, -45.0, -90.0}),
                Deg2Rad(CVec7d{170.0, 90.0, 170.0, 90.0, 170.0, 45.0, 60.0}),
                Deg2Rad(CVec7d{0.0, 0.0, -90.0, 0.0, 0.0, 90.0, 0.0}),
                std::array<int, 7>{1, -1, 1, 1, 1, -1, 1},
                (Mat4d() << -1, 0, 0, 0,
                             0,-1, 0, 0,
                             0, 0, 1, 0,
                             0, 0, 0, 1).finished(),
                (Mat4d() << 0, 0, -1, -0.066,
                            0, 1, 0, 0,
                            1, 0, 0, 0,
                            0, 0, 0, 1).finished()
            ),
            SRSGeometryParams(0.1025, 0.26, 0.264, 0.0)));

        SRSKineParamsRegistry::Register(SRSKineParams("DexForce", "W1V022Right",
            ArmCommonParams<7>(
                Deg2Rad(CVec7d{-170.0, -90.0, -170.0, -90.0, -170.0, -45.0, -60.0}),
                Deg2Rad(CVec7d{170.0, 120.0, 170.0, 135.0, 170.0, 45.0, 90.0}),
                Deg2Rad(CVec7d{0.0, 0.0, 90.0, 0.0, 0.0, 90.0, 0.0}),
                std::array<int, 7>{1, 1, 1, -1, 1, 1, 1},
                Mat4d::Identity(),
                (Mat4d() << 0, 0,-1, -0.066,
                            0, 1, 0, 0,
                            1, 0, 0, 0,
                            0, 0, 0, 1).finished()
            ),
            SRSGeometryParams(0.1025, 0.26, 0.264, 0.0)
        ));

        // -------------- W1 V021 --------------
        SRSKineParamsRegistry::Register(SRSKineParams("DexForce", "W1V021Left",
            ArmCommonParams<7>(
                Deg2Rad(CVec7d{-170.0, -120.0, -170.0, -135.0, -170.0, -45.0, -90.0}),
                Deg2Rad(CVec7d{170.0, 90.0, 170.0, 90.0, 170.0, 45.0, 60.0}),
                Deg2Rad(CVec7d{0.0, 0.0, -90.0, 0.0, 0.0, 90.0, 0.0}),
                std::array<int, 7>{1, -1, 1, 1, 1, -1, 1},
                (Mat4d() << -1, 0, 0, 0,
                             0,-1, 0, 0,
                             0, 0, 1, 0,
                             0, 0, 0, 1).finished(),
                (Mat4d() << 0, 0, -1, -0.066,
                            0, 1, 0, 0,
                            1, 0, 0, 0,
                            0, 0, 0, 1).finished()
            ),
            SRSGeometryParams(0.1025, 0.26, 0.264, 0.0)));

        SRSKineParamsRegistry::Register(SRSKineParams("DexForce", "W1V021Right",
            ArmCommonParams<7>(
                Deg2Rad(CVec7d{-170.0, -90.0, -170.0, -90.0, -170.0, -45.0, -60.0}),
                Deg2Rad(CVec7d{170.0, 120.0, 170.0, 135.0, 170.0, 45.0, 90.0}),
                Deg2Rad(CVec7d{0.0, 0.0, 90.0, 0.0, 0.0, 90.0, 0.0}),
                std::array<int, 7>{1, 1, 1, -1, 1, 1, 1},
                Mat4d::Identity(),
                (Mat4d() << 0, 0,-1, -0.066,
                            0, 1, 0, 0,
                            1, 0, 0, 0,
                            0, 0, 0, 1).finished()
            ),
            SRSGeometryParams(0.1025, 0.26, 0.264, 0.0)
        ));
    }
};
inline SRSKineParamsAutoInit g_auto_init_srs_kine_params;

// clang-format on

}  // namespace Kinematics
}  // namespace DexeRP

template <>
struct fmt::formatter<DexeRP::Kinematics::SRSConfig> {
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const DexeRP::Kinematics::SRSConfig& cfg, FormatContext& ctx) {
        return fmt::format_to(ctx.out(), "SRSConfig({}, {})", cfg.cfg, cfg.redundancy_parameter);
    }
};
