/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <fmt/format.h>

#include "dexe_rp/common/CommonHeader.h"
#include "dexe_rp/kinematics/Types.h"

namespace DexeRP {
namespace Kinematics {

struct FEPConfig {
    ArmConfig cfg{ArmConfig::Direction::Pos, ArmConfig::Direction::Pos, ArmConfig::Direction::Pos};
    // support: q7
    double redundancy_parameter{0.0};
};

// clang-format off

// Franka Emika Panda robot (without ee) as reference model
struct FEPGeometryParams {
public:
    double a_34, a_45, a_67;
    double d_01, d_23, d_45;

private:
    Mat<double, 7, 4> mdh_;           // MDH parameters
    Mat<double, 3, 7> Jw_home_;       // angular velocity Jacobian at home configuration

public:
    FEPGeometryParams(double a_34, double a_45, double a_67, double d_01, double d_23, double d_45) :
        a_34(a_34), a_45(a_45), a_67(a_67), d_01(d_01), d_23(d_23), d_45(d_45) {
        //       a    alpha   d   theta
        mdh_ <<  0.0,    0.0, d_01, 0.0,
                0.0, -kPi/2,  0.0, 0.0,
                0.0,  kPi/2, d_23, 0.0,
               a_34,  kPi/2,  0.0, 0.0,
              -a_45, -kPi/2, d_45, 0.0,
                0.0,  kPi/2,  0.0, 0.0,
               a_67,  kPi/2,  0.0, 0.0;
        // base frame -> x -> front, z -> up
        // Jw = [w1, w2, w3, w4, w5, w6, w7]
        Jw_home_ << 0.0, 0.0, 0.0,  0.0, 0.0,  0.0,  0.0,
                   0.0, 1.0, 0.0, -1.0, 0.0, -1.0,  0.0,
                   1.0, 0.0, 1.0,  0.0, 1.0,  0.0, -1.0;
        }

    static FEPGeometryParams FromJson(const nlohmann::json& j) {
        return FEPGeometryParams(j.value("a_34", 0.0),
                                 j.value("a_45", 0.0),
                                 j.value("a_67", 0.0),
                                 j.value("d_01", 0.0),
                                 j.value("d_23", 0.0),
                                 j.value("d_45", 0.0));
    }

    nlohmann::json ToJson() const {
        return {{"a_34", a_34}, {"a_45", a_45}, {"a_67", a_67}, {"d_01", d_01}, {"d_23", d_23}, {"d_45", d_45}};
    }

    const auto& mdh() const { return mdh_; }
    const auto& Jw_home() const { return Jw_home_; }
};

using FEPKineParams = ArmKineParams<7, FEPGeometryParams>;
using FEPKineParamsRegistry = KineParamsRegistry<FEPKineParams>;

struct FEPKineParamsAutoInit {
    FEPKineParamsAutoInit() {
        FEPKineParamsRegistry::Register(FEPKineParams("Franka", "FR3", 
            ArmCommonParams<7>(
                Deg2Rad(CVec7d{-166.0, -101.0, -166.0, -176.0, -166.0, -1.0, -166.0}),
                Deg2Rad(CVec7d{166.0, 101.0, 166.0, -4.0, 166.0, 215.0, 166.0}),
                Deg2Rad(CVec7d{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
                std::array<int, 7>{1, 1, 1, 1, 1, 1, 1},
                Mat4d::Identity(),
                (Mat4d() << 1,0,0,0,
                            0,1,0,0,
                            0,0,1,0.107,
                            0,0,0,1).finished()
            ),
            FEPGeometryParams(0.0825, 0.0825, 0.088, 0.333, 0.316, 0.384)));
        
        /// NOTE: Do NOT elbow up (not supported yet)
        FEPKineParamsRegistry::Register(FEPKineParams("DexForce", "W1V030Left", 
            ArmCommonParams<7>(
                Deg2Rad(CVec7d{-170.0, -120.0, -170.0, -148.0, -170.0, -95.0, -100.0}),
                Deg2Rad(CVec7d{170.0, 90.0, 170.0, 0.0, 170.0, 95.0, 100.0}),
                Deg2Rad(CVec7d{0.0, 0.0, -90.0, 0.0, 0.0, 90.0, 0.0}),
                std::array<int, 7>{1, -1, 1, 1, 1, 1, 1},
                (Mat4d() << 0.0, -1.0, 0.0, 0.0,
                            0.0,  0.0, 1.0, 0.113,
                           -1.0,  0.0, 0.0, 0.0,
                            0.0,  0.0, 0.0, 1.0).finished(),
                (Mat4d() << 0.0,  0.0, 1.0, 0.049,
                            0.0, -1.0, 0.0, 0.0,
                            1.0,  0.0, 0.0, 0.0,
                            0.0,  0.0, 0.0, 1.0).finished()
            ),
            FEPGeometryParams(0.015, 0.015, 0.075, 0.088, 0.26, 0.227)
        ));

        FEPKineParamsRegistry::Register(FEPKineParams("DexForce", "W1V030Right", 
            ArmCommonParams<7>(
                Deg2Rad(CVec7d{-170.0, -90.0, -170.0, 0.0, -170.0, -95.0, -100.0}),
                Deg2Rad(CVec7d{170.0, 120.0, 170.0, 148.0, 170.0, 95.0, 100.0}),
                Deg2Rad(CVec7d{0.0, 0.0, 90.0, 0.0, 0.0, 90.0, 0.0}),
                std::array<int, 7>{-1, 1, -1, -1, -1, -1, 1},
                (Mat4d() << 0.0, 1.0,  0.0, 0.0,
                            0.0, 0.0, -1.0, -0.113,
                            -1.0, 0.0,  0.0, 0.0,
                            0.0, 0.0,  0.0, 1.0).finished(),
                (Mat4d() << 0.0,  0.0, 1.0, 0.049,
                            0.0, -1.0, 0.0, 0.0,
                            1.0,  0.0, 0.0, 0.0,
                            0.0,  0.0, 0.0, 1.0).finished()
            ),
            FEPGeometryParams(0.015, 0.015, 0.075, 0.088, 0.26, 0.227)
        ));

        FEPKineParamsRegistry::Register(FEPKineParams("DexForce", "W1V031Left", 
            ArmCommonParams<7>(
                Deg2Rad(CVec7d{-170.0, -120.0, -170.0, -148.0, -170.0, -95.0, -100.0}),
                Deg2Rad(CVec7d{170.0, 90.0, 170.0, 0.0, 170.0, 95.0, 100.0}),
                Deg2Rad(CVec7d{0.0, 0.0, -90.0, 0.0, 0.0, 90.0, 0.0}),
                std::array<int, 7>{1, -1, 1, 1, 1, 1, 1},
                (Mat4d() << 0.0, -1.0, 0.0, 0.0,
                            0.0,  0.0, 1.0, 0.113,
                           -1.0,  0.0, 0.0, 0.0,
                            0.0,  0.0, 0.0, 1.0).finished(),
                (Mat4d() << 0.0,  0.0, 1.0, 0.049,
                            0.0, -1.0, 0.0, 0.0,
                            1.0,  0.0, 0.0, 0.0,
                            0.0,  0.0, 0.0, 1.0).finished()
            ),
            FEPGeometryParams(0.015, 0.015, 0.075, 0.088, 0.26, 0.227)
        ));

        FEPKineParamsRegistry::Register(FEPKineParams("DexForce", "W1V031Right", 
            ArmCommonParams<7>(
                Deg2Rad(CVec7d{-170.0, -90.0, -170.0, 0.0, -170.0, -95.0, -100.0}),
                Deg2Rad(CVec7d{170.0, 120.0, 170.0, 148.0, 170.0, 95.0, 100.0}),
                Deg2Rad(CVec7d{0.0, 0.0, 90.0, 0.0, 0.0, 90.0, 0.0}),
                std::array<int, 7>{-1, 1, -1, -1, -1, -1, 1},
                (Mat4d() << 0.0, 1.0,  0.0, 0.0,
                            0.0, 0.0, -1.0, -0.113,
                            -1.0, 0.0,  0.0, 0.0,
                            0.0, 0.0,  0.0, 1.0).finished(),
                (Mat4d() << 0.0,  0.0, 1.0, 0.049,
                            0.0, -1.0, 0.0, 0.0,
                            1.0,  0.0, 0.0, 0.0,
                            0.0,  0.0, 0.0, 1.0).finished()
            ),
            FEPGeometryParams(0.015, 0.015, 0.075, 0.088, 0.26, 0.227)
        ));

        FEPKineParamsRegistry::Register(FEPKineParams("DexForce", "W1V031IPLUSLeft", 
            ArmCommonParams<7>(
                Deg2Rad(CVec7d{-170.0, -120.0, -170.0, -148.0, -170.0, -95.0, -100.0}),
                Deg2Rad(CVec7d{170.0, 90.0, 170.0, 0.0, 170.0, 95.0, 100.0}),
                Deg2Rad(CVec7d{0.0, 0.0, -90.0, 0.0, 0.0, 90.0, 0.0}),
                std::array<int, 7>{1, -1, 1, 1, 1, 1, 1},
                (Mat4d() << 0.0, -1.0, 0.0, 0.0,
                            0.0,  0.0, 1.0, 0.113,
                           -1.0,  0.0, 0.0, 0.0,
                            0.0,  0.0, 0.0, 1.0).finished(),
                (Mat4d() << 0.0,  0.0, 1.0, 0.049,
                            0.0, -1.0, 0.0, 0.0,
                            1.0,  0.0, 0.0, 0.0,
                            0.0,  0.0, 0.0, 1.0).finished()
            ),
            FEPGeometryParams(0.015, 0.015, 0.075, 0.088, 0.26, 0.227)
        ));

        FEPKineParamsRegistry::Register(FEPKineParams("DexForce", "W1V031IPLUSRight", 
            ArmCommonParams<7>(
                Deg2Rad(CVec7d{-170.0, -90.0, -170.0, 0.0, -170.0, -95.0, -100.0}),
                Deg2Rad(CVec7d{170.0, 120.0, 170.0, 148.0, 170.0, 95.0, 100.0}),
                Deg2Rad(CVec7d{0.0, 0.0, 90.0, 0.0, 0.0, 90.0, 0.0}),
                std::array<int, 7>{-1, 1, -1, -1, -1, -1, 1},
                (Mat4d() << 0.0, 1.0,  0.0, 0.0,
                            0.0, 0.0, -1.0, -0.113,
                            -1.0, 0.0,  0.0, 0.0,
                            0.0, 0.0,  0.0, 1.0).finished(),
                (Mat4d() << 0.0,  0.0, 1.0, 0.049,
                            0.0, -1.0, 0.0, 0.0,
                            1.0,  0.0, 0.0, 0.0,
                            0.0,  0.0, 0.0, 1.0).finished()
            ),
            FEPGeometryParams(0.015, 0.015, 0.075, 0.088, 0.26, 0.227)
        ));
    }
};
inline FEPKineParamsAutoInit g_auto_init_fep_kine_params;

// clang-format on

}  // namespace Kinematics
}  // namespace DexeRP

template <>
struct fmt::formatter<DexeRP::Kinematics::FEPConfig> {
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const DexeRP::Kinematics::FEPConfig& cfg, FormatContext& ctx) {
        return fmt::format_to(ctx.out(), "FEPConfig({}, {})", cfg.cfg, cfg.redundancy_parameter);
    }
};
