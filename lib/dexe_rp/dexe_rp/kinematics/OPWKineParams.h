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

struct OPWConfig {
    ArmConfig cfg{ArmConfig::Direction::Pos, ArmConfig::Direction::Pos, ArmConfig::Direction::Pos};
};

// clang-format off

/*
"An Analytical Solution of the Inverse Kinematics Problem of 
Industrial Serial Manipulators with an Ortho-parallel Basis 
and a Spherical Wrist" as reference model
*/ 
struct OPWGeometryParams {
public:
    // a1, a2 : the arm-offsets.
    // b : the the lateral offset of the third arm in y0-direction.
    // c1, c2, c3, c4 : the main arm lengths.
    double a1, a2, b, c1, c2, c3, c4;

private:
    // Mat<double, 6, 4> mdh_;           // MDH parameters
    Mat<double, 3, 6> Jw_home_;       // angular velocity Jacobian at home configuration

public:
    OPWGeometryParams(double a1, double a2, double b, double c1, double c2, double c3, double c4) :
        a1(a1), a2(a2), b(b), c1(c1), c2(c2), c3(c3), c4(c4) {
        //      a    alpha   d   theta
        // mdh_ << 

        // base frame -> x -> front, z -> up
        // Jw = [w1, w2, w3, w4, w5, w6]
        Jw_home_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 1.0, 0.0, 1.0, 0.0,
                    1.0, 0.0, 0.0, 1.0, 0.0, 1.0;
    }

    static OPWGeometryParams FromJson(const nlohmann::json& j) {
        return OPWGeometryParams(j.value("a1", 0.0),
                                 j.value("a2", 0.0),
                                 j.value("b", 0.0),
                                 j.value("c1", 0.0),
                                 j.value("c2", 0.0),
                                 j.value("c3", 0.0),
                                 j.value("c4", 0.0));
    }

    nlohmann::json ToJson() const {
        return {{"a1", a1}, {"a2", a2}, {"b", b}, {"c1", c1}, {"c2", c2}, {"c3", c3}, {"c4", c4}};
    }

    // const auto& mdh() const { return mdh_; }
    const auto& Jw_home() const { return Jw_home_; }
};

using OPWKineParams = ArmKineParams<6, OPWGeometryParams>;
using OPWKineParamsRegistry = KineParamsRegistry<OPWKineParams>;

struct OPWKineParamsAutoInit {
    OPWKineParamsAutoInit() {
        OPWKineParamsRegistry::Register(OPWKineParams::FromJson({
            {"brand", "AGILEX"},
            {"model", "CobotMagicArm"},
            {"common", {
                {"qpos_lb", {-2.618, 0.0, -2.967, -1.745, -1.22, -2.0944}},
                {"qpos_ub", {2.618, 3.14, 0, 1.745, 1.22, 2.0944}},
                {"qpos_offsets", {0.0, -1.435512, 2.918473, 0.0, 0.0, 0.0}},
                {"axis_dirs", {1, 1, 1, 1, 1, 1}},
                {"T_b_ob", {
                    {1,0,0,0},
                    {0,1,0,0},
                    {0,0,1,0},
                    {0,0,0,1}
                }},
                {"T_oe_e", {
                    {1,0,0,0},
                    {0,1,0,0},
                    {0,0,1,0},
                    {0,0,0,1}
                }}
            }},
            {"geometry", {
                {"a1", 0.0},
                {"a2", -0.021984},
                {"b", 0.0},
                {"c1", 0.0},
                {"c2", 0.123},
                {"c3", 0.28503},
                {"c4", 0.25075}
            }}
        }));

        /// TODO: Consider the multi-loop case
        OPWKineParamsRegistry::Register(OPWKineParams::FromJson({
            {"brand", "ABB"},
            {"model", "IRB1200_5_90"},
            {"common", {
                {"qpos_lb", {-2.967, -1.745, -3.490, -4.712, -2.268, -6.981}},
                {"qpos_ub", {2.967, 2.268, 1.221, 4.71, 2.268, 6.981}},
                {"qpos_offsets", {0.0, 0.0, kPi/2, 0.0, 0.0, 0.0}},
                {"axis_dirs", {1, 1, 1, 1, 1, 1, 1}},
                {"T_b_ob", {
                    {1,0,0,0},
                    {0,1,0,0},
                    {0,0,1,0.3991},
                    {0,0,0,1}
                }},
                {"T_oe_e", {
                    {1,0,0,0},
                    {0,1,0,0},
                    {0,0,1,0},
                    {0,0,0,1}
                }}
            }},
            {"geometry", {
                {"a1", 0.0},
                {"a2", -0.042},
                {"b", 0.0},
                {"c1", 0.0},
                {"c2", 0.448},
                {"c3", 0.451},
                {"c4", 0.082}
            }}
        }));
        
        // OPWKineParamsRegistry::Register(OPWKineParams::FromJson({
        //     {"brand", "Motoman"},
        //     {"model", "GP7"},
        //     {"common", {
        //         {"qpos_lb", {-2.967, -1.1344, -1.2217, -3.3161, -2.3561, -6.2944}},
        //         {"qpos_ub", {2.967, 2.5307, 3.3161, 3.3161, 2.3561, 6.2944}},
        //         {"qpos_offsets", {0.0, 0.0, kPi/2, 0.0, 0.0, 0.0}},
        //         {"axis_dirs", {1, 1, -1, -1, -1, -1}},
        //         {"T_b_ob", {
        //             {1,0,0,0},
        //             {0,1,0,0},
        //             {0,0,1,0.33},
        //             {0,0,0,1}
        //         }},
        //         {"T_oe_e", {
        //             {-1,0,0,0},
        //             {0,-1,0,0},
        //             {0,0,1,0},
        //             {0,0,0,1}
        //         }}
        //     }},
        //     {"geometry", {
        //         {"a1", 0.04},
        //         {"a2", -0.04},
        //         {"b", 0.0},
        //         {"c1", 0.0},
        //         {"c2", 0.445},
        //         {"c3", 0.44},
        //         {"c4", 0.08}
        //     }}
        // }));

        // OPWKineParamsRegistry::Register(OPWKineParams::FromJson({
        //     {"brand", "Motoman"},
        //     {"model", "GP12"},
        //     {"common", {
        //         {"qpos_lb", {-2.967, -1.5708, -1.4835, -3.4906, -2.6179, -7.9412}},
        //         {"qpos_ub", {2.967, 2.7052, 2.6179, 3.4906, 2.6179, 7.9412}},
        //         {"qpos_offsets", {0.0, 0.0, kPi/2, 0.0, 0.0, 0.0}},
        //         {"axis_dirs", {1, 1, -1, -1, -1, -1}},
        //         {"T_b_ob", {
        //             {1,0,0,0},
        //             {0,1,0,0},
        //             {0,0,1,0.45},
        //             {0,0,0,1}
        //         }},
        //         {"T_oe_e", {
        //             {-1,0,0,0},
        //             {0,-1,0,0},
        //             {0,0,1,0},
        //             {0,0,0,1}
        //         }}
        //     }},
        //     {"geometry", {
        //         {"a1", 0.155},
        //         {"a2", -0.2},
        //         {"b", 0.0},
        //         {"c1", 0.0},
        //         {"c2", 0.614},
        //         {"c3", 0.64},
        //         {"c4", 0.1}
        //     }}
        // }));

    };

};
inline OPWKineParamsAutoInit g_auto_init_opw_kine_params;

// clang-format on

}  // namespace Kinematics
}  // namespace DexeRP

template <>
struct fmt::formatter<DexeRP::Kinematics::OPWConfig> {
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const DexeRP::Kinematics::OPWConfig& cfg, FormatContext& ctx) {
        return fmt::format_to(ctx.out(), "OPWConfig({})", cfg.cfg);
    }
};
