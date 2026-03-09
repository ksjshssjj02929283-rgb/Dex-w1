/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <fstream>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "dexe_rp/kinematics/Types.h"

namespace DexeRP {
namespace Kinematics {

struct OPWConfig : public ArmConfig {
public:
    OPWConfig(Direction s, Direction e, Direction w) : ArmConfig{s, e, w} {}

    OPWConfig(const CVec6d& qpos) {
        shoulder_ = qpos(0) > 0 ? Pos : Neg;
        elbow_ = qpos(2) > 0 ? Pos : Neg;
        wrist_ = qpos(4) > 0 ? Pos : Neg;
    }

    virtual ~OPWConfig() = default;
};

struct OPWKineParams {
    // OPW available through papers: An Analytical Solution of the Inverse
    // Kinematics Problem of Industrial Serial Manipulators with an Ortho-parallel
    // Basis and a Spherical Wrist

    // a1, a2 : the arm-offsets.
    // c1, c2, c3, c4 : the main arm lengths.
    // b : the the lateral offset of the third arm in y0-direction.
    double a1, a2, c1, c2, c3, c4, b;

    // joint limits in rad
    CVec6d qpos_lb, qpos_ub;

    // joint offsets in rad
    std::array<double, 6> offsets{};

    // rotation direction (about z-axis)
    // 1 for clockwise,
    // -1 for counterclockwise.
    std::array<int, 6> axis_dirs;

    // from "base" frame to opsw_base frame
    Mat4d T_b_ob;
    // from opsw_end_effector frame to "end_effector" frame
    Mat4d T_oe_e;

    OPWKineParams() : a1{0}, a2{0}, c1{0}, c2{0}, c3{0}, c4{0}, b{0} {
        // debug logger
    }

    OPWKineParams(double a1,
                  double a2,
                  double c1,
                  double c2,
                  double c3,
                  double c4,
                  double b,
                  const CVec6d& qpos_lb,
                  const CVec6d& qpos_ub,
                  const std::array<double, 6>& offsets = {0, 0, 0, 0, 0, 0},
                  const std::array<int, 6>& axis_dirs = {1, 1, 1, 1, 1, 1},
                  const Mat4d& T_b_ob = Mat4d::Identity(),
                  const Mat4d& T_oe_e = Mat4d::Identity());

    virtual ~OPWKineParams() = default;
};

class OPWKineParamsRegistry {
public:
    OPWKineParamsRegistry(const OPWKineParamsRegistry&) = delete;
    static void SetJson(const nlohmann::json& j) { json_ = j; }

    static void SetJsonFile(const std::string& filename) {
        std::ifstream in(filename);
        in >> json_;
    }

    static bool Register(const std::string& brand, const std::string& model, const OPWKineParams& params) {
        auto& models = Instance().models_;
        models[brand][model] = params;
        return true;
    }

    static std::optional<OPWKineParams> Get(const std::string& brand, const std::string& model);

    static std::vector<std::pair<std::string, std::string>> SupportedRobots();

private:
    std::unordered_map<std::string, std::unordered_map<std::string, OPWKineParams>> models_;
    static nlohmann::json json_;

    OPWKineParamsRegistry() = default;

    static OPWKineParamsRegistry& Instance() {
        static OPWKineParamsRegistry instance;
        return instance;
    }
};

}  // namespace Kinematics
}  // namespace DexeRP
