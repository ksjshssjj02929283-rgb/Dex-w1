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

struct URsConfig : public ArmConfig {
public:
    URsConfig(Direction s, Direction e, Direction w) : ArmConfig{s, e, w} {}

    URsConfig(const CVec6d& qpos) {
        shoulder_ = qpos(0) > 0 ? Pos : Neg;
        elbow_ = qpos(2) > 0 ? Pos : Neg;
        wrist_ = qpos(4) > 0 ? Pos : Neg;
    }

    virtual ~URsConfig() = default;
};

struct URsKineParams {
    // read from https://github.com/pyni/ur_inverse_solutions
    double d1, a2, a3, d4, d5, d6;

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
    //< from opsw_end_effector frame to "end_effector" frame
    Mat4d T_oe_e;

    URsKineParams() : d1{0}, a2{0}, a3{0}, d4{0}, d5{0}, d6{0} {
        // debug logger
    }

    URsKineParams(double d1,
                  double a2,
                  double a3,
                  double d4,
                  double d5,
                  double d6,
                  const CVec6d& qpos_lb,
                  const CVec6d& qpos_ub,
                  const std::array<double, 6>& offsets = {0, 0, 0, 0, 0, 0},
                  const std::array<int, 6>& axis_dirs = {1, 1, 1, 1, 1, 1},
                  const Mat4d& T_b_ob = Mat4d::Identity(),
                  const Mat4d& T_oe_e = Mat4d::Identity());

    virtual ~URsKineParams() = default;
};

class URsKineParamsRegistry {
public:
    URsKineParamsRegistry(const URsKineParamsRegistry&) = delete;
    static void SetJson(const nlohmann::json& j) { json_ = j; }

    static void SetJsonFile(const std::string& filename) {
        std::ifstream in(filename);
        in >> json_;
    }

    static bool Register(const std::string& brand, const std::string& model, const URsKineParams& params) {
        auto& models = Instance().models_;
        models[brand][model] = params;
        return true;
    }

    static std::optional<URsKineParams> Get(const std::string& brand, const std::string& model);

    static std::vector<std::pair<std::string, std::string>> SupportedRobots();

private:
    std::unordered_map<std::string, std::unordered_map<std::string, URsKineParams>> models_;
    static nlohmann::json json_;

    URsKineParamsRegistry() = default;

    static URsKineParamsRegistry& Instance() {
        static URsKineParamsRegistry instance;
        return instance;
    }
};

}  // namespace Kinematics
}  // namespace DexeRP
