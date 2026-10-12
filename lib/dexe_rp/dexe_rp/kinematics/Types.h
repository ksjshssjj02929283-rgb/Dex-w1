/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <fmt/format.h>

#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>

#include "dexe_rp/common/CommonHeader.h"

namespace DexeRP {
namespace Kinematics {

// -------------- Arm Configuration --------------

struct ArmConfig {
    enum class Direction : int { Pos = 1, Neg = -1 };

    static constexpr double ToDouble(Direction d) { return static_cast<double>(static_cast<int>(d)); }

    Direction shoulder{Direction::Pos};
    Direction elbow{Direction::Pos};
    Direction wrist{Direction::Pos};
};

// -------------- Kinematics parameters --------------

template <size_t DoF>
struct ArmCommonParams {
    // joint limits in rad
    CVec<double, DoF> qpos_lb, qpos_ub;

    // joint offsets in rad w.r.t. reference model in along reference model axis
    CVec<double, DoF> qpos_offsets;

    // rotation axis direction w.r.t. reference model: 1 for the same as model, -1 otherwise
    std::array<int, DoF> axis_dirs;

    // base frame -> reference model base frame
    Mat4d T_b_ob;
    // reference model end frame -> flange frame
    Mat4d T_oe_e;

    ArmCommonParams(const CVec<double, DoF>& lb,
                    const CVec<double, DoF>& ub,
                    const CVec<double, DoF>& offsets,
                    const std::array<int, DoF>& axis,
                    const Mat4d& T_b_ob,
                    const Mat4d& T_oe_e)
        : qpos_lb(lb), qpos_ub(ub), qpos_offsets(offsets), axis_dirs(axis), T_b_ob(T_b_ob), T_oe_e(T_oe_e) {}

    static ArmCommonParams FromJson(const nlohmann::json& j) {
        auto JsonToCVec = [](const nlohmann::json& arr) {
            CVec<double, DoF> v = CVec<double, DoF>::Zero();
            if (arr.is_array()) {
                for (size_t i = 0; i < std::min(arr.size(), (size_t)DoF); ++i) {
                    v(i) = arr[i].get<double>();
                }
            }
            return v;
        };
        auto JsonToArray = [](const nlohmann::json& arr) {
            std::array<int, DoF> a{};
            a.fill(1);
            if (arr.is_array()) {
                for (size_t i = 0; i < std::min(arr.size(), (size_t)DoF); ++i) {
                    a[i] = arr[i].get<int>();
                }
            }
            return a;
        };
        auto JsonToMat4 = [](const nlohmann::json& arr) {
            Mat4d m = Mat4d::Identity();
            if (arr.is_array() && arr.size() >= 4) {
                for (int i = 0; i < 4; ++i) {
                    if (arr[i].is_array()) {
                        for (int j = 0; j < std::min((int)arr[i].size(), 4); ++j) {
                            m(i, j) = arr[i][j].get<double>();
                        }
                    }
                }
            }
            return m;
        };
        return ArmCommonParams(JsonToCVec(j.value("qpos_lb", nlohmann::json::array())),
                               JsonToCVec(j.value("qpos_ub", nlohmann::json::array())),
                               JsonToCVec(j.value("qpos_offsets", nlohmann::json::array())),
                               JsonToArray(j.value("axis_dirs", nlohmann::json::array())),
                               JsonToMat4(j.value("T_b_ob", nlohmann::json::array())),
                               JsonToMat4(j.value("T_oe_e", nlohmann::json::array())));
    }

    nlohmann::json ToJson() const {
        auto CVecToJson = [](const CVec<double, DoF>& v) {
            nlohmann::json arr = nlohmann::json::array();
            for (size_t i = 0; i < DoF; ++i) {
                arr.push_back(v(i));
            }
            return arr;
        };
        auto ArrayToJson = [](const std::array<int, DoF>& a) {
            nlohmann::json arr = nlohmann::json::array();
            for (size_t i = 0; i < DoF; ++i) {
                arr.push_back(a[i]);
            }
            return arr;
        };
        auto Mat4ToJson = [](const Mat4d& m) {
            nlohmann::json arr = nlohmann::json::array();
            for (int i = 0; i < 4; ++i) {
                nlohmann::json row = nlohmann::json::array();
                for (int j = 0; j < 4; ++j) {
                    row.push_back(m(i, j));
                }
                arr.push_back(std::move(row));
            }
            return arr;
        };
        return {{"qpos_lb", CVecToJson(qpos_lb)},
                {"qpos_ub", CVecToJson(qpos_ub)},
                {"qpos_offsets", CVecToJson(qpos_offsets)},
                {"axis_dirs", ArrayToJson(axis_dirs)},
                {"T_b_ob", Mat4ToJson(T_b_ob)},
                {"T_oe_e", Mat4ToJson(T_oe_e)}};
    }
};

template <size_t DoF, typename GeometryParams>
struct ArmKineParams {
    using geometry_type = GeometryParams;  // for pybind

    std::string brand;
    std::string model;
    ArmCommonParams<DoF> common;
    GeometryParams geometry;

    static ArmKineParams FromJson(const nlohmann::json& j) {
        return ArmKineParams{j.value("brand", ""), j.value("model", ""),
                             ArmCommonParams<DoF>::FromJson(j.value("common", nlohmann::json::object())),
                             GeometryParams::FromJson(j.value("geometry", nlohmann::json::object()))};
    }

    nlohmann::json ToJson() const {
        return {{"brand", brand}, {"model", model}, {"common", common.ToJson()}, {"geometry", geometry.ToJson()}};
    }
};

// -------------- Kinematics parameters registry --------------

template <typename KineParams>
class KineParamsRegistry {
public:
    static bool Register(const KineParams& params) {
        auto& models = Instance().models_;
        auto& model_map = models[params.brand];
        auto it = model_map.find(params.model);
        if (it != model_map.end()) {
            DEXERP_INFO("Overwriting existing kinematics params: brand='{}', model='{}'", params.brand, params.model);
        }
        model_map.insert_or_assign(params.model, params);
        return true;
    }

    static bool RegisterFromJson(const nlohmann::json& j) {
        if (j.is_array()) {
            bool all_ok = true;
            for (const auto& item : j) {
                if (!Register(KineParams::FromJson(item))) {
                    all_ok = false;
                }
            }
            return all_ok;
        }
        return Register(KineParams::FromJson(j));
    }

    static bool RegisterFromJsonFile(const std::string& file_path) {
        std::ifstream ifs(file_path);
        if (!ifs.is_open()) {
            DEXERP_ERROR("Failed to open JSON file: {}", file_path);
            return false;
        }
        nlohmann::json j;
        ifs >> j;
        return RegisterFromJson(j);
    }

    static std::optional<KineParams> Get(const std::string& brand, const std::string& model) {
        auto& models = Instance().models_;
        auto brand_it = models.find(brand);
        if (brand_it != models.end()) {
            auto model_it = brand_it->second.find(model);
            if (model_it != brand_it->second.end()) {
                return model_it->second;
            }
        }
        DEXERP_ERROR("Kinematics params not found: brand='{}', model='{}'", brand, model);
        return std::nullopt;
    }

    static std::vector<std::pair<std::string, std::string>> SupportedRobots() {
        std::vector<std::pair<std::string, std::string>> robots;
        const auto& models = Instance().models_;
        for (const auto& [brand, model_map] : models) {
            for (const auto& [model, _] : model_map) {
                robots.emplace_back(brand, model);
            }
        }
        return robots;
    }

private:
    using ModelMap = std::unordered_map<std::string, std::unordered_map<std::string, KineParams>>;

    static KineParamsRegistry& Instance() {
        static KineParamsRegistry<KineParams> instance;
        return instance;
    }

    ModelMap models_;
};

}  // namespace Kinematics
}  // namespace DexeRP

template <>
struct fmt::formatter<DexeRP::Kinematics::ArmConfig> {
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const DexeRP::Kinematics::ArmConfig& cfg, FormatContext& ctx) {
        return fmt::format_to(ctx.out(), "ArmConfig({}, {}, {})", cfg.shoulder, cfg.elbow, cfg.wrist);
    }
};

template <size_t DoF, typename GeometryParams>
struct fmt::formatter<DexeRP::Kinematics::ArmKineParams<DoF, GeometryParams>> {
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const DexeRP::Kinematics::ArmKineParams<DoF, GeometryParams>& params, FormatContext& ctx) {
        return fmt::format_to(ctx.out(), "{}", params.ToJson().dump(2));
    }
};
