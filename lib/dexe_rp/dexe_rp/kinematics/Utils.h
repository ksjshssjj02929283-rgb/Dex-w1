/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include "dexe_rp/common/CommonHeader.h"
#include "dexe_rp/common/Types.h"

namespace DexeRP {
namespace Kinematics {

template <typename T>
Mat<T, 3, 3> Skew(const CVec<T, 3>& vec) noexcept {
    Mat<T, 3, 3> m;
    // clang-format off
    m << 0, -vec(2), vec(1), 
        vec(2), 0, -vec(0), 
        -vec(1), vec(0), 0;
    // clang-format on
    return m;
}

template <typename T>
Mat<T, 4, 4> DhTransform(T a, T alpha, T d, T theta) noexcept {
    const T st = std::sin(theta);
    const T ct = std::cos(theta);
    const T sa = std::sin(alpha);
    const T ca = std::cos(alpha);
    Mat<T, 4, 4> m;
    // clang-format off
    m << ct, -st*ca,  st*sa, a*ct,
         st,  ct*ca, -ct*sa, a*st,
          0,     sa,     ca,    d,
          0,      0,      0,    1;
    // clang-format on
    return m;
}

template <typename T>
Mat<T, 4, 4> MdhTransform(T a, T alpha, T d, T theta) noexcept {
    const T st = std::sin(theta);
    const T ct = std::cos(theta);
    const T sa = std::sin(alpha);
    const T ca = std::cos(alpha);
    Mat<T, 4, 4> m;
    // clang-format off
    m <<    ct,   -st,  0.,     a,
         st*ca, ct*ca, -sa, -d*sa,
         st*sa, ct*sa,  ca,  d*ca,
             0,     0,   0,     1;
    // clang-format on
    return m;
}

template <size_t DoF>
CVec<double, DoF> ToModelQpos(const CVec<double, DoF>& qpos,
                              const CVec<double, DoF>& qpos_offsets,
                              const std::array<int, DoF>& axis_dirs,
                              const CVec<double, DoF>& model_qpos_lb,
                              const CVec<double, DoF>& model_qpos_ub) {
    CVec<double, DoF> model_qpos;
    for (size_t i = 0; i < DoF; ++i) {
        double q = qpos(i) * static_cast<double>(axis_dirs[i]) + qpos_offsets(i);
        model_qpos(i) = TryWrapToLimits(q, model_qpos_lb(i), model_qpos_ub(i));
    }
    return model_qpos;
}

template <size_t DoF>
CVec<double, DoF> ToUserQpos(const CVec<double, DoF>& model_qpos,
                             const CVec<double, DoF>& qpos_offsets,
                             const std::array<int, DoF>& axis_dirs,
                             const CVec<double, DoF>& qpos_lb,
                             const CVec<double, DoF>& qpos_ub) {
    CVec<double, DoF> user_qpos;
    for (size_t i = 0; i < DoF; ++i) {
        double q = static_cast<double>(axis_dirs[i]) * (model_qpos(i) - qpos_offsets(i));
        user_qpos(i) = TryWrapToLimits(q, qpos_lb(i), qpos_ub(i));
    }
    return user_qpos;
}

template <size_t DoF>
std::pair<CVec<double, DoF>, CVec<double, DoF>> ToModelQposLimits(const CVec<double, DoF>& qpos_lb,
                                                                  const CVec<double, DoF>& qpos_ub,
                                                                  const CVec<double, DoF>& qpos_offsets,
                                                                  const std::array<int, DoF>& axis_dirs) {
    CVec<double, DoF> model_qpos_lb, model_qpos_ub;
    for (size_t i = 0; i < DoF; ++i) {
        double lb = qpos_lb(i) * static_cast<double>(axis_dirs[i]) + qpos_offsets(i);
        double ub = qpos_ub(i) * static_cast<double>(axis_dirs[i]) + qpos_offsets(i);
        if (lb > ub) std::swap(lb, ub);
        model_qpos_lb(i) = lb;
        model_qpos_ub(i) = ub;
    }
    return std::make_pair(model_qpos_lb, model_qpos_ub);
}

template <size_t DoF>
double CalcDistanceCost(const CVec<double, DoF>& qpos,
                        const CVec<double, DoF>& ref_qpos,
                        const CVec<double, DoF>& qpos_lb,
                        const CVec<double, DoF>& qpos_ub,
                        const CVec<double, DoF>& weights) {
    auto norm_diff = (qpos - ref_qpos).array() / (qpos_ub - qpos_lb).array();
    return (weights.array() * norm_diff.abs()).sum();
}

template <size_t DoF>
double CalcLimitCost(const CVec<double, DoF>& qpos,
                     const CVec<double, DoF>& qpos_lb,
                     const CVec<double, DoF>& qpos_ub,
                     const CVec<double, DoF>& weights,
                     double penalty_bound = 0.95) {
    CVec<double, DoF> mid = 0.5 * (qpos_lb + qpos_ub);
    CVec<double, DoF> half_range = 0.5 * (qpos_ub - qpos_lb);
    auto norm_diff = (qpos - mid).array().abs() / half_range.array();
    auto cost = (norm_diff.array() <= penalty_bound)
                    .select(0.1 * norm_diff.array(), 10.0 * (norm_diff.array() - penalty_bound).exp());
    return (weights.array() * cost.array()).sum();
}

/// NOTE: For SRS and FEP only
double CalcSingularityCost(const CVec7d& qpos, double penalty_bound = Deg2Rad(5.0));

std::vector<std::vector<double>> GenerateRadialLayerSamples(
    double ref, double lb, double ub, double half_layer_step, double sample_step);

}  // namespace Kinematics
}  // namespace DexeRP
