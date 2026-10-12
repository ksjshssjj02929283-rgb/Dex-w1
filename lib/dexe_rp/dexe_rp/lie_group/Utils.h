/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include "dexe_rp/lie_group/Types.h"

namespace DexeRP {
namespace LieGroup {

template <typename Scalar>
inline manif::SO3<Scalar> SO3FromMat3(const Mat<Scalar, 3, 3>& R) {
    Quat<Scalar> quat(R);
    quat.normalize();
    return manif::SO3<Scalar>(quat.x(), quat.y(), quat.z(), quat.w());
}

template <typename Scalar>
inline manif::SE3<Scalar> SE3FromMat4(const Mat<Scalar, 4, 4>& T) {
    const Mat<Scalar, 3, 3> R = T.template block<3, 3>(0, 0);
    const Vec<Scalar, 3> t = T.template block<3, 1>(0, 3);
    return manif::SE3<Scalar>(t, SO3FromMat3(R));
}

template <typename Scalar>
inline manif::Bundle<Scalar, manif::R3, manif::SO3> R3xSO3FromMat4(const Mat<Scalar, 4, 4>& T) {
    const Mat<Scalar, 3, 3> R = T.template block<3, 3>(0, 0);
    const Vec<Scalar, 3> t = T.template block<3, 1>(0, 3);
    return manif::Bundle<Scalar, manif::R3, manif::SO3>(t, SO3FromMat3(R));
}

template <typename Scalar>
inline manif::Bundle<Scalar, manif::R3, manif::SO3> R3xSO3FromSE3(const manif::SE3<Scalar>& SE3) {
    return manif::Bundle<Scalar, manif::R3, manif::SO3>(SE3.translation(), SE3.rotation());
}

template <typename _LieGroup>
inline Vec<typename _LieGroup::Scalar, _LieGroup::Tangent::DoF> GetWeights() {
    Vec<typename _LieGroup::Scalar, _LieGroup::Tangent::DoF> weights;
    weights.setOnes();
    // if constexpr (std::is_same_v<_LieGroup, R3xSO3d>) {
    //     weights << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;
    // }
    return weights;
}

template <>
inline Vec<double, LieGroup::R3xSO3Tangentd::DoF> GetWeights<LieGroup::R3xSO3d>() {
    Vec<double, LieGroup::R3xSO3Tangentd::DoF> weights;
    weights << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;
    return weights;
}

template <typename _LieGroup>
inline typename _LieGroup::Scalar WeightedNorm(
    const typename _LieGroup::Tangent& tangent,
    const Vec<typename _LieGroup::Scalar, _LieGroup::Tangent::DoF>& weights) {
    const auto& x = tangent.coeffs();
    typename _LieGroup::Scalar weighted_sum = 0.0;
    for (int i = 0; i < _LieGroup::Tangent::DoF; ++i) {
        weighted_sum += weights(i) * x(i) * x(i);
    }
    return std::sqrt(weighted_sum);
}

template <typename _LieGroup>
inline typename _LieGroup::Scalar WeightedNorm(const typename _LieGroup::Tangent& tangent) {
    return WeightedNorm<_LieGroup>(tangent, GetWeights<_LieGroup>());
}

template <>
inline double WeightedNorm<LieGroup::R3xSO3d>(const LieGroup::R3xSO3Tangentd& tangent) {
    double dist_R3 = WeightedNorm<LieGroup::R3xSO3d>(tangent, GetWeights<LieGroup::R3xSO3d>());
    if (dist_R3 < kEpsilon) {
        Vec<double, LieGroup::R3xSO3Tangentd::DoF> weights;
        weights << 0.0, 0.0, 0.0, 1.0, 1.0, 1.0;
        double dist_SO3 = WeightedNorm<LieGroup::R3xSO3d>(tangent, weights);
        return std::max(dist_R3, dist_SO3);
    }
    return dist_R3;
}

}  // namespace LieGroup
}  // namespace DexeRP
