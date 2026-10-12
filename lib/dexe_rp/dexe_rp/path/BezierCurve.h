/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include "dexe_rp/common/CommonHeader.h"

namespace DexeRP {
namespace Path {

template <typename _LieGroup>
class BezierCurve {
private:
    using Scalar = typename _LieGroup::Scalar;
    using Tangent = typename _LieGroup::Tangent;

public:
    BezierCurve(const std::vector<_LieGroup>& control_points) : control_points_(control_points) {
        DEXERP_ENSURE(control_points.size() >= 2, "Bezier curve requires at least 2 control points.");

        n_ = control_points_.size() - 1;

        diff_.emplace_back(n_);
        for (size_t i = 0; i < n_; ++i) {
            diff_[0][i] = control_points_[i + 1] - control_points_[i];
        }
        for (size_t k = 1; k < n_; ++k) {
            diff_.emplace_back(n_ - k);
            for (size_t i = 0; i < n_ - k; ++i) {
                diff_[k][i] = diff_[k - 1][i + 1] - diff_[k - 1][i];
            }
        }
    }

    virtual ~BezierCurve() = default;

    std::vector<_LieGroup> GetControlPoints() const { return control_points_; }

    _LieGroup GetPoint(Scalar s) const {
        Scalar s_ = std::clamp(s, 0.0, 1.0);
        auto casteljau_cache_ = control_points_;
        for (size_t k = 1; k <= n_; ++k) {
            const size_t level_size = n_ + 1 - k;
            for (size_t i = 0; i < level_size; ++i) {
                casteljau_cache_[i] += s_ * (casteljau_cache_[i + 1] - casteljau_cache_[i]);
            }
        }
        return casteljau_cache_[0];
    }

    Tangent GetPointDerivative(Scalar s, size_t order) const {
        DEXERP_ENSURE(order >= 1, "Derivative order must >= 1");
        Scalar s_ = std::clamp(s, 0.0, 1.0);

        Tangent out = Tangent::Zero();
        if (order > n_) return out;

        const auto B = _CalcBernsteinBasis(n_ - order, s_);
        const auto& diff = diff_[order - 1];
        Scalar coeff = 1.0;
        for (size_t r = 0; r < order; ++r) coeff *= (n_ - r);
        for (size_t i = 0; i <= n_ - order; ++i) {
            out += (coeff * B[i]) * diff[i];
        }
        return out;
    }

protected:
    std::vector<Scalar> _CalcBernsteinBasis(int n, Scalar s) const {
        std::vector<Scalar> B(n + 1);
        if (n == 0 || s <= 0.0) {
            B[0] = 1.0;
            return B;
        }

        if (s >= 1.0) {
            B[n] = 1.0;
            return B;
        }

        Scalar u = 1.0 - s;
        B[0] = std::pow(u, n);
        Scalar ratio = s / u;
        for (int k = 0; k < n; ++k) {
            B[k + 1] = B[k] * (Scalar)(n - k) / (k + 1) * ratio;
        }
        return B;
    }

    size_t n_{0};                             // degree of the Bezier curve
    std::vector<_LieGroup> control_points_;   // control points of this segment
    std::vector<std::vector<Tangent>> diff_;  // precomputed
};

}  // namespace Path
}  // namespace DexeRP
