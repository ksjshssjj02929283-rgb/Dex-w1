/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include "dexe_rp/common/CommonHeader.h"
#include "dexe_rp/path/BezierCurve.h"
#include "dexe_rp/path/PathBase.h"

namespace DexeRP {
namespace Path {

template <typename _LieGroup>
struct BezierResult {
    bool is_success{false};
    std::vector<_LieGroup> control_points;
    double length{0.0};

    BezierResult(bool is_success, std::vector<_LieGroup> control_points, double length)
        : is_success(is_success), control_points(std::move(control_points)), length(length) {}
};

template <typename _LieGroup>
BezierResult<_LieGroup> Calc5thBezierControlPoints(const std::array<_LieGroup, 3>& waypoints);

template <typename _LieGroup>
class PathSegBezier : public PathBase<_LieGroup> {
private:
    using Scalar = typename _LieGroup::Scalar;
    using Tangent = typename _LieGroup::Tangent;

public:
    PathSegBezier(const std::vector<_LieGroup>& control_points, Scalar length)
        : PathBase<_LieGroup>(length), bezier_curve_(control_points) {}

    virtual ~PathSegBezier() = default;

    virtual std::shared_ptr<PathBase<_LieGroup>> Clone() const override {
        return std::make_shared<PathSegBezier<_LieGroup>>(bezier_curve_.GetControlPoints(), this->length_);
    }

    virtual _LieGroup GetConfig(Scalar s) const override {
        const Scalar s_local = this->_GetLocalS(s);
        return bezier_curve_.GetPoint(s_local);
    }

    virtual Tangent GetTangent(Scalar s) const override {
        const Scalar s_local = this->_GetLocalS(s);
        return bezier_curve_.GetPointDerivative(s_local, 1) / this->length_;
    }

    virtual Tangent GetCurvature(Scalar s) const override {
        const Scalar s_local = this->_GetLocalS(s);
        return bezier_curve_.GetPointDerivative(s_local, 2) / (this->length_ * this->length_);
    }

    virtual Tangent GetTorsion(Scalar s) const override {
        const Scalar s_local = this->_GetLocalS(s);
        return bezier_curve_.GetPointDerivative(s_local, 3) / (this->length_ * this->length_ * this->length_);
    }

protected:
    inline Scalar _GetLocalS(Scalar s_global) const {
        if (this->length_ < kEpsilon) return 0.0;
        return std::clamp(s_global / this->length_, 0.0, 1.0);
    }

    BezierCurve<_LieGroup> bezier_curve_;
};

}  // namespace Path
}  // namespace DexeRP

template <typename _LieGroup>
struct fmt::formatter<DexeRP::Path::BezierResult<_LieGroup>> {
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const DexeRP::Path::BezierResult<_LieGroup>& v, FormatContext& ctx) const {
        return fmt::format_to(ctx.out(), "BezierResult({}, {}, [{}])", v.is_success, v.length,
                              fmt::join(v.control_points, ", "));
    }
};
