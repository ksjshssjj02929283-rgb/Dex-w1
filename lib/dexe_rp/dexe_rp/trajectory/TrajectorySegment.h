/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include "dexe_rp/path/PathHeader.h"
#include "dexe_rp/trajectory/Polynomial.h"
#include "dexe_rp/trajectory/TrajectoryBase.h"

namespace DexeRP {
namespace Trajectory {

template <typename _LieGroup>
class TrajSegAlongPath : public TrajBase<_LieGroup> {
    using Scalar = typename _LieGroup::Scalar;
    using Tangent = typename _LieGroup::Tangent;

public:
    TrajSegAlongPath(const std::shared_ptr<Path::PathBase<_LieGroup>>& path,
                     const std::shared_ptr<PolynomialBase>& polynomial)
        : TrajBase<_LieGroup>(0.0) {
        DEXERP_ENSURE(path != nullptr, "Path segment cannot be null!");
        DEXERP_ENSURE(polynomial != nullptr, "Polynomial cannot be null!");
        path_ = std::move(path);
        polynomial_ = std::move(polynomial);
        this->duration_ = polynomial_->GetDuration();
    }

    virtual ~TrajSegAlongPath() = default;

    virtual std::shared_ptr<TrajBase<_LieGroup>> Clone() const override {
        return std::make_shared<TrajSegAlongPath<_LieGroup>>(path_, polynomial_);
    }

    virtual _LieGroup GetPosition(Scalar t) const override {
        const Scalar s = polynomial_->GetPosition(t);
        return path_->GetConfig(s);
    };

    virtual Tangent GetVelocity(Scalar t) const override {
        const Scalar s = polynomial_->GetPosition(t);
        const Scalar ds = polynomial_->GetVelocity(t);
        return path_->GetTangent(s) * ds;
    };

    virtual Tangent GetAcceleration(Scalar t) const override {
        const Scalar s = polynomial_->GetPosition(t);
        const Scalar ds = polynomial_->GetVelocity(t);
        const Scalar dds = polynomial_->GetAcceleration(t);
        return path_->GetTangent(s) * dds + path_->GetCurvature(s) * ds * ds;
    };

    virtual Tangent GetJerk(Scalar t) const override {
        const Scalar s = polynomial_->GetPosition(t);
        const Scalar ds = polynomial_->GetVelocity(t);
        const Scalar dds = polynomial_->GetAcceleration(t);
        const Scalar ddds = polynomial_->GetJerk(t);
        const Tangent tangent = path_->GetTangent(s);
        const Tangent curvature = path_->GetCurvature(s);
        const Tangent torsion = path_->GetTorsion(s);
        return tangent * ddds + (3.0 * curvature * dds + torsion * ds * ds) * ds;
    };

    Scalar GetLength() const { return path_->GetLength(); }
    const Path::PathBase<_LieGroup>& GetPath() const { return *path_; }
    const PolynomialBase& GetPolynomial() const { return *polynomial_; }

protected:
    std::shared_ptr<Path::PathBase<_LieGroup>> path_;  // p(s)
    std::shared_ptr<PolynomialBase> polynomial_;       // s(t)
};

}  // namespace Trajectory
}  // namespace DexeRP
