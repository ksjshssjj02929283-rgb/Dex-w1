/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

namespace DexeRP {
namespace Path {

template <typename _LieGroup>
struct PathPoint {
    _LieGroup config;
    typename _LieGroup::Tangent tangent;
    typename _LieGroup::Tangent curvature;
    typename _LieGroup::Tangent torsion;

    PathPoint() = default;
    PathPoint(const _LieGroup& config,
              const typename _LieGroup::Tangent& tangent,
              const typename _LieGroup::Tangent& curvature,
              const typename _LieGroup::Tangent& torsion)
        : config(config), tangent(tangent), curvature(curvature), torsion(torsion) {}
};

}  // namespace Path
}  // namespace DexeRP
