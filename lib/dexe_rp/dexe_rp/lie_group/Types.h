/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <fmt/format.h>
#include <fmt/ranges.h>

#include "dexe_rp/common/CommonHeader.h"
#include "manif/manif.h"

namespace DexeRP {
namespace LieGroup {

// alias for double type
using R1d = manif::R1d;
using R2d = manif::R2d;
using R3d = manif::R3d;
using R4d = manif::R4d;
using R5d = manif::R5d;
using R6d = manif::R6d;
using R7d = manif::R7d;
using R8d = manif::R8d;
using R9d = manif::R9d;
using R20d = manif::Rn<double, 20>;
using SO3d = manif::SO3d;
using SE3d = manif::SE3d;
using R3xSO3d = manif::Bundle<double, manif::R3, manif::SO3>;

using R1Tangentd = manif::R1Tangentd;
using R2Tangentd = manif::R2Tangentd;
using R3Tangentd = manif::R3Tangentd;
using R4Tangentd = manif::R4Tangentd;
using R5Tangentd = manif::R5Tangentd;
using R6Tangentd = manif::R6Tangentd;
using R7Tangentd = manif::R7Tangentd;
using R8Tangentd = manif::R8Tangentd;
using R9Tangentd = manif::R9Tangentd;
using R20Tangentd = manif::RnTangent<double, 20>;
using SO3Tangentd = manif::SO3Tangentd;
using SE3Tangentd = manif::SE3Tangentd;
using R3xSO3Tangentd = manif::BundleTangent<double, manif::R3, manif::SO3>;

// // alias for float type
// using R1f = manif::R1f;
// using R2f = manif::R2f;
// using R3f = manif::R3f;
// using R4f = manif::R4f;
// using R5f = manif::R5f;
// using R6f = manif::R6f;
// using R7f = manif::R7f;
// using R8f = manif::R8f;
// using R9f = manif::R9f;
// using R20f = manif::Rn<float, 20>;
// using SO3f = manif::SO3f;
// using SE3f = manif::SE3f;
// using R3xSO3f = manif::Bundle<float, manif::R3, manif::SO3>;

// using R1Tangentf = manif::R1Tangentf;
// using R2Tangentf = manif::R2Tangentf;
// using R3Tangentf = manif::R3Tangentf;
// using R4Tangentf = manif::R4Tangentf;
// using R5Tangentf = manif::R5Tangentf;
// using R6Tangentf = manif::R6Tangentf;
// using R7Tangentf = manif::R7Tangentf;
// using R8Tangentf = manif::R8Tangentf;
// using R9Tangentf = manif::R9Tangentf;
// using R20Tangentf = manif::RnTangent<float, 20>;
// using SO3Tangentf = manif::SO3Tangentf;
// using SE3Tangentf = manif::SE3Tangentf;
// using R3xSO3Tangentf = manif::BundleTangent<float, manif::R3, manif::SO3>;

}  // namespace LieGroup
}  // namespace DexeRP

template <typename _Derived>
struct fmt::formatter<_Derived, char, std::enable_if_t<std::is_base_of_v<manif::LieGroupBase<_Derived>, _Derived>>> {
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const _Derived& v, FormatContext& ctx) const {
        return fmt::format_to(ctx.out(), "{}", v.coeffs());
    }
};

template <typename _Derived>
struct fmt::formatter<_Derived, char, std::enable_if_t<std::is_base_of_v<manif::TangentBase<_Derived>, _Derived>>> {
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const _Derived& v, FormatContext& ctx) const {
        return fmt::format_to(ctx.out(), "{}", v.coeffs());
    }
};
