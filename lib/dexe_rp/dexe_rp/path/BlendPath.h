/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <fmt/format.h>

#include "dexe_rp/common/CommonHeader.h"
#include "dexe_rp/path/PathBase.h"

namespace DexeRP {
namespace Path {

enum class BlendType : int { Bezier2nd = 1, Bezier5th = 2 };

template <typename _LieGroup>
std::vector<std::vector<_LieGroup>> SplitTurnAroundWaypoints(const std::vector<_LieGroup>& waypoints);

template <typename _LieGroup>
StatusType CreateBlendPath(const std::vector<_LieGroup>& waypoints,
                           BlendType blend_type,
                           typename _LieGroup::Scalar blend_tolerance,
                           std::shared_ptr<CompoundPath<_LieGroup>>& path);

}  // namespace Path
}  // namespace DexeRP

template <>
struct fmt::formatter<DexeRP::Path::BlendType> {
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const DexeRP::Path::BlendType& blend_type, FormatContext& ctx) {
        std::string type_str = "Unknown";
        switch (blend_type) {
            case DexeRP::Path::BlendType::Bezier2nd:
                type_str = "Bezier2nd";
                break;
            case DexeRP::Path::BlendType::Bezier5th:
                type_str = "Bezier5th";
                break;
            default:
                break;
        }
        return fmt::format_to(ctx.out(), "BlendType.{}", type_str);
    }
};
