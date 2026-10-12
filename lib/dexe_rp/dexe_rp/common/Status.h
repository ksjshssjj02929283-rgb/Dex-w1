/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <fmt/format.h>

namespace DexeRP {

enum class StatusType : int {
    // common
    Success = 1,

    UnknownFailed = -1,
    InputInvalid = -2,

    // kinematics
    Kine_OutOfWorkspace = -100,
    Kine_OutOfJpsLimits = -101,
    Kine_IKNotContinue = -102,
    Kine_StartPoseNotMatch = -103,
    Kine_NumIKFailed = -104,

    // path
    Path_DenseWaypoints = -200,
    Path_PureSO3 = -201,
    Path_TurnAround = -202,

    // traj
    Traj_DistTooShort = -300
};

}  // namespace DexeRP

template <>
struct fmt::formatter<DexeRP::StatusType> {
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const DexeRP::StatusType& status_type, FormatContext& ctx) const {
        std::string type_str = "UnknownFailed";
        switch (status_type) {
            case DexeRP::StatusType::Success:
                type_str = "Success";
                break;
            case DexeRP::StatusType::InputInvalid:
                type_str = "InputInvalid";
                break;
            case DexeRP::StatusType::Kine_OutOfWorkspace:
                type_str = "Kine_OutOfWorkspace";
                break;
            case DexeRP::StatusType::Kine_OutOfJpsLimits:
                type_str = "Kine_OutOfJpsLimits";
                break;
            case DexeRP::StatusType::Kine_IKNotContinue:
                type_str = "Kine_IKNotContinue";
                break;
            case DexeRP::StatusType::Kine_StartPoseNotMatch:
                type_str = "Kine_StartPoseNotMatch";
                break;
            case DexeRP::StatusType::Kine_NumIKFailed:
                type_str = "Kine_NumIKFailed";
                break;
            case DexeRP::StatusType::Path_DenseWaypoints:
                type_str = "Path_DenseWaypoints";
                break;
            case DexeRP::StatusType::Path_PureSO3:
                type_str = "Path_PureSO3";
                break;
            case DexeRP::StatusType::Path_TurnAround:
                type_str = "Path_TurnAround";
                break;
            case DexeRP::StatusType::Traj_DistTooShort:
                type_str = "Traj_DistTooShort";
                break;
            default:
                break;
        }
        return fmt::format_to(ctx.out(), "StatusType.{}", type_str);
    }
};
