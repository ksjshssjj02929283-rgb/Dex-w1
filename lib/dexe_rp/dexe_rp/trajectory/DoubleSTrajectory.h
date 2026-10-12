/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include "dexe_rp/common/CommonHeader.h"
#include "dexe_rp/path/PathHeader.h"
#include "dexe_rp/trajectory/TrajectoryBase.h"
#include "dexe_rp/trajectory/Types.h"

namespace DexeRP {
namespace Trajectory {

template <typename _LieGroup>
StatusType CreateDoubleSTrajAlongBlendPath(const std::vector<_LieGroup>& waypoints,
                                           Path::BlendType blend_type,
                                           typename _LieGroup::Scalar blend_tolerance,
                                           typename _LieGroup::Scalar path_vel_start,
                                           typename _LieGroup::Scalar path_acc_start,
                                           typename _LieGroup::Scalar path_vel_end,
                                           typename _LieGroup::Scalar path_acc_end,
                                           const TrajectoryConstraint<_LieGroup>& constraint,
                                           std::shared_ptr<Path::CompoundPath<_LieGroup>>& path,
                                           std::shared_ptr<CompoundTraj<_LieGroup>>& traj) {
    int num = waypoints.size();
    if (num < 2) {
        DEXERP_ERROR("At least 2 waypoints are required to create a path!");
        return StatusType::InputInvalid;
    }
    std::vector<TrajectoryConstraint<_LieGroup>> constraints(num, constraint);
    return CreateDoubleSTrajAlongBlendPath<_LieGroup>(waypoints, blend_type, blend_tolerance, path_vel_start,
                                                      path_acc_start, path_vel_end, path_acc_end, constraints, path,
                                                      traj);
}

template <typename _LieGroup>
StatusType CreateDoubleSTrajAlongBlendPath(const std::vector<_LieGroup>& waypoints,
                                           Path::BlendType blend_type,
                                           typename _LieGroup::Scalar blend_tolerance,
                                           typename _LieGroup::Scalar path_vel_start,
                                           typename _LieGroup::Scalar path_acc_start,
                                           typename _LieGroup::Scalar path_vel_end,
                                           typename _LieGroup::Scalar path_acc_end,
                                           const std::vector<TrajectoryConstraint<_LieGroup>>& constraints,
                                           std::shared_ptr<Path::CompoundPath<_LieGroup>>& path,
                                           std::shared_ptr<CompoundTraj<_LieGroup>>& traj);

/* TODO
    1. from traj points to path points(polynomial)
    2. from qpos constraint to traj constraints
    3. fix time (bisection)
*/

}  // namespace Trajectory
}  // namespace DexeRP
