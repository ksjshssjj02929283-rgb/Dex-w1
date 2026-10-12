/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <iostream>
#include <memory>
#include <optional>
#include <vector>

#include "dexe_rp/common/CommonHeader.h"

namespace DexeRP {
namespace Planner {
class StateToPoseBase {
public:
    virtual ~StateToPoseBase() = default;

    /**
     * @brief Get full joints(or the corresponding links) poses in the order of model joints' order from the given state
     * (typically joint positions)
     *
     * @note The input state might be a subset of the full robot joint space, but the returned poses is full body poses
     * in the order of the robot model. The order of the robot model is provided by GetFullJointOrder() API. The state
     * validator will deal with the mapping of different orders.
     *
     * @note The order of joint in the model might be different from the order of real robot.
     */
    virtual bool StateToPose(const CVecXd& state, std::vector<Mat4d>& poses) = 0;

    /**
     * @brief Get the full joint order of the robot model
     */
    virtual const std::vector<std::string> GetFullJointOrder() const = 0;

    /**
     * @brief Get the child link name corresponding to the given joint name
     */
    virtual const std::string GetChildLinkNameFromJointName(const std::string& joint_name) = 0;
};
}  // namespace Planner
}  // namespace DexeRP