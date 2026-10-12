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
class StateValidatorBase {
public:
    virtual ~StateValidatorBase() = default;
    /**
     * @brief Check the given state (typically joint positions) is valid (e.g., collision-free, within joint limits,
     * etc.)
     *
     * @param state The state vector to be checked, usually representing the planning freedom of the robot (e.g., whole
     * body joint positions, single arm joint positions)
     *
     * @return true if the state is valid, false otherwise
     */
    virtual bool IsStateValid(const CVecXd& state) = 0;

protected:
};

}  // namespace Planner
}  // namespace DexeRP