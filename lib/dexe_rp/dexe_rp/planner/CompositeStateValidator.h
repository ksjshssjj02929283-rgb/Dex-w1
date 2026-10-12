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
#include "dexe_rp/planner/StateValidatorBase.h"

namespace DexeRP {
namespace Planner {
class CompositeStateValidator final : public StateValidatorBase {
public:
    CompositeStateValidator() = default;

    ~CompositeStateValidator() override = default;

    bool IsStateValid(const CVecXd& state) override;

    bool AddValidator(const std::string& name, std::shared_ptr<StateValidatorBase> validator);

private:
    std::unordered_map<std::string, std::shared_ptr<StateValidatorBase>> validators_;
};
}  // namespace Planner
}  // namespace DexeRP