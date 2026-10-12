/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <optional>

#include "dexe_rp/common/CommonHeader.h"
#include "dexe_rp/trajectory/Polynomial.h"
#include "dexe_rp/trajectory/Types.h"

namespace DexeRP {
namespace Trajectory {

std::pair<StatusType, std::optional<DoubleSTimeParams>> CalcDoubleSTimeParams(PolynomialPoint point0,
                                                                              PolynomialPoint point1,
                                                                              PolynomialConstraint constraint,
                                                                              bool verbose = true);

std::shared_ptr<CompoundPolynomial> CreateDoubleSProfile(PolynomialPoint point0, double jm, DoubleSTimeParams params);

}  // namespace Trajectory
}  // namespace DexeRP
