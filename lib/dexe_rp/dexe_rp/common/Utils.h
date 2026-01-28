/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include "dexe_rp/common/Types.h"
// #include <numbers>

namespace DexeRP {

inline constexpr double kPi = 3.14159265358979323846;  // std::numbers::kPi; g++ 9.5 can not found?
inline constexpr double kEpsLarge = 1e-4;
inline constexpr double kEpsilon = 1e-6;
inline constexpr double kEpsSmall = 1e-8;

template <typename T>
constexpr auto Deg2Rad(const Eigen::MatrixBase<T>& m) noexcept {
    return m * kPi / 180.0;
}

template <typename T>
constexpr auto Rad2Deg(const Eigen::MatrixBase<T>& m) noexcept {
    return m * 180.0 / kPi;
}

inline double Deg2Rad(double x) noexcept { return x * kPi / 180.0; }

inline double Rad2Deg(double x) noexcept { return x * 180.0 / kPi; }

inline double Sign(double x) noexcept {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

inline double WithinPiAndNpi(double q) noexcept {
    if (q < -kPi) {
        return q + 2.0 * kPi;
    } else if (q < kPi) {
        return q;
    } else {
        return q - 2.0 * kPi;
    }
}

}  // namespace DexeRP