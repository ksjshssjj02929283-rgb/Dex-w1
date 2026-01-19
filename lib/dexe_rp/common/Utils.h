/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include "Types.h"
// #include <numbers>

namespace DexeRP {

inline constexpr double PI = 3.14159265358979323846;
// inline constexpr double PI = std::numbers::pi; // TODO: g++ 9.5找不到？

template <typename T>
auto Deg2Rad(const Eigen::MatrixBase<T>& m) {
    return m * PI / 180.0;
    ;
}

template <typename T>
auto Rad2Deg(const Eigen::MatrixBase<T>& m) {
    return m * 180.0 / PI;
}

inline double Deg2Rad(double x) { return x * PI / 180.0; }

inline double Rad2Deg(double x) { return x * 180.0 / PI; }

template <typename T>
inline Mat<T, 3, 3> Skew(const CVec<T, 3>& vec) {
    Mat<T, 3, 3> m;
    m << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
    return m;
}

template <typename T>
inline int Sign(T x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

}  // namespace DexeRP