/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include "dexe_rp/common/Types.h"

namespace DexeRP {
namespace Kinematics {

template <typename T>
constexpr Mat<T, 3, 3> Skew(const CVec<T, 3>& vec) noexcept {
    Mat<T, 3, 3> m;
    m << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
    return m;
}

template <typename T>
constexpr Mat<T, 4, 4> DhTransform(T a, T alpha, T d, T theta) noexcept {
    T st = std::sin(theta);
    T ct = std::cos(theta);
    T sa = std::sin(alpha);
    T ca = std::cos(alpha);
    Mat<T, 4, 4> m;
    // clang-format off
    m << ct, -st*ca,  st*sa, a*ct,
         st,  ct*ca, -ct*sa, a*st,
          0,     sa,     ca,    d,
          0,      0,      0,    1;
    // clang-format on
    return m;
}

template <typename T>
constexpr Mat<T, 4, 4> MdhTransform(T a, T alpha, T d, T theta) noexcept {
    T st = std::sin(theta);
    T ct = std::cos(theta);
    T sa = std::sin(alpha);
    T ca = std::cos(alpha);
    Mat<T, 4, 4> m;
    // clang-format off
    m <<    ct,   -st,  0.,     a,
         st*ca, ct*ca, -sa, -d*sa,
         st*sa, ct*sa,  ca,  d*ca,
             0,     0,   0,     1;
    // clang-format on
    return m;
}

}  // namespace Kinematics
}  // namespace DexeRP