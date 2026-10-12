/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <fmt/format.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

namespace DexeRP {

// Eigen type alias
/**
 * @NOTE:
 * 1. By default Eigen uses ColumnMajor, here use RowMajor, Be careful about this.
 * 2. Column Vector is prefferred in Robotics.
 * 3. Using DontAlign allows safe appending to STL containers (without aligned_allocato), but loses SIMD improvements.
 * 4. https://github.com/ethz-asl/eigen_catkin/wiki/Eigen-Memory-Issues
 */
template <typename T, int32_t Row, int32_t Col>
using Mat = Eigen::Matrix<T, Row, Col, Eigen::RowMajor | Eigen::DontAlign>;

template <typename T, int32_t Col>
using Vec = Eigen::Matrix<T, 1, Col, Eigen::RowMajor | Eigen::DontAlign>;

template <typename T, int32_t Row>
using CVec = Eigen::Matrix<T, Row, 1, Eigen::ColMajor | Eigen::DontAlign>;

template <typename T>
using Quat = Eigen::Quaternion<T, Eigen::DontAlign>;  // wxyz

template <typename T>
using AngleAxis = Eigen::AngleAxis<T>;

// alias for double type
using Vec2d = Vec<double, 2>;
using Vec3d = Vec<double, 3>;
using Vec4d = Vec<double, 4>;
using Vec6d = Vec<double, 6>;
using Vec7d = Vec<double, 7>;
using VecXd = Vec<double, Eigen::Dynamic>;

using CVec2d = CVec<double, 2>;
using CVec3d = CVec<double, 3>;
using CVec4d = CVec<double, 4>;
using CVec6d = CVec<double, 6>;
using CVec7d = CVec<double, 7>;
using CVecXd = CVec<double, Eigen::Dynamic>;

using Mat2d = Mat<double, 2, 2>;
using Mat3d = Mat<double, 3, 3>;
using Mat4d = Mat<double, 4, 4>;
using Mat6d = Mat<double, 6, 6>;
using MatXd = Mat<double, Eigen::Dynamic, Eigen::Dynamic>;

using Quatd = Quat<double>;
using AngleAxisd = AngleAxis<double>;

// alias for float type
using Vec2f = Vec<float, 2>;
using Vec3f = Vec<float, 3>;
using Vec4f = Vec<float, 4>;
using Vec6f = Vec<float, 6>;
using Vec7f = Vec<float, 7>;
using VecXf = Vec<float, Eigen::Dynamic>;

using CVec2f = CVec<float, 2>;
using CVec3f = CVec<float, 3>;
using CVec4f = CVec<float, 4>;
using CVec6f = CVec<float, 6>;
using CVec7f = CVec<float, 7>;
using CVecXf = CVec<float, Eigen::Dynamic>;

using Mat2f = Mat<float, 2, 2>;
using Mat3f = Mat<float, 3, 3>;
using Mat4f = Mat<float, 4, 4>;
using Mat6f = Mat<float, 6, 6>;
using MatXf = Mat<float, Eigen::Dynamic, Eigen::Dynamic>;

using Quatf = Quat<float>;
using AngleAxisf = AngleAxis<float>;

}  // namespace DexeRP

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(DexeRP::Mat4d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(DexeRP::Mat4f)
