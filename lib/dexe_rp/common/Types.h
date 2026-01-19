/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <spdlog/fmt/fmt.h>

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

template <typename T, int32_t Row>
using CVec = Eigen::Matrix<T, Row, 1, Eigen::ColMajor | Eigen::DontAlign>;

template <typename T>
using Quat = Eigen::Quaternion<T, Eigen::DontAlign>;  // wxyz

template <typename T>
using AngleAxis = Eigen::AngleAxis<T>;

// alias for double type
using CVec2d = CVec<double, 2>;
using CVec3d = CVec<double, 3>;
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
using CVec2f = CVec<float, 2>;
using CVec3f = CVec<float, 3>;
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

template <typename T, int32_t Row, int32_t Col>
struct fmt::formatter<DexeRP::Mat<T, Row, Col>> {  // 忽略格式解析
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const DexeRP::Mat<T, Row, Col>& m, FormatContext& ctx) {
        fmt::memory_buffer buf;
        for (int i = 0; i < Row; ++i) {
            buf.push_back('[');
            for (int j = 0; j < Col; ++j) {
                std::string val_str = fmt::format("{}", m(i, j));
                buf.append(val_str.data(), val_str.data() + val_str.size());
                if (j != Col - 1) {
                    buf.push_back(',');
                    buf.push_back(' ');
                }
            }
            buf.push_back(']');
            if (i != Row - 1) {
                buf.push_back(',');
                buf.push_back('\n');
            }
        }
        return fmt::format_to(ctx.out(), "\n[{}]", fmt::to_string(buf));
    }
};

template <typename T, int32_t Row>
struct fmt::formatter<DexeRP::CVec<T, Row>> {
    // 忽略格式解析
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const DexeRP::CVec<T, Row>& v, FormatContext& ctx) {
        fmt::memory_buffer buf;
        for (int i = 0; i < Row; ++i) {
            std::string val_str = fmt::format("{}", v(i));
            buf.append(val_str.data(), val_str.data() + val_str.size());
            if (i != Row - 1) {
                buf.push_back(',');
                buf.push_back(' ');
            }
        }
        return fmt::format_to(ctx.out(), "[{}].T", fmt::to_string(buf));
    }
};
