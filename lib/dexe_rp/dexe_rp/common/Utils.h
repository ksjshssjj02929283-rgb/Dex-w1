/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>
#include <utility>
#include <vector>

#include "dexe_rp/common/Logger.h"
#include "dexe_rp/common/Macros.h"
#include "dexe_rp/common/Types.h"

namespace DexeRP {

inline constexpr double kPi = 3.141592653589793238462643383279502884;
inline constexpr double kEpsLarge = 1e-4;
inline constexpr double kEpsilon = 1e-6;
inline constexpr double kEpsSmall = 1e-8;

template <typename Derived>
auto Deg2Rad(const Eigen::MatrixBase<Derived>& m) noexcept {
    using Scalar = typename Derived::Scalar;
    return m * static_cast<Scalar>(kPi / 180.0);
}

template <typename Derived>
auto Rad2Deg(const Eigen::MatrixBase<Derived>& m) noexcept {
    using Scalar = typename Derived::Scalar;
    return m * static_cast<Scalar>(180.0 / kPi);
}

inline constexpr double Deg2Rad(double x) noexcept { return x * kPi / 180.0; }

inline constexpr double Rad2Deg(double x) noexcept { return x * 180.0 / kPi; }

inline constexpr double Sign(double x) noexcept { return (0.0 < x) - (x < 0.0); }

inline double TryWrapToLimits(double q, double lb, double ub) noexcept {
    if (q >= lb && q <= ub) return q;
    double k_min = std::ceil((lb - q) / (2.0 * kPi));
    double k_max = std::floor((ub - q) / (2.0 * kPi));
    if (k_min > k_max) return q;
    double k = k_min;
    return q + k * 2.0 * kPi;
}

template <typename Derived>
auto TryWrapToLimits(const Eigen::MatrixBase<Derived>& q,
                     const Eigen::MatrixBase<Derived>& lb,
                     const Eigen::MatrixBase<Derived>& ub) noexcept {
    auto q_wrapped = q.eval();
    for (Eigen::Index i = 0; i < q.size(); ++i) {
        q_wrapped(i) = TryWrapToLimits(q(i), lb(i), ub(i));
    }
    return q_wrapped;
}

inline double WrapToPi(double q) noexcept {
    return std::remainder(q, 2.0 * kPi);  // [-pi, pi]
}

// [-pi/2, pi/2]
inline double SafeAsin(double x) noexcept { return std::asin(std::clamp(x, -1.0, 1.0)); }

// [0, pi]
inline double SafeAcos(double x) noexcept { return std::acos(std::clamp(x, -1.0, 1.0)); }

inline double SafeSqrt(double x) noexcept { return std::sqrt(std::max(x, 0.0)); }

inline std::vector<double> SampleByStep(double start, double end, double step) {
    std::vector<double> samples;
    if (step <= 0.0) return samples;

    const double length = end - start;
    if (length < kEpsSmall) return {start, end};

    const int num_samples = static_cast<int>(std::ceil(length / step)) + 1;
    samples.reserve(num_samples);
    for (int i = 0; i < num_samples; ++i) {
        double s = start + i * step;
        if (s > end) s = end;
        samples.push_back(s);
    }
    return samples;
}

inline std::vector<double> SampleByNum(double start, double end, int num) {
    if (num <= 2) return {start, end};
    return SampleByStep(start, end, (end - start) / static_cast<double>(num - 1));
}

enum class SolutionStatus { None, Finite, Infinite };

SolutionStatus SolveSinCosEq(double a, double b, double c, std::vector<double>& solutions);

/*
Note (fsh): Use liegroup instead
DexeRP::LieGroup::SO3d temp(xyzw);
or
DexeRP::LieGroup::SO3d temp(DexeRP::Quatd(wxyz));
DexeRP::Mat3d rot = temp.rotation();
*/
inline std::optional<Mat3d> Quat2Rot(const CVec4d& quat) {
    // quat should be in the form of (w, x, y, z)
    if (!std::isfinite(quat(0)) || !std::isfinite(quat(1)) || !std::isfinite(quat(2)) || !std::isfinite(quat(3))) {
        DEXERP_ERROR("Input quaternion contains non-finite values");
        return std::nullopt;
    }
    double norm = quat.norm();

    if (norm < kEpsLarge) {
        DEXERP_ERROR("Input quaternion has near-zero norm ({}). Cannot convert to rotation matrix.", norm);
        return std::nullopt;
    }

    if (std::abs(norm - 1.0) > 1.0) {
        DEXERP_ERROR(
            "Input quaternion has large deviation from unit norm (norm = {}). Please provide a valid quaternion.",
            norm);
        return std::nullopt;
    }

    if (std::abs(norm - 1.0) > kEpsLarge) {
        DEXERP_WARN("Input quaternion is not normalized (norm = {}). Normalizing it.", norm);
    }
    CVec4d q = quat / norm;  // Normalize the quaternion

    double w = q(0);
    double x = q(1);
    double y = q(2);
    double z = q(3);

    Mat3d Rot;
    Rot(0, 0) = 1 - 2 * (y * y + z * z);
    Rot(0, 1) = 2 * (x * y - z * w);
    Rot(0, 2) = 2 * (x * z + y * w);
    Rot(1, 0) = 2 * (x * y + z * w);
    Rot(1, 1) = 1 - 2 * (x * x + z * z);
    Rot(1, 2) = 2 * (y * z - x * w);
    Rot(2, 0) = 2 * (x * z - y * w);
    Rot(2, 1) = 2 * (y * z + x * w);
    Rot(2, 2) = 1 - 2 * (x * x + y * y);

    return Rot;
}

/*
Note (fsh): Use liegroup instead
DexeRP::LieGroup::SO3d temp = SO3FromMat3(Rot);
auto xyzw = temp.quat();
*/
inline std::optional<CVec4d> Rot2Quat(const Mat3d& Rot) {
    if (!Rot.allFinite()) {
        DEXERP_ERROR("Input rotation matrix contains non-finite values");
        return std::nullopt;
    }

    double orth_err = (Rot.transpose() * Rot - Mat3d::Identity()).norm();
    double det_err = std::abs(Rot.determinant() - 1.0);
    if (orth_err > 1e-2 || det_err > 1e-2) {
        DEXERP_ERROR(
            "Input rotation matrix is not valid. It should be orthogonal and have a determinant of 1. Orthogonality "
            "error: {}, "
            "Determinant error: {}.",
            orth_err, det_err);
        return std::nullopt;
    }

    // Use SVD to normalize the rotation matrix
    Eigen::JacobiSVD<Mat3d> svd(Rot, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Mat3d U = svd.matrixU();
    Mat3d V = svd.matrixV();

    Mat3d R = U * V.transpose();
    if (R.determinant() < 0) {
        U.col(2) *= -1;
        R = U * V.transpose();
    }

    double trace = R.trace();
    double w, x, y, z;
    if (trace > 0) {
        w = 0.5 * std::sqrt(trace + 1.0);
        double s = 0.25 / w;
        x = (R(2, 1) - R(1, 2)) * s;
        y = (R(0, 2) - R(2, 0)) * s;
        z = (R(1, 0) - R(0, 1)) * s;
    } else {
        if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
            x = 0.5 * std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2));
            double s = 0.25 / x;
            w = (R(2, 1) - R(1, 2)) * s;
            y = (R(0, 1) + R(1, 0)) * s;
            z = (R(0, 2) + R(2, 0)) * s;
        } else if (R(1, 1) > R(2, 2)) {
            y = 0.5 * std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2));
            double s = 0.25 / y;
            w = (R(0, 2) - R(2, 0)) * s;
            x = (R(0, 1) + R(1, 0)) * s;
            z = (R(1, 2) + R(2, 1)) * s;
        } else {
            z = 0.5 * std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1));
            double s = 0.25 / z;
            w = (R(1, 0) - R(0, 1)) * s;
            x = (R(0, 2) + R(2, 0)) * s;
            y = (R(1, 2) + R(2, 1)) * s;
        }
    }
    CVec4d quat(w, x, y, z);
    quat = quat / quat.norm();
    return quat;
}

/**
 * @brief Convert a pulse value to a joint position value based on the specified limits. This function is useful for
 * converting pulse values from dexterous hand joints to corresponding joint positions in the Pinocchio model.
 *
 * @param pulse The input pulse value to be converted. Typically ranges from 0 to 100, but can be adjusted using
 * pulse limits.
 * @param position_lower_limit The lower limit of the joint position corresponding to the minimum pulse value.
 * @param position_upper_limit The upper limit of the joint position corresponding to the maximum pulse value.
 */
inline double PulseToJointPosition(double pulse,
                                   double position_lower_limit,
                                   double position_upper_limit,
                                   double pulse_lower_limit = 0.0,
                                   double pulse_upper_limit = 100.0) {
    if (pulse_lower_limit >= pulse_upper_limit) {
        DEXERP_ERROR(
            "Pulse lower limit {} is greater than or equal to pulse upper limit {}. Please provide valid pulse "
            "limits.",
            pulse_lower_limit, pulse_upper_limit);
        DEXERP_THROW("Invalid pulse limits.");
    }
    if (pulse < pulse_lower_limit || pulse > pulse_upper_limit) {
        DEXERP_ERROR("Pulse value {} is out of range [{}, {}]. Please provide a valid pulse value.", pulse,
                     pulse_lower_limit, pulse_upper_limit);
        DEXERP_THROW("Invalid pulse value.");
    }
    double ratio = (pulse - pulse_lower_limit) / (pulse_upper_limit - pulse_lower_limit);
    double jps = position_lower_limit + (position_upper_limit - position_lower_limit) * ratio;
    return jps;
}

}  // namespace DexeRP
