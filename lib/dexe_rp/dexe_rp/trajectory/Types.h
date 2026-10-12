/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <fmt/format.h>

namespace DexeRP {
namespace Trajectory {

struct SingleSTimeParams {
    double Tj1{0.0};  // (Tj3) time of jerk phase 1
    double Tac{0.0};  // (Tdc) time of acceleration phase
    double Tj2{0.0};  // (Tj4) time of jerk phase 2

    SingleSTimeParams() = default;
    SingleSTimeParams(double tj1, double tac, double tj2) : Tj1(tj1), Tac(tac), Tj2(tj2) {}

    double GetDuration() const { return Tj1 + Tac + Tj2; }
};

struct DoubleSTimeParams {
    double Tj1{0.0};  // time of jerk phase 1
    double Tac{0.0};  // time of acceleration phase
    double Tj2{0.0};  // time of jerk phase 2
    double Tv{0.0};   // time of constant velocity phase
    double Tj3{0.0};  // time of jerk phase 3
    double Tdc{0.0};  // time of deceleration phase
    double Tj4{0.0};  // time of jerk phase 4

    DoubleSTimeParams() = default;
    DoubleSTimeParams(double Tj1, double Tac, double Tj2, double Tv, double Tj3, double Tdc, double Tj4)
        : Tj1(Tj1), Tac(Tac), Tj2(Tj2), Tv(Tv), Tj3(Tj3), Tdc(Tdc), Tj4(Tj4) {}

    double GetDuration() const { return Tj1 + Tac + Tj2 + Tv + Tj3 + Tdc + Tj4; }
    double GetAccDuration() const { return Tj1 + Tac + Tj2; }
    double GetDccDuration() const { return Tj3 + Tdc + Tj4; }
};

struct PolynomialPoint {
    double pos{0.0};
    double vel{0.0};
    double acc{0.0};
    double jerk{0.0};

    PolynomialPoint() = default;
    PolynomialPoint(double pos, double vel, double acc = 0.0, double jerk = 0.0)
        : pos(pos), vel(vel), acc(acc), jerk(jerk) {}
};

struct PolynomialConstraint {
    double max_vel{0.0};
    double max_acc{0.0};
    double max_jerk{0.0};

    PolynomialConstraint() = default;
    PolynomialConstraint(double max_vel, double max_acc, double max_jerk)
        : max_vel(max_vel), max_acc(max_acc), max_jerk(max_jerk) {}
};

template <typename _LieGroup>
struct TrajectoryPoint {
    _LieGroup pos;
    typename _LieGroup::Tangent vel;
    typename _LieGroup::Tangent acc;
    typename _LieGroup::Tangent jerk;

    TrajectoryPoint() = default;
    TrajectoryPoint(const _LieGroup& pos,
                    const typename _LieGroup::Tangent& vel,
                    const typename _LieGroup::Tangent& acc,
                    const typename _LieGroup::Tangent& jerk)
        : pos(pos), vel(vel), acc(acc), jerk(jerk) {}
};

template <typename _LieGroup>
struct TrajectoryConstraint {
    typename _LieGroup::Tangent max_vel;
    typename _LieGroup::Tangent max_acc;
    typename _LieGroup::Tangent max_jerk;

    TrajectoryConstraint() = default;
    TrajectoryConstraint(const typename _LieGroup::Tangent& max_vel,
                         const typename _LieGroup::Tangent& max_acc,
                         const typename _LieGroup::Tangent& max_jerk)
        : max_vel(max_vel), max_acc(max_acc), max_jerk(max_jerk) {}
};

}  // namespace Trajectory
}  // namespace DexeRP

template <>
struct fmt::formatter<DexeRP::Trajectory::SingleSTimeParams> {
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const DexeRP::Trajectory::SingleSTimeParams& p, FormatContext& ctx) {
        return fmt::format_to(ctx.out(), "SingleSTimeParams(Tj1={}, Tac={}, Tj2={})", p.Tj1, p.Tac, p.Tj2);
    }
};

template <>
struct fmt::formatter<DexeRP::Trajectory::DoubleSTimeParams> {
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const DexeRP::Trajectory::DoubleSTimeParams& p, FormatContext& ctx) {
        return fmt::format_to(ctx.out(), "DoubleSTimeParams(Tj1={}, Tac={}, Tj2={}, Tv={}, Tj3={}, Tdc={}, Tj4={})",
                              p.Tj1, p.Tac, p.Tj2, p.Tv, p.Tj3, p.Tdc, p.Tj4);
    }
};

template <>
struct fmt::formatter<DexeRP::Trajectory::PolynomialPoint> {
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const DexeRP::Trajectory::PolynomialPoint& p, FormatContext& ctx) {
        return fmt::format_to(ctx.out(), "PolynomialPoint(pos={}, vel={}, acc={}, jerk={})", p.pos, p.vel, p.acc,
                              p.jerk);
    }
};

template <>
struct fmt::formatter<DexeRP::Trajectory::PolynomialConstraint> {
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const DexeRP::Trajectory::PolynomialConstraint& c, FormatContext& ctx) {
        return fmt::format_to(ctx.out(), "PolynomialConstraint(max_vel={}, max_acc={}, max_jerk={})", c.max_vel,
                              c.max_acc, c.max_jerk);
    }
};
