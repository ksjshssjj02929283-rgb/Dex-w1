/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <algorithm>

#include "dexe_rp/common/CommonHeader.h"
#include "dexe_rp/trajectory/Types.h"

namespace DexeRP {
namespace Trajectory {

class PolynomialBase {
public:
    virtual ~PolynomialBase() = default;
    // non-copyable to enforce explicit cloning semantics
    PolynomialBase(const PolynomialBase&) = delete;
    PolynomialBase& operator=(const PolynomialBase&) = delete;
    // movable
    PolynomialBase(PolynomialBase&&) noexcept = default;
    PolynomialBase& operator=(PolynomialBase&&) noexcept = default;
    // deep copy
    virtual std::shared_ptr<PolynomialBase> Clone() const = 0;

    virtual double GetPosition(double t) const = 0;
    virtual double GetVelocity(double t) const = 0;
    virtual double GetAcceleration(double t) const = 0;
    virtual double GetJerk(double t) const = 0;

    double GetDuration() const { return duration_; }

    PolynomialPoint Eval(double t) const {
        return PolynomialPoint{GetPosition(t), GetVelocity(t), GetAcceleration(t), GetJerk(t)};
    }

    std::vector<PolynomialPoint> SampleByStep(double delta_t) const {
        std::vector<PolynomialPoint> samples;
        auto t_samples = DexeRP::SampleByStep(0.0, this->duration_, delta_t);
        samples.reserve(t_samples.size());
        for (const auto& t : t_samples) {
            samples.emplace_back(Eval(t));
        }
        return samples;
    }

    std::vector<PolynomialPoint> SampleByNum(int num_samples) const {
        std::vector<PolynomialPoint> samples;
        auto t_samples = DexeRP::SampleByNum(0.0, this->duration_, num_samples);
        samples.reserve(t_samples.size());
        for (const auto& t : t_samples) {
            samples.emplace_back(Eval(t));
        }
        return samples;
    }

protected:
    explicit PolynomialBase(double duration) : duration_(duration) {
        DEXERP_ENSURE(duration >= 0.0, "Duration must be non-negative!");
    }

    double duration_{0.0};
};

class CompoundPolynomial : public PolynomialBase {
public:
    CompoundPolynomial(std::vector<std::shared_ptr<PolynomialBase>> segments) : PolynomialBase(0.0) {
        DEXERP_ENSURE(!segments.empty(), "Compound polynomial must have at least one segment!");

        segments_.reserve(segments.size());
        time_prefix_.reserve(segments.size() + 1);
        time_prefix_.emplace_back(0.0);
        double total_duration = 0.0;
        for (const auto& segment : segments) {
            auto seg = segment->Clone();
            total_duration += seg->GetDuration();
            segments_.emplace_back(std::move(seg));
            time_prefix_.emplace_back(total_duration);
        }
        this->duration_ = total_duration;
    }

    virtual ~CompoundPolynomial() = default;

    virtual std::shared_ptr<PolynomialBase> Clone() const override {
        std::vector<std::shared_ptr<PolynomialBase>> segs;
        segs.reserve(segments_.size());
        for (const auto& segment : segments_) {
            segs.push_back(segment->Clone());
        }
        return std::make_shared<CompoundPolynomial>(segs);
    }

    virtual double GetPosition(double t) const override {
        auto [seg, local_t] = _GetSegmentAtT(t);
        return seg->GetPosition(local_t);
    };

    virtual double GetVelocity(double t) const override {
        auto [seg, local_t] = _GetSegmentAtT(t);
        return seg->GetVelocity(local_t);
    };

    virtual double GetAcceleration(double t) const override {
        auto [seg, local_t] = _GetSegmentAtT(t);
        return seg->GetAcceleration(local_t);
    };

    virtual double GetJerk(double t) const override {
        auto [seg, local_t] = _GetSegmentAtT(t);
        return seg->GetJerk(local_t);
    };

    std::vector<std::shared_ptr<PolynomialBase>> GetPolynomialSegments() const { return segments_; }

protected:
    std::pair<std::shared_ptr<PolynomialBase>, double> _GetSegmentAtT(double t) const {
        if (this->duration_ <= kEpsilon) return {segments_.front(), 0.0};
        const double t_ = std::clamp(t, 0.0, this->duration_);
        auto it = std::upper_bound(time_prefix_.begin(), time_prefix_.end(), t_);
        size_t idx = std::distance(time_prefix_.begin(), it);
        idx = std::clamp<size_t>(idx, 1, segments_.size()) - 1;
        return {segments_[idx], t_ - time_prefix_[idx]};
    }

    std::vector<std::shared_ptr<PolynomialBase>> segments_;
    std::vector<double> time_prefix_;
};

class FirstOrderPolynomial : public PolynomialBase {
public:
    FirstOrderPolynomial(PolynomialPoint point0, double duration) : PolynomialBase(duration), pp0_(point0) {}
    virtual ~FirstOrderPolynomial() = default;

    virtual std::shared_ptr<PolynomialBase> Clone() const override {
        return std::make_shared<FirstOrderPolynomial>(pp0_, this->duration_);
    }
    virtual double GetPosition(double t) const override { return pp0_.pos + t * pp0_.vel; };
    virtual double GetVelocity(double t) const override {
        (void)t;
        return pp0_.vel;
    };
    virtual double GetAcceleration(double t) const override {
        (void)t;
        return 0.0;
    };
    virtual double GetJerk(double t) const override {
        (void)t;
        return 0.0;
    };

protected:
    PolynomialPoint pp0_;
};

class CubicPolynomial : public PolynomialBase {
public:
    CubicPolynomial(PolynomialPoint point0, double duration) : PolynomialBase(duration), pp0_(point0) {}
    virtual ~CubicPolynomial() = default;

    virtual std::shared_ptr<PolynomialBase> Clone() const override {
        return std::make_shared<CubicPolynomial>(pp0_, this->duration_);
    }

    virtual double GetPosition(double t) const override {
        const double p = pp0_.pos;
        const double v = pp0_.vel;
        const double a = pp0_.acc;
        const double j = pp0_.jerk;
        return ((j * t / 6.0 + 0.5 * a) * t + v) * t + p;
    };

    virtual double GetVelocity(double t) const override {
        const double v = pp0_.vel;
        const double a = pp0_.acc;
        const double j = pp0_.jerk;
        return (0.5 * j * t + a) * t + v;
    };

    virtual double GetAcceleration(double t) const override {
        const double a = pp0_.acc;
        const double j = pp0_.jerk;
        return j * t + a;
    };

    virtual double GetJerk(double t) const override {
        (void)t;
        return pp0_.jerk;
    };

protected:
    PolynomialPoint pp0_;
};

}  // namespace Trajectory
}  // namespace DexeRP
