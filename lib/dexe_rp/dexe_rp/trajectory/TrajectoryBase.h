/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include "dexe_rp/common/CommonHeader.h"
#include "dexe_rp/trajectory/Types.h"

namespace DexeRP {
namespace Trajectory {

template <typename _LieGroup>
class TrajBase {
private:
    using Scalar = typename _LieGroup::Scalar;
    using Tangent = typename _LieGroup::Tangent;

public:
    virtual ~TrajBase() = default;
    // non-copyable to enforce explicit cloning semantics
    TrajBase(const TrajBase&) = delete;
    TrajBase& operator=(const TrajBase&) = delete;
    // movable
    TrajBase(TrajBase&&) noexcept = default;
    TrajBase& operator=(TrajBase&&) noexcept = default;
    // deep copy
    virtual std::shared_ptr<TrajBase<_LieGroup>> Clone() const = 0;

    virtual _LieGroup GetPosition(Scalar t) const = 0;
    virtual Tangent GetVelocity(Scalar t) const = 0;
    virtual Tangent GetAcceleration(Scalar t) const = 0;
    virtual Tangent GetJerk(Scalar t) const = 0;

    Scalar GetDuration() const { return duration_; }

    TrajectoryPoint<_LieGroup> Eval(Scalar t) const {
        return TrajectoryPoint<_LieGroup>{GetPosition(t), GetVelocity(t), GetAcceleration(t), GetJerk(t)};
    }

    std::vector<TrajectoryPoint<_LieGroup>> SampleByStep(Scalar delta_t) const {
        std::vector<TrajectoryPoint<_LieGroup>> samples;
        auto t_samples = DexeRP::SampleByStep(0.0, this->duration_, delta_t);
        samples.reserve(t_samples.size());
        for (const auto& t : t_samples) {
            samples.emplace_back(Eval(t));
        }
        return samples;
    }

    std::vector<TrajectoryPoint<_LieGroup>> SampleByNum(int num_samples) const {
        std::vector<TrajectoryPoint<_LieGroup>> samples;
        auto t_samples = DexeRP::SampleByNum(0.0, this->duration_, num_samples);
        samples.reserve(t_samples.size());
        for (const auto& t : t_samples) {
            samples.emplace_back(Eval(t));
        }
        return samples;
    }

protected:
    explicit TrajBase(Scalar duration) : duration_(duration) {
        DEXERP_ENSURE(duration >= 0.0, "Duration must be non-negative!");
    }

    Scalar duration_{0.0};
};

template <typename _LieGroup>
class CompoundTraj : public TrajBase<_LieGroup> {
private:
    using Scalar = typename _LieGroup::Scalar;
    using Tangent = typename _LieGroup::Tangent;

public:
    using SegmentPtr = std::shared_ptr<TrajBase<_LieGroup>>;
    using SegmentConstPtr = std::shared_ptr<const TrajBase<_LieGroup>>;

    CompoundTraj(const std::vector<SegmentPtr>& segments) : TrajBase<_LieGroup>(0.0) {
        DEXERP_ENSURE(!segments.empty(), "Compound trajectory must have at least one segment!");
        time_prefix_.emplace_back(0.0);
        AddTrajSegments(segments);
    }

    virtual ~CompoundTraj() = default;

    virtual SegmentPtr Clone() const override {
        std::vector<SegmentPtr> segs;
        segs.reserve(segments_.size());
        for (const auto& seg : segments_) {
            segs.push_back(seg->Clone());
        }
        return std::make_shared<CompoundTraj<_LieGroup>>(segs);
    }

    virtual _LieGroup GetPosition(Scalar t) const override {
        auto [segment, local_t] = _GetSegmentAtT(t);
        return segment->GetPosition(local_t);
    }

    virtual Tangent GetVelocity(Scalar t) const override {
        auto [segment, local_t] = _GetSegmentAtT(t);
        return segment->GetVelocity(local_t);
    }

    virtual Tangent GetAcceleration(Scalar t) const override {
        auto [segment, local_t] = _GetSegmentAtT(t);
        return segment->GetAcceleration(local_t);
    }

    virtual Tangent GetJerk(Scalar t) const override {
        auto [segment, local_t] = _GetSegmentAtT(t);
        return segment->GetJerk(local_t);
    }

    const std::vector<SegmentPtr>& GetTrajSegments() const { return segments_; }

    void AddTrajSegment(const SegmentConstPtr& segment) { AddTrajSegments(std::vector<SegmentConstPtr>{segment}); }

    void AddTrajSegment(const SegmentPtr& segment) { AddTrajSegments(std::vector<SegmentPtr>{segment}); }

    void AddTrajSegments(const std::vector<SegmentConstPtr>& segments) {
        std::vector<SegmentPtr> segments_cloned;
        segments_cloned.reserve(segments.size());
        for (const auto& seg : segments) {
            segments_cloned.push_back(seg->Clone());
        }
        return AddTrajSegments(segments_cloned);
    }

    void AddTrajSegments(const std::vector<SegmentPtr>& segments) {
        segments_.reserve(segments_.size() + segments.size());
        time_prefix_.reserve(segments_.size() + segments.size() + 1);
        Scalar total_duration = this->duration_;
        for (auto& seg : segments) {
            total_duration += seg->GetDuration();
            segments_.emplace_back(std::move(seg));
            time_prefix_.emplace_back(total_duration);
        }
        this->duration_ = total_duration;
    }

protected:
    std::pair<SegmentPtr, Scalar> _GetSegmentAtT(Scalar t) const {
        if (this->duration_ <= kEpsilon) return {segments_.front(), 0.0};
        const Scalar t_ = std::clamp(t, 0.0, this->duration_);
        auto it = std::upper_bound(time_prefix_.begin(), time_prefix_.end(), t_);
        size_t idx = std::distance(time_prefix_.begin(), it);
        idx = std::clamp<size_t>(idx, 1, segments_.size()) - 1;
        return {segments_[idx], t_ - time_prefix_[idx]};
    }

    std::vector<SegmentPtr> segments_;
    std::vector<Scalar> time_prefix_;
};

}  // namespace Trajectory
}  // namespace DexeRP
